// =============================================================================
// micro-ROS AMR Firmware mit Odometrie
// Version: 2.1.0 | Phase 3.3
// =============================================================================
//
// Topics:
//   Publisher:  /odom              (nav_msgs/Odometry)   - Position & Velocity
//   Publisher:  /esp32/heartbeat   (std_msgs/Int32)      - Watchdog
//   Subscriber: /cmd_vel           (geometry_msgs/Twist) - Motorsteuerung
//   Subscriber: /esp32/led_cmd     (std_msgs/Bool)       - LED-Steuerung
//
// Hardware: Seeed Studio XIAO ESP32-S3 + Cytron MDD3A + JGA25-370 Encoder
// =============================================================================

#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>

#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/int32.h>

#include "config.h"

// =============================================================================
// Pin-Definitionen
// =============================================================================
#define LED_PIN 21 // Onboard LED (active LOW)

// =============================================================================
// micro-ROS Objekte
// =============================================================================
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// Publisher
rcl_publisher_t pub_heartbeat;
rcl_publisher_t pub_odom;
std_msgs__msg__Int32 msg_heartbeat;
nav_msgs__msg__Odometry msg_odom;

// Subscribers
rcl_subscription_t sub_cmd_vel;
rcl_subscription_t sub_led_cmd;
geometry_msgs__msg__Twist msg_cmd_vel;
std_msgs__msg__Bool msg_led_cmd;

// Executor
rclc_executor_t executor;

// =============================================================================
// Encoder (volatile für ISR)
// =============================================================================
volatile long encoder_ticks_left = 0;
volatile long encoder_ticks_right = 0;

// Vorherige Werte für Delta-Berechnung
long prev_ticks_left = 0;
long prev_ticks_right = 0;

// =============================================================================
// Geschwindigkeitsmessung
// =============================================================================
float velocity_left = 0.0f;  // [m/s]
float velocity_right = 0.0f; // [m/s]

// =============================================================================
// Odometrie
// =============================================================================
float odom_x = 0.0f;
float odom_y = 0.0f;
float odom_theta = 0.0f;

// =============================================================================
// Motorsteuerung
// =============================================================================
volatile float target_linear = 0.0f;  // m/s
volatile float target_angular = 0.0f; // rad/s
unsigned long last_cmd_time = 0;
int32_t heartbeat_counter = 0;

// =============================================================================
// Timing
// =============================================================================
unsigned long last_loop_time = 0;
unsigned long last_odom_time = 0;

// =============================================================================
// Interrupt Service Routines
// =============================================================================

void IRAM_ATTR isr_encoder_left() { encoder_ticks_left++; }

void IRAM_ATTR isr_encoder_right() { encoder_ticks_right++; }

// =============================================================================
// Hardware Abstraction Layer
// =============================================================================

void hal_encoder_init() {
    pinMode(PIN_ENC_LEFT_A, INPUT_PULLUP);
    pinMode(PIN_ENC_RIGHT_A, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(PIN_ENC_LEFT_A), isr_encoder_left,
                    RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_RIGHT_A), isr_encoder_right,
                    RISING);
}

void hal_encoder_read(long *left, long *right) {
    noInterrupts();
    *left = encoder_ticks_left;
    *right = encoder_ticks_right;
    interrupts();
}

void setupMotors() {
    ledcSetup(PWM_CH_LEFT_A, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
    ledcSetup(PWM_CH_LEFT_B, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
    ledcSetup(PWM_CH_RIGHT_A, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
    ledcSetup(PWM_CH_RIGHT_B, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);

    ledcAttachPin(PIN_MOTOR_LEFT_A, PWM_CH_LEFT_A);
    ledcAttachPin(PIN_MOTOR_LEFT_B, PWM_CH_LEFT_B);
    ledcAttachPin(PIN_MOTOR_RIGHT_A, PWM_CH_RIGHT_A);
    ledcAttachPin(PIN_MOTOR_RIGHT_B, PWM_CH_RIGHT_B);

    ledcWrite(PWM_CH_LEFT_A, 0);
    ledcWrite(PWM_CH_LEFT_B, 0);
    ledcWrite(PWM_CH_RIGHT_A, 0);
    ledcWrite(PWM_CH_RIGHT_B, 0);
}

void stopMotors() {
    ledcWrite(PWM_CH_LEFT_A, 0);
    ledcWrite(PWM_CH_LEFT_B, 0);
    ledcWrite(PWM_CH_RIGHT_A, 0);
    ledcWrite(PWM_CH_RIGHT_B, 0);
}

uint8_t applyDeadzone(float speed_normalized) {
    if (fabs(speed_normalized) < 0.01f) {
        return 0;
    }
    uint8_t pwm = (uint8_t)(PWM_DEADZONE + fabs(speed_normalized) *
                                               (MOTOR_PWM_MAX - PWM_DEADZONE));
    return min(pwm, (uint8_t)MOTOR_PWM_MAX);
}

void setMotorLeft(float speed) {
    uint8_t pwm = applyDeadzone(speed);

    if (speed > 0.01f) {
        ledcWrite(PWM_CH_LEFT_A, pwm);
        ledcWrite(PWM_CH_LEFT_B, 0);
    } else if (speed < -0.01f) {
        ledcWrite(PWM_CH_LEFT_A, 0);
        ledcWrite(PWM_CH_LEFT_B, pwm);
    } else {
        ledcWrite(PWM_CH_LEFT_A, 0);
        ledcWrite(PWM_CH_LEFT_B, 0);
    }
}

void setMotorRight(float speed) {
    uint8_t pwm = applyDeadzone(speed);

    if (speed > 0.01f) {
        ledcWrite(PWM_CH_RIGHT_A, pwm);
        ledcWrite(PWM_CH_RIGHT_B, 0);
    } else if (speed < -0.01f) {
        ledcWrite(PWM_CH_RIGHT_A, 0);
        ledcWrite(PWM_CH_RIGHT_B, pwm);
    } else {
        ledcWrite(PWM_CH_RIGHT_A, 0);
        ledcWrite(PWM_CH_RIGHT_B, 0);
    }
}

// =============================================================================
// Geschwindigkeitsberechnung (aus Serial-Bridge übernommen)
// =============================================================================

void velocity_update(float dt) {
    long ticks_left, ticks_right;
    hal_encoder_read(&ticks_left, &ticks_right);

    long delta_left = ticks_left - prev_ticks_left;
    long delta_right = ticks_right - prev_ticks_right;

    prev_ticks_left = ticks_left;
    prev_ticks_right = ticks_right;

    // Sanity Check
    if (abs(delta_left) > MAX_TICK_DELTA || abs(delta_right) > MAX_TICK_DELTA) {
        return;
    }

    // Geschwindigkeit [m/s] = Strecke / Zeit
    if (dt > 0.001f) {
        float dist_left = delta_left * METERS_PER_TICK_LEFT;
        float dist_right = delta_right * METERS_PER_TICK_RIGHT;

        // Tiefpass-Filter für Rauschunterdrückung
        const float alpha = 0.3f;
        velocity_left =
            alpha * (dist_left / dt) + (1.0f - alpha) * velocity_left;
        velocity_right =
            alpha * (dist_right / dt) + (1.0f - alpha) * velocity_right;
    }
}

// =============================================================================
// Odometrie (aus Serial-Bridge übernommen)
// =============================================================================

void odometry_update(float dt) {
    float d_left = velocity_left * dt;
    float d_right = velocity_right * dt;

    float d_center = (d_left + d_right) / 2.0f;
    float d_theta = (d_right - d_left) / WHEEL_BASE;

    odom_x += d_center * cos(odom_theta + d_theta / 2.0f);
    odom_y += d_center * sin(odom_theta + d_theta / 2.0f);
    odom_theta += d_theta;

    // Normalisieren auf [-π, π]
    while (odom_theta > PI)
        odom_theta -= 2.0f * PI;
    while (odom_theta < -PI)
        odom_theta += 2.0f * PI;
}

// =============================================================================
// Differential Drive Kinematik
// =============================================================================

void updateMotors() {
    float v = constrain(target_linear, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED);
    float w = constrain(target_angular, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED);

    // v_left  = v - (ω * L/2)
    // v_right = v + (ω * L/2)
    float v_left = v - (w * WHEEL_BASE / 2.0f);
    float v_right = v + (w * WHEEL_BASE / 2.0f);

    // Normalisieren
    float v_left_norm = v_left / MAX_LINEAR_SPEED;
    float v_right_norm = v_right / MAX_LINEAR_SPEED;

    v_left_norm = constrain(v_left_norm, -1.0f, 1.0f);
    v_right_norm = constrain(v_right_norm, -1.0f, 1.0f);

    setMotorLeft(v_left_norm);
    setMotorRight(v_right_norm);
}

// =============================================================================
// Callbacks
// =============================================================================

void cmd_vel_callback(const void *msgin) {
    const geometry_msgs__msg__Twist *msg =
        (const geometry_msgs__msg__Twist *)msgin;
    target_linear = msg->linear.x;
    target_angular = msg->angular.z;
    last_cmd_time = millis();
}

void led_callback(const void *msgin) {
    const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool *)msgin;
    digitalWrite(PIN_LED_MOSFET, msg->data ? HIGH : LOW);
}

// =============================================================================
// Failsafe
// =============================================================================

void checkFailsafe() {
    if (millis() - last_cmd_time > FAILSAFE_TIMEOUT_MS) {
        target_linear = 0.0f;
        target_angular = 0.0f;
        stopMotors();
    }
}

// =============================================================================
// Odometry Message erstellen
// =============================================================================

void publish_odometry() {
    // Header
    msg_odom.header.stamp.sec = millis() / 1000;
    msg_odom.header.stamp.nanosec = (millis() % 1000) * 1000000;
    // frame_id wird in setup() gesetzt

    // Position
    msg_odom.pose.pose.position.x = odom_x;
    msg_odom.pose.pose.position.y = odom_y;
    msg_odom.pose.pose.position.z = 0.0;

    // Orientation (Quaternion aus theta)
    // q = [cos(θ/2), 0, 0, sin(θ/2)] für Rotation um Z
    msg_odom.pose.pose.orientation.x = 0.0;
    msg_odom.pose.pose.orientation.y = 0.0;
    msg_odom.pose.pose.orientation.z = sin(odom_theta / 2.0);
    msg_odom.pose.pose.orientation.w = cos(odom_theta / 2.0);

    // Velocity (im base_link Frame)
    msg_odom.twist.twist.linear.x = (velocity_left + velocity_right) / 2.0;
    msg_odom.twist.twist.linear.y = 0.0;
    msg_odom.twist.twist.linear.z = 0.0;
    msg_odom.twist.twist.angular.x = 0.0;
    msg_odom.twist.twist.angular.y = 0.0;
    msg_odom.twist.twist.angular.z =
        (velocity_right - velocity_left) / WHEEL_BASE;

    // Covariance (diagonal, vereinfacht)
    // Pose covariance [x, y, z, roll, pitch, yaw]
    for (int i = 0; i < 36; i++) {
        msg_odom.pose.covariance[i] = 0.0;
        msg_odom.twist.covariance[i] = 0.0;
    }
    msg_odom.pose.covariance[0] = 0.01;   // x
    msg_odom.pose.covariance[7] = 0.01;   // y
    msg_odom.pose.covariance[35] = 0.03;  // yaw
    msg_odom.twist.covariance[0] = 0.01;  // vx
    msg_odom.twist.covariance[35] = 0.03; // vyaw

    (void)rcl_publish(&pub_odom, &msg_odom, NULL);
}

// =============================================================================
// LED Helper
// =============================================================================

void setStatusLED(bool on) { digitalWrite(LED_PIN, on ? LOW : HIGH); }

// =============================================================================
// Setup
// =============================================================================

void setup() {
    // Status LED
    pinMode(LED_PIN, OUTPUT);
    setStatusLED(false);

    // MOSFET LED
    pinMode(PIN_LED_MOSFET, OUTPUT);
    digitalWrite(PIN_LED_MOSFET, LOW);

    // Hardware initialisieren
    setupMotors();
    hal_encoder_init();

    // USB Serial
    Serial.begin(115200);
    delay(2000);

    set_microros_serial_transports(Serial);

    allocator = rcl_get_default_allocator();

    // Warte auf Agent
    while (RMW_RET_OK != rmw_uros_ping_agent(100, 1)) {
        setStatusLED((millis() / 250) % 2);
        delay(100);
    }
    setStatusLED(true);

    // Support & Node
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "esp32_node", "", &support);

    // Publisher: Heartbeat
    rclc_publisher_init_best_effort(
        &pub_heartbeat, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "esp32/heartbeat");

    // Publisher: Odometry
    rclc_publisher_init_default(
        &pub_odom, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "odom");

    // Frame IDs setzen (statische Strings)
    msg_odom.header.frame_id.data = (char *)"odom";
    msg_odom.header.frame_id.size = 4;
    msg_odom.header.frame_id.capacity = 5;
    msg_odom.child_frame_id.data = (char *)"base_link";
    msg_odom.child_frame_id.size = 9;
    msg_odom.child_frame_id.capacity = 10;

    // Subscriber: cmd_vel
    rclc_subscription_init_best_effort(
        &sub_cmd_vel, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel");

    // Subscriber: LED
    rclc_subscription_init_default(
        &sub_led_cmd, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "esp32/led_cmd");

    // Executor mit 2 Subscriptions
    rclc_executor_init(&executor, &support.context, 2, &allocator);
    rclc_executor_add_subscription(&executor, &sub_cmd_vel, &msg_cmd_vel,
                                   &cmd_vel_callback, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &sub_led_cmd, &msg_led_cmd,
                                   &led_callback, ON_NEW_DATA);

    // Timing initialisieren
    last_cmd_time = millis();
    last_loop_time = millis();
    last_odom_time = millis();
}

// =============================================================================
// Loop
// =============================================================================

void loop() {
    unsigned long now = millis();

    // micro-ROS Executor
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

    // Hauptschleife (100 Hz)
    if (now - last_loop_time >= LOOP_PERIOD_MS) {
        float dt = (now - last_loop_time) / 1000.0f;
        last_loop_time = now;

        // Geschwindigkeit messen
        velocity_update(dt);

        // Odometrie aktualisieren
        odometry_update(dt);

        // Failsafe
        checkFailsafe();

        // Motoren
        updateMotors();
    }

    // Odometrie publizieren (50 Hz)
    if (now - last_odom_time >= ODOM_PERIOD_MS) {
        last_odom_time = now;
        publish_odometry();
    }

    // Heartbeat (1 Hz)
    static unsigned long last_heartbeat = 0;
    if (now - last_heartbeat > 1000) {
        msg_heartbeat.data = heartbeat_counter++;
        (void)rcl_publish(&pub_heartbeat, &msg_heartbeat, NULL);
        last_heartbeat = now;
    }
}
