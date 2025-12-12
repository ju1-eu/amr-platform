/**
 * @file main.cpp
 * @brief AMR Firmware Phase 1: Basic Motion & Safety
 * @version 1.1.0
 * @date 2025-12-12
 * @author Jan Unger
 * 
 * @details 
 * Implementiert FreeRTOS Tasks für Kommunikation und Aktorik.
 * Integriert Failsafe-Mechanismen, Thread-Safety und visuelles Feedback.
 * 
 * @hardware
 * - Seeed Studio XIAO ESP32-S3
 * - Cytron MDD3A Motortreiber (Dual-PWM Mode)
 * - JGA25-370 DC-Motoren mit Hall-Encoder
 * 
 * @standards
 * - REP-103: SI-Einheiten (Meter, Radiant)
 * - REP-105: TF-Frames (odom → base_link) [Phase 2]
 * - Safety: Failsafe Timeout, Task Watchdog
 * 
 * @phase Phase 1 - Open Loop Control
 * @todo Phase 2: Encoder-Integration, Odometrie, PID-Regelung
 */

#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>

// ESP32-spezifisch
#include <esp_task_wdt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#include "config.h"

// ==========================================================================
// GLOBALE OBJEKTE & VARIABLEN
// ==========================================================================

// --- micro-ROS Entities ---
rcl_subscription_t cmd_vel_sub;
geometry_msgs__msg__Twist twist_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// --- Thread-Safety: Mutex für geteilte Variablen ---
SemaphoreHandle_t cmd_mutex;

// --- Steuerungsvariablen (geschützt durch cmd_mutex) ---
typedef struct {
    float linear_x;      // [m/s] Vorwärtsgeschwindigkeit
    float angular_z;     // [rad/s] Drehgeschwindigkeit
    uint32_t timestamp;  // [ms] Zeitstempel letzter Empfang
} CommandState;

volatile CommandState cmd_state = {0.0f, 0.0f, 0};

// --- Encoder-Zähler (Phase 2) ---
volatile int32_t encoder_left_ticks = 0;
volatile int32_t encoder_right_ticks = 0;

// --- System States ---
typedef enum {
    STATE_BOOTING,
    STATE_WAITING_FOR_AGENT,
    STATE_CONNECTED,
    STATE_FAILSAFE,
    STATE_ERROR
} SystemState;

volatile SystemState system_state = STATE_BOOTING;

// ==========================================================================
// HARDWARE ABSTRACTION LAYER (HAL)
// ==========================================================================

/**
 * @brief Initialisiert alle PWM-Kanäle für Motoren und LED
 */
void hal_pwm_init() {
    // Motor PWM Kanäle (20 kHz, 8-bit)
    ledcSetup(PWM_CH_LEFT_A, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
    ledcSetup(PWM_CH_LEFT_B, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
    ledcSetup(PWM_CH_RIGHT_A, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
    ledcSetup(PWM_CH_RIGHT_B, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
    
    // Pins zuweisen
    ledcAttachPin(PIN_MOTOR_LEFT_A, PWM_CH_LEFT_A);
    ledcAttachPin(PIN_MOTOR_LEFT_B, PWM_CH_LEFT_B);
    ledcAttachPin(PIN_MOTOR_RIGHT_A, PWM_CH_RIGHT_A);
    ledcAttachPin(PIN_MOTOR_RIGHT_B, PWM_CH_RIGHT_B);
    
    // LED PWM
    ledcSetup(LED_PWM_CHANNEL, LED_PWM_FREQ, LED_PWM_BITS);
    ledcAttachPin(PIN_LED_MOSFET, LED_PWM_CHANNEL);
    
    // Sicherer Startzustand: Alle Motoren aus
    ledcWrite(PWM_CH_LEFT_A, 0);
    ledcWrite(PWM_CH_LEFT_B, 0);
    ledcWrite(PWM_CH_RIGHT_A, 0);
    ledcWrite(PWM_CH_RIGHT_B, 0);
}

/**
 * @brief Steuert einen Motor über Cytron MDD3A (Dual-PWM)
 * 
 * @param ch_a PWM-Kanal A (Vorwärts)
 * @param ch_b PWM-Kanal B (Rückwärts)
 * @param speed Geschwindigkeit [-1.0 ... +1.0]
 * 
 * @note MDD3A Logik:
 *       - A>0, B=0: Vorwärts
 *       - A=0, B>0: Rückwärts
 *       - A=0, B=0: Coast
 *       - A>0, B>0: Active Brake
 */
void hal_motor_set(uint8_t ch_a, uint8_t ch_b, float speed) {
    // Clamp auf [-1.0, +1.0]
    speed = constrain(speed, -1.0f, 1.0f);
    
    // Auf PWM-Bereich skalieren
    int pwm = abs((int)(speed * MOTOR_PWM_MAX));
    
    // Deadzone anwenden (unter Schwelle = Stopp)
    if (pwm < PWM_DEADZONE) {
        ledcWrite(ch_a, 0);
        ledcWrite(ch_b, 0);
        return;
    }
    
    // Richtung bestimmen
    if (speed >= 0) {
        // Vorwärts: A aktiv, B aus
        ledcWrite(ch_a, pwm);
        ledcWrite(ch_b, 0);
    } else {
        // Rückwärts: A aus, B aktiv
        ledcWrite(ch_a, 0);
        ledcWrite(ch_b, pwm);
    }
}

/**
 * @brief Stoppt beide Motoren sofort (Coast-Mode)
 */
void hal_motors_stop() {
    ledcWrite(PWM_CH_LEFT_A, 0);
    ledcWrite(PWM_CH_LEFT_B, 0);
    ledcWrite(PWM_CH_RIGHT_A, 0);
    ledcWrite(PWM_CH_RIGHT_B, 0);
}

/**
 * @brief Bremst beide Motoren aktiv (Short Brake)
 * @note Nur für Notfälle - erhöhter Stromverbrauch
 */
void hal_motors_brake() {
    ledcWrite(PWM_CH_LEFT_A, 128);
    ledcWrite(PWM_CH_LEFT_B, 128);
    ledcWrite(PWM_CH_RIGHT_A, 128);
    ledcWrite(PWM_CH_RIGHT_B, 128);
}

// ==========================================================================
// ENCODER ISR (Phase 2 - Stubs)
// ==========================================================================

/**
 * @brief ISR für linken Encoder
 * @note IRAM_ATTR: Funktion im RAM für minimale Latenz
 */
void IRAM_ATTR encoder_left_isr() {
    encoder_left_ticks++;
    // TODO Phase 2: Richtungserkennung mit Quadratur-Auswertung
}

/**
 * @brief ISR für rechten Encoder
 */
void IRAM_ATTR encoder_right_isr() {
    encoder_right_ticks++;
}

/**
 * @brief Initialisiert Encoder-Interrupts
 */
void hal_encoder_init() {
    pinMode(PIN_ENC_LEFT_A, INPUT_PULLUP);
    pinMode(PIN_ENC_RIGHT_A, INPUT_PULLUP);
    
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_LEFT_A), encoder_left_isr, RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_RIGHT_A), encoder_right_isr, RISING);
}

// ==========================================================================
// KINEMATIK & REGELUNG
// ==========================================================================

/**
 * @brief Berechnet Radgeschwindigkeiten aus Twist-Nachricht
 * 
 * @details Differential Drive Kinematik:
 *          v_left  = v_x - (ω × L/2)
 *          v_right = v_x + (ω × L/2)
 * 
 * @param linear_x Lineare Geschwindigkeit [m/s]
 * @param angular_z Winkelgeschwindigkeit [rad/s]
 * @param[out] v_left Geschwindigkeit linkes Rad [m/s]
 * @param[out] v_right Geschwindigkeit rechtes Rad [m/s]
 */
void kinematics_twist_to_wheels(float linear_x, float angular_z, 
                                 float* v_left, float* v_right) {
    *v_left  = linear_x - (angular_z * WHEEL_BASE / 2.0f);
    *v_right = linear_x + (angular_z * WHEEL_BASE / 2.0f);
}

/**
 * @brief Aktualisiert Motorausgänge basierend auf aktuellem Befehl
 * 
 * @note Thread-safe durch Mutex
 */
void update_motors() {
    float linear_x = 0.0f;
    float angular_z = 0.0f;
    uint32_t last_cmd_time = 0;
    
    // Kritischer Bereich: Variablen kopieren
    if (xSemaphoreTake(cmd_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        linear_x = cmd_state.linear_x;
        angular_z = cmd_state.angular_z;
        last_cmd_time = cmd_state.timestamp;
        xSemaphoreGive(cmd_mutex);
    } else {
        // Mutex nicht erhalten → Sicherheitsstopp
        hal_motors_stop();
        return;
    }
    
    // --- FAILSAFE CHECK ---
    uint32_t now = millis();
    if ((now - last_cmd_time) > FAILSAFE_TIMEOUT_MS) {
        hal_motors_stop();
        if (system_state == STATE_CONNECTED) {
            system_state = STATE_FAILSAFE;
        }
        return;
    }
    
    // --- KINEMATIK ---
    float v_left, v_right;
    kinematics_twist_to_wheels(linear_x, angular_z, &v_left, &v_right);
    
    // --- MAPPING: m/s → normalisierte Geschwindigkeit [-1, +1] ---
    // Phase 1: Open Loop (direkte Zuordnung)
    // Phase 2: Ersetzt durch PID mit Encoder-Feedback
    float norm_left  = v_left / MAX_LINEAR_SPEED;
    float norm_right = v_right / MAX_LINEAR_SPEED;
    
    // --- MOTOR OUTPUT ---
    hal_motor_set(PWM_CH_LEFT_A, PWM_CH_LEFT_B, norm_left);
    hal_motor_set(PWM_CH_RIGHT_A, PWM_CH_RIGHT_B, norm_right);
}

// ==========================================================================
// ROS CALLBACKS
// ==========================================================================

/**
 * @brief Callback für /cmd_vel Nachrichten
 * @param msg_in Pointer auf empfangene Twist-Nachricht
 */
void cmd_vel_callback(const void* msg_in) {
    const geometry_msgs__msg__Twist* msg = 
        (const geometry_msgs__msg__Twist*)msg_in;
    
    // Thread-sicher in geteilte Struktur schreiben
    if (xSemaphoreTake(cmd_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        cmd_state.linear_x = msg->linear.x;
        cmd_state.angular_z = msg->angular.z;
        cmd_state.timestamp = millis();
        xSemaphoreGive(cmd_mutex);
    }
    
    // Zustandswechsel
    if (system_state != STATE_CONNECTED) {
        system_state = STATE_CONNECTED;
    }
}

// ==========================================================================
// FREERTOS TASKS
// ==========================================================================

/**
 * @brief LED-Steuerung und Status-Anzeige (Core 0)
 * 
 * @details Blinkmuster:
 *          - WAITING:  Langsames Pulsieren (Suche Agent)
 *          - CONNECTED: Dauerlicht (Betriebsbereit)
 *          - FAILSAFE:  Schnelles Blinken (Keine Befehle)
 *          - ERROR:     SOS-Muster
 */
void task_led_control(void* parameter) {
    (void)parameter;
    
    // Boot-Animation: Sanftes Aufdimmen
    for (int i = 0; i <= 255; i += 5) {
        ledcWrite(LED_PWM_CHANNEL, i);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    vTaskDelay(pdMS_TO_TICKS(200));
    for (int i = 255; i >= 0; i -= 5) {
        ledcWrite(LED_PWM_CHANNEL, i);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    // Hauptschleife: Status-Anzeige
    while (true) {
        switch (system_state) {
            case STATE_WAITING_FOR_AGENT:
                // Breathing-Effekt (sanftes Pulsieren)
                for (int i = 20; i <= 100 && system_state == STATE_WAITING_FOR_AGENT; i += 5) {
                    ledcWrite(LED_PWM_CHANNEL, i);
                    vTaskDelay(pdMS_TO_TICKS(30));
                }
                for (int i = 100; i >= 20 && system_state == STATE_WAITING_FOR_AGENT; i -= 5) {
                    ledcWrite(LED_PWM_CHANNEL, i);
                    vTaskDelay(pdMS_TO_TICKS(30));
                }
                break;
                
            case STATE_CONNECTED:
                // Konstant hell
                ledcWrite(LED_PWM_CHANNEL, 255);
                vTaskDelay(pdMS_TO_TICKS(100));
                break;
                
            case STATE_FAILSAFE:
                // Schnelles Warnen
                ledcWrite(LED_PWM_CHANNEL, 200);
                vTaskDelay(pdMS_TO_TICKS(100));
                ledcWrite(LED_PWM_CHANNEL, 0);
                vTaskDelay(pdMS_TO_TICKS(100));
                break;
                
            case STATE_ERROR:
                // SOS-Muster (3 kurz, 3 lang, 3 kurz)
                for (int i = 0; i < 3; i++) {
                    ledcWrite(LED_PWM_CHANNEL, 255);
                    vTaskDelay(pdMS_TO_TICKS(100));
                    ledcWrite(LED_PWM_CHANNEL, 0);
                    vTaskDelay(pdMS_TO_TICKS(100));
                }
                for (int i = 0; i < 3; i++) {
                    ledcWrite(LED_PWM_CHANNEL, 255);
                    vTaskDelay(pdMS_TO_TICKS(300));
                    ledcWrite(LED_PWM_CHANNEL, 0);
                    vTaskDelay(pdMS_TO_TICKS(100));
                }
                for (int i = 0; i < 3; i++) {
                    ledcWrite(LED_PWM_CHANNEL, 255);
                    vTaskDelay(pdMS_TO_TICKS(100));
                    ledcWrite(LED_PWM_CHANNEL, 0);
                    vTaskDelay(pdMS_TO_TICKS(100));
                }
                vTaskDelay(pdMS_TO_TICKS(500));
                break;
                
            default:
                vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

/**
 * @brief micro-ROS Kommunikation (Core 1)
 * 
 * @details Verbindet mit Agent, empfängt /cmd_vel, aktualisiert Motoren
 */
void task_microros(void* parameter) {
    (void)parameter;
    
    // Task Watchdog aktivieren
    #if ENABLE_TASK_WDT
    esp_task_wdt_add(NULL);
    #endif
    
    // Serial Transport initialisieren
    Serial.begin(115200);
    set_microros_serial_transports(Serial);
    
    allocator = rcl_get_default_allocator();
    
    // Reconnect-Loop
    while (true) {
        system_state = STATE_WAITING_FOR_AGENT;
        
        // Warten auf Agent
        while (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
            #if ENABLE_TASK_WDT
            esp_task_wdt_reset();
            #endif
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        
        // --- micro-ROS Entities erstellen ---
        rcl_ret_t rc;
        
        rc = rclc_support_init(&support, 0, NULL, &allocator);
        if (rc != RCL_RET_OK) {
            system_state = STATE_ERROR;
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        
        rc = rclc_node_init_default(&node, "amr_esp32", "", &support);
        if (rc != RCL_RET_OK) {
            rclc_support_fini(&support);
            system_state = STATE_ERROR;
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        
        rc = rclc_subscription_init_default(
            &cmd_vel_sub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
            "cmd_vel"
        );
        if (rc != RCL_RET_OK) {
            rcl_node_fini(&node);
            rclc_support_fini(&support);
            system_state = STATE_ERROR;
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        
        rc = rclc_executor_init(&executor, &support.context, 1, &allocator);
        if (rc != RCL_RET_OK) {
            rcl_subscription_fini(&cmd_vel_sub, &node);
            rcl_node_fini(&node);
            rclc_support_fini(&support);
            system_state = STATE_ERROR;
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        
        rclc_executor_add_subscription(
            &executor, 
            &cmd_vel_sub, 
            &twist_msg, 
            &cmd_vel_callback, 
            ON_NEW_DATA
        );
        
        system_state = STATE_CONNECTED;
        
        // --- Haupt-Loop (verbunden) ---
        while (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
            #if ENABLE_TASK_WDT
            esp_task_wdt_reset();
            #endif
            
            // ROS-Nachrichten verarbeiten
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
            
            // Motoren aktualisieren
            update_motors();
            
            vTaskDelay(pdMS_TO_TICKS(LOOP_PERIOD_MS));
        }
        
        // --- Cleanup bei Verbindungsverlust ---
        hal_motors_stop();  // Sicherheitsstopp
        system_state = STATE_FAILSAFE;
        
        rcl_subscription_fini(&cmd_vel_sub, &node);
        rclc_executor_fini(&executor);
        rcl_node_fini(&node);
        rclc_support_fini(&support);
        
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// ==========================================================================
// SETUP & LOOP
// ==========================================================================

void setup() {
    // --- Mutex erstellen (vor Tasks!) ---
    cmd_mutex = xSemaphoreCreateMutex();
    if (cmd_mutex == NULL) {
        // Kritischer Fehler - Endlosschleife
        while (true) { delay(1000); }
    }
    
    // --- Hardware initialisieren ---
    hal_pwm_init();
    hal_encoder_init();
    
    // --- Task Watchdog konfigurieren ---
    #if ENABLE_TASK_WDT
    esp_task_wdt_init(TASK_WDT_TIMEOUT_S, true);  // true = Panic bei Timeout
    #endif
    
    // --- FreeRTOS Tasks starten ---
    
    // LED Task: Core 0, niedrige Priorität
    xTaskCreatePinnedToCore(
        task_led_control,
        "LED_Task",
        2048,           // Stack Size
        NULL,           // Parameter
        1,              // Priorität (niedrig)
        NULL,           // Task Handle
        0               // Core 0
    );
    
    // micro-ROS Task: Core 1, hohe Priorität
    xTaskCreatePinnedToCore(
        task_microros,
        "ROS_Task",
        16384,          // Stack Size (micro-ROS braucht viel)
        NULL,
        2,              // Priorität (höher als LED)
        NULL,
        1               // Core 1
    );
}

void loop() {
    // FreeRTOS übernimmt - Arduino loop() nicht benötigt
    vTaskDelete(NULL);
}
