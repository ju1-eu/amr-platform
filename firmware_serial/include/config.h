/**
 * @file config.h
 * @brief Zentrale Konfiguration für AMR Slave-Node (ESP32-S3)
 * @version 1.2.0
 * @date 2025-12-12
 *
 * @standard REP-103 (SI-Einheiten), REP-105 (Frames), Safety-First
 * @hardware Seeed Studio XIAO ESP32-S3, Cytron MDD3A, JGA25-370
 *
 * CHANGELOG v1.2.0:
 *   - Encoder-Werte kalibriert (10-Umdrehungen-Test)
 *   - Separate TICKS_PER_REV für links/rechts
 *   - Odometrie-Parameter hinzugefügt
 */

#ifndef CONFIG_H
#define CONFIG_H

// ==========================================================================
// 1. HARDWARE ABSTRACTION LAYER (HAL)
// ==========================================================================
// Target: Seeed Studio XIAO ESP32-S3

// --- Antriebsstrang (Cytron MDD3A - DUAL PWM MODE) ---
// MDD3A Steuerlogik:
//   PWM_A > 0, PWM_B = 0  → Vorwärts
//   PWM_A = 0, PWM_B > 0  → Rückwärts
//   PWM_A = 0, PWM_B = 0  → Coast (Auslaufen)
//   PWM_A > 0, PWM_B > 0  → Bremsen (Active Brake)

#define PIN_MOTOR_LEFT_A D0  // MDD3A M1A (Vorwärts-PWM)
#define PIN_MOTOR_LEFT_B D1  // MDD3A M1B (Rückwärts-PWM)
#define PIN_MOTOR_RIGHT_A D2 // MDD3A M2A (Vorwärts-PWM)
#define PIN_MOTOR_RIGHT_B D3 // MDD3A M2B (Rückwärts-PWM)

// PWM-Kanäle (ESP32 LEDC)
#define PWM_CH_LEFT_A 0
#define PWM_CH_LEFT_B 1
#define PWM_CH_RIGHT_A 2
#define PWM_CH_RIGHT_B 3

// --- Odometrie (Hall-Encoder JGA25-370) ---
#define PIN_ENC_LEFT_A D6  // Interrupt-fähig
#define PIN_ENC_RIGHT_A D7 // Interrupt-fähig

// --- Peripherie & Status ---
#define PIN_LED_MOSFET D10 // IRLZ24N Low-Side Switch

// --- I2C Bus (MPU6050 / Future Use) ---
#define PIN_I2C_SDA D4
#define PIN_I2C_SCL D5
#define IMU_I2C_ADDR 0x68

// --- Servos (Pan/Tilt - Optional) ---
#define PIN_SERVO_PAN D8
#define PIN_SERVO_TILT D9

// ==========================================================================
// 2. KINEMATISCHE PARAMETER (SI-Einheiten / REP-103)
// ==========================================================================

// Raddurchmesser [m] - Gemessen: 65 mm
#define WHEEL_DIAMETER 0.065f

// Radradius [m] - Abgeleitet
#define WHEEL_RADIUS (WHEEL_DIAMETER / 2.0f)

// Spurbreite [m] - Reifenmitte zu Reifenmitte
// Berechnung: Außenmaß (203 mm) - Reifenbreite (25 mm) = 178 mm
#define WHEEL_BASE 0.178f

// Radumfang [m] - Für Odometrie
#define WHEEL_CIRCUMFERENCE (WHEEL_DIAMETER * 3.14159265359f)

// ==========================================================================
// 2.1 ENCODER-KALIBRIERUNG (Phase 2 - 2025-12-12)
// ==========================================================================
// Methode: 10-Umdrehungen-Test mit calibration_encoder.cpp
// Ergebnis: Links 3743 Ticks, Rechts 3736 Ticks

#define TICKS_PER_REV_LEFT 374.3f  // Kalibriert (theoretisch: 390.5)
#define TICKS_PER_REV_RIGHT 373.6f // Kalibriert (theoretisch: 390.5)

// Durchschnitt für symmetrische Berechnungen (falls benötigt)
#define TICKS_PER_REV ((TICKS_PER_REV_LEFT + TICKS_PER_REV_RIGHT) / 2.0f)

// Umrechnung: Meter pro Tick (für jedes Rad separat)
#define METERS_PER_TICK_LEFT (WHEEL_CIRCUMFERENCE / TICKS_PER_REV_LEFT)
#define METERS_PER_TICK_RIGHT (WHEEL_CIRCUMFERENCE / TICKS_PER_REV_RIGHT)

// ==========================================================================
// 3. REGELUNGSTECHNIK
// ==========================================================================

// --- PWM Konfiguration ---
#define MOTOR_PWM_FREQ 20000 // 20 kHz (unhörbar)
#define MOTOR_PWM_BITS 8     // 8-bit Auflösung (0-255)
#define MOTOR_PWM_MAX 255

// --- LED PWM (Status-Feedback) ---
#define LED_PWM_FREQ 5000
#define LED_PWM_BITS 8
#define LED_PWM_CHANNEL 4 // Kanal 4 (0-3 für Motoren reserviert)

// --- PID-Regler (Phase 2 - Velocity Control) ---
#define PID_KP 1.5f
#define PID_KI 0.05f
#define PID_KD 0.0f

// --- Geschwindigkeitslimits ---
#define MAX_LINEAR_SPEED 0.5f  // [m/s] Maximale Vorwärtsgeschwindigkeit
#define MAX_ANGULAR_SPEED 2.0f // [rad/s] Maximale Drehgeschwindigkeit

// --- Motor Deadzone ---
// PWM-Wert unter dem der Motor nicht anläuft (Haftreibung)
#define PWM_DEADZONE 35

// ==========================================================================
// 4. SAFETY STANDARDS
// ==========================================================================

// --- Failsafe / Watchdog ---
// Stoppt Motoren wenn > TIMEOUT_MS kein /cmd_vel empfangen
#define FAILSAFE_TIMEOUT_MS 5000 // 5 Sekunden statt 500ms

// --- Hardware Watchdog ---
#define ENABLE_TASK_WDT true
#define TASK_WDT_TIMEOUT_S 5

// --- Sanity Checks ---
// Maximaler plausibler Encoder-Sprung pro Zyklus
// Bei 100 Hz und 0.5 m/s: ~10 Ticks/Zyklus, Faktor 5 als Reserve
#define MAX_TICK_DELTA 50

// ==========================================================================
// 5. TIMING & LOOP RATES
// ==========================================================================

#define LOOP_RATE_HZ 100 // Haupt-Regelschleife
#define LOOP_PERIOD_MS (1000 / LOOP_RATE_HZ)

#define ODOM_PUBLISH_HZ 50 // Odometrie-Veröffentlichung
#define ODOM_PERIOD_MS (1000 / ODOM_PUBLISH_HZ)

// ==========================================================================
// 6. DEBUG & DIAGNOSTICS
// ==========================================================================

#define DEBUG_SERIAL false
#define DEBUG_BAUD 115200

#define ENABLE_DIAGNOSTICS false // Phase 3

#endif // CONFIG_H
