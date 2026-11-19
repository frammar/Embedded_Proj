#pragma once

// UART link to ESP32-CAM (UART0)
#define LINK_TX_PIN       0
#define LINK_RX_PIN       1
// HC-SR04
#define US_TRIG_PIN       2
#define US_ECHO_PIN       3
// L298N
#define MOTOR_ENA_PIN     6
#define MOTOR_IN1_PIN     7
#define MOTOR_IN2_PIN     8
// SG90 servos
#define SERVO_DROP_L_PIN  14
#define SERVO_DROP_R_PIN  15
#define SERVO_RESET_PIN   18
// Buzzer (through 2N2222)
#define BUZZER_PIN        16
// LEDs
#define LED_ONBOARD_PIN   25
#define LED_DIST_PIN      26
#define LED_STATUS_PIN    27