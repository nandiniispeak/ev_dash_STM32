/**
 * @file    ultrasonic.h
 * @brief   HC-SR04 ultrasonic sensor driver — TIM2 µs polling
 */

#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include "common.h"

/* ─── Sensor IDs ─────────────────────────────────────────────────────────── */
typedef enum {
    HCSR04_FRONT = 0,
    HCSR04_LEFT  = 1,
    HCSR04_RIGHT = 2,
    HCSR04_COUNT = 3,
} HCSR04_ID;

/* ─── Measurement limits ─────────────────────────────────────────────────── */
#define HCSR04_MIN_CM          2.0f
#define HCSR04_MAX_CM        400.0f
#define HCSR04_ECHO_TIMEOUT_US 30000UL  /* 30 ms → out of range             */
#define HCSR04_TRIG_PULSE_US      10UL  /* 10 µs TRIG pulse                 */
#define HCSR04_INTER_SENSOR_US 50000UL  /* 50 ms gap — avoid cross-talk     */
#define SOUND_SPEED_CM_PER_US  0.0343f

/* ─── Result struct ──────────────────────────────────────────────────────── */
typedef struct {
    float   distance_cm;
    uint8_t valid;          /* 0 = timeout / out of range */
} HCSR04_Result_t;

/* ─── API ────────────────────────────────────────────────────────────────── */
void          HCSR04_Init(void);
HCSR04_Result_t HCSR04_Read(HCSR04_ID id);
void          HCSR04_ReadAll(void);
float         HCSR04_GetDistance(HCSR04_ID id);
uint8_t       HCSR04_IsValid(HCSR04_ID id);

#endif /* ULTRASONIC_H */
