/**
 * @file    ev_control.h
 * @brief   Electric Vehicle dynamics model — torque, speed, SOC, range
 */

#ifndef EV_CONTROL_H
#define EV_CONTROL_H


/* USER CODE BEGIN PFP */



/* USER CODE END PFP */

#include "common.h"

/* ─── EV Handle ──────────────────────────────────────────────────────────── */
typedef struct {
    float   accel_pedal;    /* 0–100 %                */
    float   brake_pedal;    /* 0–100 %                */
    float   speed_kmh;      /* 0–200 km/h             */
    float   motor_torque;   /* Nm  (neg = regen)      */
    float   motor_temp;     /* °C  (ADC simulated)    */
    float   soc;            /* 0–100 %                */
    float   regen_level;    /* 0–100 %                */
    float   power_kw;       /* + drive, − regen       */
    float   range_km;       /* estimated range        */
    uint8_t drive_mode;     /* ECO / NORMAL / SPORT   */
} EV_HandleTypeDef;

/* ─── Constants ──────────────────────────────────────────────────────────── */
#define EV_MAX_TORQUE_NM        150.0f
#define EV_REGEN_TORQUE_MAX_NM   80.0f
#define EV_REGEN_THRESHOLD_PCT    5.0f
#define EV_BATTERY_CAPACITY_KWH  60.0f
#define EV_MASS_FACTOR         1500.0f
#define EV_MAX_SPEED_KMH        200.0f
#define EV_MAX_MOTOR_TEMP_C      90.0f
#define EV_FAULT_SOC_PCT          2.0f
#define EV_DRAG_COEFF             2.0f

#define EV_SIM_SCALE        50.0f

/* Efficiency Wh/km per mode */
#define EV_EFFICIENCY_ECO    14.0f
#define EV_EFFICIENCY_OTHER  18.0f

/* ─── API ────────────────────────────────────────────────────────────────── */
void EV_Init(EV_HandleTypeDef *ev);
void EV_Update(EV_HandleTypeDef *ev, float dt);
void EV_ReadADC(EV_HandleTypeDef *ev);
void EV_SetDriveMode(EV_HandleTypeDef *ev, uint8_t mode);
void EV_InjectSpeed(EV_HandleTypeDef *ev, float speed_kmh);
void EV_InjectSOC(EV_HandleTypeDef *ev, float soc_pct);
void EV_InjectMotorTemp(EV_HandleTypeDef *ev, float temp_c);
uint16_t Read_ADC_Channel(uint32_t channel);

#endif /* EV_CONTROL_H */
