#include "ultrasonic.h"

typedef struct {
    GPIO_TypeDef *trig_port;
    uint16_t      trig_pin;
    GPIO_TypeDef *echo_port;
    uint16_t      echo_pin;
} HCSR04_Sensor_t;

static const HCSR04_Sensor_t SENSORS[HCSR04_COUNT] = {
    [HCSR04_FRONT] = { FRONT_TRIG_PORT, FRONT_TRIG_PIN,
                        FRONT_ECHO_PORT, FRONT_ECHO_PIN },
    [HCSR04_LEFT]  = { LEFT_TRIG_PORT,  LEFT_TRIG_PIN,
                        LEFT_ECHO_PORT,  LEFT_ECHO_PIN  },
    [HCSR04_RIGHT] = { RIGHT_TRIG_PORT, RIGHT_TRIG_PIN,
                        RIGHT_ECHO_PORT, RIGHT_ECHO_PIN },
};

static HCSR04_Result_t results[HCSR04_COUNT];

/* ── FIX 1: Cast to uint16_t for safe 16-bit rollover ─────────────────── */
static inline uint16_t TIM2_GetUS(void)
{
    return (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
}

static void Delay_US(uint16_t us)
{
    uint16_t start = TIM2_GetUS();
    while ((uint16_t)(TIM2_GetUS() - start) < us);
}

void HCSR04_Init(void)
{
    for (int i = 0; i < HCSR04_COUNT; i++) {
        results[i].distance_cm = HCSR04_MAX_CM;
        results[i].valid       = 0;
        HAL_GPIO_WritePin(SENSORS[i].trig_port,
                          SENSORS[i].trig_pin,
                          GPIO_PIN_RESET);
    }
}

HCSR04_Result_t HCSR04_Read(HCSR04_ID id)  // 0 1 2
{
    HCSR04_Result_t res  = { HCSR04_MAX_CM, 0 };
    const HCSR04_Sensor_t *s = &SENSORS[id];
    uint16_t echo_fall = 0;

    /* — Step 1: Send 10µs TRIG pulse — */
        HAL_GPIO_WritePin(s->trig_port, s->trig_pin, GPIO_PIN_RESET);
        Delay_US(2);

        // ===>>> DISABLE INTERRUPTS HERE <<<===
        __disable_irq();

        HAL_GPIO_WritePin(s->trig_port, s->trig_pin, GPIO_PIN_SET);
        Delay_US(10);
        HAL_GPIO_WritePin(s->trig_port, s->trig_pin, GPIO_PIN_RESET);

        /* — Step 2: Wait for ECHO HIGH — */
        uint16_t t_start = TIM2_GetUS();
        while (HAL_GPIO_ReadPin(s->echo_port, s->echo_pin) == GPIO_PIN_RESET) {
            if ((uint16_t)(TIM2_GetUS() - t_start) > 30000U) {
                __enable_irq(); // Clean up before leaving!
                results[id] = res;
                return res;
            }
        }
        uint16_t echo_rise = TIM2_GetUS();

        /* — Step 3: Wait for ECHO LOW — */
        while (HAL_GPIO_ReadPin(s->echo_port, s->echo_pin) == GPIO_PIN_SET) {
            echo_fall = TIM2_GetUS();
            if ((uint16_t)(echo_fall - echo_rise) > 30000U) {
                __enable_irq(); // Clean up before leaving!
                results[id] = res;
                return res;
            }
        }

        // ===>>> RE-ENABLE INTERRUPTS HERE <<<===
        __enable_irq();


    /* ── Step 4: Calculate distance ────────────────────────────────── */
    uint16_t pulse_us = echo_fall - echo_rise;
    float dist_cm   = (pulse_us * SOUND_SPEED_CM_PER_US) / 2.0f;

    res.distance_cm = CLAMP(dist_cm, HCSR04_MIN_CM, HCSR04_MAX_CM);
    res.valid       = 1;
    results[id]     = res;
    return res;
}

void HCSR04_ReadAll(void)
{
    for (int i = 0; i < HCSR04_COUNT; i++) {
        HCSR04_Read((HCSR04_ID)i);
        HAL_Delay(50);
    }
}

float   HCSR04_GetDistance(HCSR04_ID id) { return results[id].distance_cm; }
uint8_t HCSR04_IsValid    (HCSR04_ID id) { return results[id].valid;        }
