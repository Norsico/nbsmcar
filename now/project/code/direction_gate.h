#ifndef __DIRECTION_GATE_H__
#define __DIRECTION_GATE_H__

#include "headfile.h"

/* UART2: module TX -> P46 (MCU RX), module RX -> P47 (MCU TX). */
#define DIRECTION_GATE_UART                    (UART_2)
#define DIRECTION_GATE_UART_BAUD               (115200)
#define DIRECTION_GATE_UART_RX_PIN             (UART2_RX_P46)
#define DIRECTION_GATE_UART_TX_PIN             (UART2_TX_P47)

/* 0: full drive, 1: UART monitor, 2: steering test with motors stopped. */
#define DIRECTION_GATE_TEST_MODE               (0)
#define DIRECTION_GATE_TEST_DRIVE              (0)
#define DIRECTION_GATE_TEST_UART               (1)
#define DIRECTION_GATE_TEST_STEER              (2)

/* Change to -1 when the measured angle turns the car the wrong way. */
#define DIRECTION_GATE_ANGLE_SIGN              (1)

#if (DIRECTION_GATE_ANGLE_SIGN != 1) && (DIRECTION_GATE_ANGLE_SIGN != -1)
#error DIRECTION_GATE_ANGLE_SIGN must be 1 or -1
#endif

/* Steering control. Angle values use 0.01 degree units. */
#define DIRECTION_GATE_STEER_KP                (18)
#define DIRECTION_GATE_STEER_KD                (6)
#define DIRECTION_GATE_STEER_DEADBAND          (150)
#define DIRECTION_GATE_ANGLE_DELTA_LIMIT       (2500)

/* Motor target values use the existing 5 ms encoder control units. */
#define DIRECTION_GATE_CRUISE_SPEED            (50)
#define DIRECTION_GATE_APPROACH_SPEED          (50)
#define DIRECTION_GATE_TURN_SPEED              (50)
#define DIRECTION_GATE_TIGHT_TURN_SPEED        (50)
#define DIRECTION_GATE_CLEAR_SPEED             (50)
#define DIRECTION_GATE_TURN_ANGLE              (2500)
#define DIRECTION_GATE_TIGHT_TURN_ANGLE        (5500)
#define DIRECTION_GATE_INNER_REDUCTION         (35)

/* Signal and gate-change detection. Tune these after reading live data. */
#define DIRECTION_GATE_SIGNAL_MIN              (40)
#define DIRECTION_GATE_NEAR_SIGNAL             (700)
#define DIRECTION_GATE_PASS_SIGNAL             (650)
#define DIRECTION_GATE_SWITCH_DROP_PERCENT     (55)
#define DIRECTION_GATE_SWITCH_ANGLE            (4000)
#define DIRECTION_GATE_SIGNAL_TIMEOUT_MS       (200)
#define DIRECTION_GATE_CLEAR_TIME_MS           (300)
#define DIRECTION_GATE_SCREEN_UPDATE_MS        (100)

typedef enum
{
    DIRECTION_GATE_WAIT_SIGNAL = 0,
    DIRECTION_GATE_TRACK,
    DIRECTION_GATE_CLEAR_GATE
} direction_gate_phase;

typedef struct
{
    volatile int16 angle_cdeg;
    volatile uint16 channel1;
    volatile uint16 channel2;
    volatile uint16 strength;
    volatile uint16 sequence;
    volatile uint16 valid_frames;
    volatile uint16 invalid_frames;
    volatile uint16 rx_bytes;
    volatile uint16 rx_overflows;
    volatile uint8 signal_valid;
    direction_gate_phase phase;
    uint8 pass_count;
} direction_gate_data;

extern direction_gate_data DirectionGate;

void direction_gate_init(void);
void direction_gate_update(void);
void direction_gate_tick_5ms(void);
void direction_gate_uart_callback(uint8 dat);

#endif
