#include "headfile.h"

#define DIRECTION_GATE_RX_BUFFER_SIZE          (64)
#define DIRECTION_GATE_RX_BUFFER_MASK          (DIRECTION_GATE_RX_BUFFER_SIZE - 1)
#define DIRECTION_GATE_LINE_SIZE               (40)
#define DIRECTION_GATE_TIMEOUT_TICKS           ((DIRECTION_GATE_SIGNAL_TIMEOUT_MS + MOTOR_CTRL_PERIOD_MS - 1) / MOTOR_CTRL_PERIOD_MS)
#define DIRECTION_GATE_CLEAR_TICKS             ((DIRECTION_GATE_CLEAR_TIME_MS + MOTOR_CTRL_PERIOD_MS - 1) / MOTOR_CTRL_PERIOD_MS)
#define DIRECTION_GATE_SCREEN_TICKS            ((DIRECTION_GATE_SCREEN_UPDATE_MS + MOTOR_CTRL_PERIOD_MS - 1) / MOTOR_CTRL_PERIOD_MS)
#define DIRECTION_GATE_SCREEN_VALUE_X          (80)

direction_gate_data DirectionGate;

static volatile uint8 xdata GateRxBuffer[DIRECTION_GATE_RX_BUFFER_SIZE];
static volatile uint8 GateRxHead = 0;
static volatile uint8 GateRxTail = 0;
static volatile uint16 GateSignalAgeTicks = 0xffff;
static volatile uint16 GateClearTicks = 0;
static volatile uint16 GateScreenTicks = 0;
static volatile uint8 GateFramePending = 0;

static char GateLine[DIRECTION_GATE_LINE_SIZE];
static uint8 GateLineLength = 0;
static uint8 GateLineOverflow = 0;
static int16 GateLastAngle = 0;
static uint16 GatePeakStrength = 0;
static uint8 GateHasLastAngle = 0;

static int16 direction_gate_abs(int16 value)
{
    return (value < 0) ? -value : value;
}

static int16 direction_gate_limit(int16 value, int16 minimum, int16 maximum)
{
    if(value < minimum)
    {
        return minimum;
    }
    if(value > maximum)
    {
        return maximum;
    }
    return value;
}

static uint8 direction_gate_parse_angle(char *text, uint8 length, int16 *result)
{
    uint8 index;
    uint8 negative;
    uint8 point_seen;
    uint8 digit_seen;
    uint8 fraction_digits;
    uint16 integer_part;
    uint16 fraction_part;
    uint16 value;
    char character;

    if((length == 0) || (result == NULL))
    {
        return 0;
    }

    index = 0;
    negative = 0;
    point_seen = 0;
    digit_seen = 0;
    fraction_digits = 0;
    integer_part = 0;
    fraction_part = 0;

    if((text[index] == '-') || (text[index] == '+'))
    {
        negative = (text[index] == '-') ? 1 : 0;
        index++;
    }

    for(; index < length; index++)
    {
        character = text[index];
        if((character >= '0') && (character <= '9'))
        {
            digit_seen = 1;
            if(point_seen == 0)
            {
                integer_part = (uint16)(integer_part * 10 + character - '0');
                if(integer_part > 90)
                {
                    return 0;
                }
            }
            else if(fraction_digits < 2)
            {
                fraction_part = (uint16)(fraction_part * 10 + character - '0');
                fraction_digits++;
            }
        }
        else if((character == '.') && (point_seen == 0))
        {
            point_seen = 1;
        }
        else
        {
            return 0;
        }
    }

    if(digit_seen == 0)
    {
        return 0;
    }
    if(fraction_digits == 1)
    {
        fraction_part *= 10;
    }

    value = (uint16)(integer_part * 100 + fraction_part);
    if(value > 9000)
    {
        return 0;
    }

    *result = negative ? -(int16)value : (int16)value;
    return 1;
}

static uint8 direction_gate_parse_channel(char *text, uint8 length, uint16 *result)
{
    uint8 index;
    uint32 value;
    char character;

    if((length == 0) || (result == NULL))
    {
        return 0;
    }

    value = 0;
    for(index = 0; index < length; index++)
    {
        character = text[index];
        if((character < '0') || (character > '9'))
        {
            return 0;
        }
        value = value * 10 + character - '0';
        if(value > 65535UL)
        {
            return 0;
        }
    }

    *result = (uint16)value;
    return 1;
}

static uint8 direction_gate_parse_frame(char *line, uint8 length)
{
    uint8 index;
    uint8 comma1;
    uint8 comma2;
    int16 angle;
    uint16 channel1;
    uint16 channel2;
    uint32 strength;

    comma1 = 0xff;
    comma2 = 0xff;
    for(index = 0; index < length; index++)
    {
        if(line[index] == ',')
        {
            if(comma1 == 0xff)
            {
                comma1 = index;
            }
            else if(comma2 == 0xff)
            {
                comma2 = index;
            }
            else
            {
                return 0;
            }
        }
    }

    if((comma1 == 0xff) || (comma2 == 0xff) ||
       (comma1 == 0) || (comma2 <= comma1 + 1) || (comma2 >= length - 1))
    {
        return 0;
    }
    if(direction_gate_parse_angle(line, comma1, &angle) == 0)
    {
        return 0;
    }
    if(direction_gate_parse_channel(&line[comma1 + 1],
                                    (uint8)(comma2 - comma1 - 1), &channel1) == 0)
    {
        return 0;
    }
    if(direction_gate_parse_channel(&line[comma2 + 1],
                                    (uint8)(length - comma2 - 1), &channel2) == 0)
    {
        return 0;
    }

    strength = (uint32)channel1 + channel2;
    DirectionGate.angle_cdeg = angle;
    DirectionGate.channel1 = channel1;
    DirectionGate.channel2 = channel2;
    DirectionGate.strength = (strength > 65535UL) ? 65535 : (uint16)strength;
    DirectionGate.signal_valid =
        (DirectionGate.strength >= DIRECTION_GATE_SIGNAL_MIN) ? 1 : 0;
    DirectionGate.sequence++;
    DirectionGate.valid_frames++;
    GateFramePending = 1;

    if(DirectionGate.signal_valid != 0)
    {
        GateSignalAgeTicks = 0;
    }
    return 1;
}

static uint8 direction_gate_rx_pop(uint8 *dat)
{
    if(GateRxTail == GateRxHead)
    {
        return 0;
    }

    *dat = GateRxBuffer[GateRxTail];
    GateRxTail = (uint8)((GateRxTail + 1) & DIRECTION_GATE_RX_BUFFER_MASK);
    return 1;
}

static void direction_gate_receive_update(void)
{
    uint8 dat;

    while(direction_gate_rx_pop(&dat) != 0)
    {
        if(dat == '\r')
        {
            continue;
        }
        if(dat == '\n')
        {
            if((GateLineOverflow == 0) && (GateLineLength != 0))
            {
                if(direction_gate_parse_frame(GateLine, GateLineLength) == 0)
                {
                    DirectionGate.invalid_frames++;
                }
            }
            else if((GateLineOverflow != 0) || (GateLineLength != 0))
            {
                DirectionGate.invalid_frames++;
            }
            GateLineLength = 0;
            GateLineOverflow = 0;
            continue;
        }

        if(GateLineLength < DIRECTION_GATE_LINE_SIZE - 1)
        {
            GateLine[GateLineLength++] = (char)dat;
        }
        else
        {
            GateLineOverflow = 1;
        }
    }
}

static void direction_gate_write_servo(int16 control)
{
    int16 servo_angle;

    servo_angle = (int16)(SERVO_ANGLE_CENTER - control);
    servo_angle = direction_gate_limit(servo_angle,
                                       SERVO_ANGLE_MIN, SERVO_ANGLE_MAX);
    pwm_set_duty(SERVO_PWM, (uint32)(servo_angle / 3 + 1500));
}

static void direction_gate_write_motor(int16 speed, int16 control)
{
    int16 reduction;

    if(speed <= 0)
    {
        Motor.target_left = 0;
        Motor.target_right = 0;
        return;
    }

    reduction = (int16)(((int32)speed * direction_gate_abs(control) *
                         DIRECTION_GATE_INNER_REDUCTION) /
                        ((int32)(SERVO_ANGLE_CENTER - SERVO_ANGLE_MIN) * 100));

    if(control > 0)
    {
        Motor.target_left = speed;
        Motor.target_right = speed - reduction;
    }
    else if(control < 0)
    {
        Motor.target_left = speed - reduction;
        Motor.target_right = speed;
    }
    else
    {
        Motor.target_left = speed;
        Motor.target_right = speed;
    }
}

static int16 direction_gate_steering_control(int16 angle)
{
    int16 delta;
    int16 control;

#if DIRECTION_GATE_ANGLE_SIGN < 0
    angle = -angle;
#endif
    if(direction_gate_abs(angle) < DIRECTION_GATE_STEER_DEADBAND)
    {
        angle = 0;
    }

    if(GateHasLastAngle == 0)
    {
        delta = 0;
        GateHasLastAngle = 1;
    }
    else
    {
        delta = angle - GateLastAngle;
        delta = direction_gate_limit(delta,
                                      -DIRECTION_GATE_ANGLE_DELTA_LIMIT,
                                      DIRECTION_GATE_ANGLE_DELTA_LIMIT);
    }
    GateLastAngle = angle;

    control = (int16)(((int32)DIRECTION_GATE_STEER_KP * angle +
                       (int32)DIRECTION_GATE_STEER_KD * delta) / 100);
    return direction_gate_limit(control,
                                SERVO_ANGLE_CENTER - SERVO_ANGLE_MAX,
                                SERVO_ANGLE_CENTER - SERVO_ANGLE_MIN);
}

static int16 direction_gate_select_speed(int16 angle, uint16 strength)
{
    int16 absolute_angle;

    absolute_angle = direction_gate_abs(angle);
    if(absolute_angle >= DIRECTION_GATE_TIGHT_TURN_ANGLE)
    {
        return DIRECTION_GATE_TIGHT_TURN_SPEED;
    }
    if(absolute_angle >= DIRECTION_GATE_TURN_ANGLE)
    {
        return DIRECTION_GATE_TURN_SPEED;
    }
    if(strength >= DIRECTION_GATE_NEAR_SIGNAL)
    {
        return DIRECTION_GATE_APPROACH_SPEED;
    }
    return DIRECTION_GATE_CRUISE_SPEED;
}

static void direction_gate_stop(void)
{
    direction_gate_write_servo(0);
    direction_gate_write_motor(0, 0);
}

static void direction_gate_begin_clear(void)
{
    DirectionGate.phase = DIRECTION_GATE_CLEAR_GATE;
    DirectionGate.pass_count++;
    GateClearTicks = DIRECTION_GATE_CLEAR_TICKS;
    GatePeakStrength = 0;
    GateHasLastAngle = 0;
    direction_gate_write_servo(0);
    direction_gate_write_motor(DIRECTION_GATE_CLEAR_SPEED, 0);
}

static uint8 direction_gate_switch_detected(int16 angle, uint16 strength)
{
    int16 angle_change;
    uint32 reduced_strength;
    uint32 peak_strength;

    if((GatePeakStrength < DIRECTION_GATE_PASS_SIGNAL) ||
       (GateHasLastAngle == 0))
    {
        return 0;
    }

#if DIRECTION_GATE_ANGLE_SIGN < 0
    angle = -angle;
#endif
    angle_change = direction_gate_abs(angle - GateLastAngle);
    reduced_strength = (uint32)strength * 100;
    peak_strength = (uint32)GatePeakStrength * DIRECTION_GATE_SWITCH_DROP_PERCENT;

    if((reduced_strength < peak_strength) ||
       (angle_change >= DIRECTION_GATE_SWITCH_ANGLE))
    {
        return 1;
    }
    return 0;
}

static void direction_gate_track_frame(void)
{
    int16 angle;
    int16 control;
    int16 speed;
    uint16 strength;

    angle = DirectionGate.angle_cdeg;
    strength = DirectionGate.strength;

    if(direction_gate_switch_detected(angle, strength) != 0)
    {
        direction_gate_begin_clear();
        return;
    }

    if(strength > GatePeakStrength)
    {
        GatePeakStrength = strength;
    }

    DirectionGate.phase = DIRECTION_GATE_TRACK;
    control = direction_gate_steering_control(angle);
    speed = direction_gate_select_speed(angle, strength);
    direction_gate_write_servo(control);
    direction_gate_write_motor(speed, control);
}

static void direction_gate_screen_clear_value(uint16 y)
{
    ips200_show_string(DIRECTION_GATE_SCREEN_VALUE_X, y, "      ");
}

static void direction_gate_screen_init(void)
{
    ips200_set_dir(IPS200_PORTAIT);
    ips200_init();
    ips200_set_color(RGB565_BLACK, RGB565_WHITE);
    ips200_clear(RGB565_WHITE);

    ips200_show_string(0, 0, "DIRECTION GATE");
    ips200_show_string(0, 16, "Bytes");
    ips200_show_string(0, 32, "Frames");
    ips200_show_string(0, 48, "Errors");
    ips200_show_string(0, 64, "Overflow");
    ips200_show_string(0, 80, "Angle x100");
    ips200_show_string(0, 96, "Channel 1");
    ips200_show_string(0, 112, "Channel 2");
    ips200_show_string(0, 128, "Strength");
    ips200_show_string(0, 144, "Valid");
    ips200_show_string(0, 160, "Phase");
    ips200_show_string(0, 176, "Motor L");
    ips200_show_string(0, 192, "Motor R");
    ips200_show_string(0, 208, "Passes");
}

static void direction_gate_screen_update(void)
{
    if(GateScreenTicks != 0)
    {
        return;
    }
    GateScreenTicks = DIRECTION_GATE_SCREEN_TICKS;

    direction_gate_screen_clear_value(16);
    ips200_show_uint16(DIRECTION_GATE_SCREEN_VALUE_X, 16, DirectionGate.rx_bytes);
    direction_gate_screen_clear_value(32);
    ips200_show_uint16(DIRECTION_GATE_SCREEN_VALUE_X, 32, DirectionGate.valid_frames);
    direction_gate_screen_clear_value(48);
    ips200_show_uint16(DIRECTION_GATE_SCREEN_VALUE_X, 48, DirectionGate.invalid_frames);
    direction_gate_screen_clear_value(64);
    ips200_show_uint16(DIRECTION_GATE_SCREEN_VALUE_X, 64, DirectionGate.rx_overflows);
    direction_gate_screen_clear_value(80);
    ips200_show_int16(DIRECTION_GATE_SCREEN_VALUE_X, 80, DirectionGate.angle_cdeg);
    direction_gate_screen_clear_value(96);
    ips200_show_uint16(DIRECTION_GATE_SCREEN_VALUE_X, 96, DirectionGate.channel1);
    direction_gate_screen_clear_value(112);
    ips200_show_uint16(DIRECTION_GATE_SCREEN_VALUE_X, 112, DirectionGate.channel2);
    direction_gate_screen_clear_value(128);
    ips200_show_uint16(DIRECTION_GATE_SCREEN_VALUE_X, 128, DirectionGate.strength);
    direction_gate_screen_clear_value(144);
    ips200_show_string(DIRECTION_GATE_SCREEN_VALUE_X, 144,
                       DirectionGate.signal_valid ? "YES" : "NO");
    direction_gate_screen_clear_value(160);
    if(DirectionGate.phase == DIRECTION_GATE_TRACK)
    {
        ips200_show_string(DIRECTION_GATE_SCREEN_VALUE_X, 160, "TRACK");
    }
    else if(DirectionGate.phase == DIRECTION_GATE_CLEAR_GATE)
    {
        ips200_show_string(DIRECTION_GATE_SCREEN_VALUE_X, 160, "CLEAR");
    }
    else
    {
        ips200_show_string(DIRECTION_GATE_SCREEN_VALUE_X, 160, "WAIT");
    }
    direction_gate_screen_clear_value(176);
    ips200_show_int16(DIRECTION_GATE_SCREEN_VALUE_X, 176, Motor.target_left);
    direction_gate_screen_clear_value(192);
    ips200_show_int16(DIRECTION_GATE_SCREEN_VALUE_X, 192, Motor.target_right);
    direction_gate_screen_clear_value(208);
    ips200_show_uint8(DIRECTION_GATE_SCREEN_VALUE_X, 208, DirectionGate.pass_count);
}

void direction_gate_uart_callback(uint8 dat)
{
    uint8 next;

    DirectionGate.rx_bytes++;
    next = (uint8)((GateRxHead + 1) & DIRECTION_GATE_RX_BUFFER_MASK);
    if(next == GateRxTail)
    {
        DirectionGate.rx_overflows++;
        return;
    }

    GateRxBuffer[GateRxHead] = dat;
    GateRxHead = next;
}

void direction_gate_tick_5ms(void)
{
    if(GateSignalAgeTicks < 0xffff)
    {
        GateSignalAgeTicks++;
    }
    if(GateClearTicks > 0)
    {
        GateClearTicks--;
    }
    if(GateScreenTicks > 0)
    {
        GateScreenTicks--;
    }
}

void direction_gate_init(void)
{
    DirectionGate.angle_cdeg = 0;
    DirectionGate.channel1 = 0;
    DirectionGate.channel2 = 0;
    DirectionGate.strength = 0;
    DirectionGate.sequence = 0;
    DirectionGate.valid_frames = 0;
    DirectionGate.invalid_frames = 0;
    DirectionGate.rx_bytes = 0;
    DirectionGate.rx_overflows = 0;
    DirectionGate.signal_valid = 0;
    DirectionGate.phase = DIRECTION_GATE_WAIT_SIGNAL;
    DirectionGate.pass_count = 0;

    GateRxHead = 0;
    GateRxTail = 0;
    GateSignalAgeTicks = 0xffff;
    GateClearTicks = 0;
    GateScreenTicks = 0;
    GateFramePending = 0;
    GateLineLength = 0;
    GateLineOverflow = 0;
    GateLastAngle = 0;
    GatePeakStrength = 0;
    GateHasLastAngle = 0;

    gpio_init(LED_DEBUG, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    direction_gate_stop();
    direction_gate_screen_init();

    uart_init(DIRECTION_GATE_UART, DIRECTION_GATE_UART_BAUD,
              DIRECTION_GATE_UART_TX_PIN, DIRECTION_GATE_UART_RX_PIN);
    uart_rx_interrupt(DIRECTION_GATE_UART, ZF_ENABLE,
                      direction_gate_uart_callback);
}

void direction_gate_update(void)
{
    direction_gate_receive_update();
    direction_gate_screen_update();

#if DIRECTION_GATE_TEST_MODE == DIRECTION_GATE_TEST_UART
    direction_gate_stop();
    return;
#elif DIRECTION_GATE_TEST_MODE == DIRECTION_GATE_TEST_STEER
    direction_gate_write_motor(0, 0);
    if((GateFramePending != 0) && (DirectionGate.signal_valid != 0))
    {
        GateFramePending = 0;
        direction_gate_write_servo(
            direction_gate_steering_control(DirectionGate.angle_cdeg));
    }
    if(GateSignalAgeTicks > DIRECTION_GATE_TIMEOUT_TICKS)
    {
        direction_gate_write_servo(0);
    }
    return;
#else
    if(DirectionGate.phase == DIRECTION_GATE_CLEAR_GATE)
    {
        direction_gate_write_servo(0);
        direction_gate_write_motor(DIRECTION_GATE_CLEAR_SPEED, 0);
        if(GateClearTicks == 0)
        {
            DirectionGate.phase = DIRECTION_GATE_WAIT_SIGNAL;
            GatePeakStrength = 0;
            GateHasLastAngle = 0;
            if((DirectionGate.signal_valid != 0) &&
               (GateSignalAgeTicks <= DIRECTION_GATE_TIMEOUT_TICKS))
            {
                direction_gate_track_frame();
                GateFramePending = 0;
            }
            else
            {
                direction_gate_stop();
            }
        }
        return;
    }

    if((GateFramePending != 0) && (DirectionGate.signal_valid != 0))
    {
        GateFramePending = 0;
        direction_gate_track_frame();
        gpio_set_level(LED_DEBUG, GPIO_LOW);
    }

    if(GateSignalAgeTicks > DIRECTION_GATE_TIMEOUT_TICKS)
    {
        DirectionGate.signal_valid = 0;
        gpio_set_level(LED_DEBUG, GPIO_HIGH);
        if((DirectionGate.phase == DIRECTION_GATE_TRACK) &&
           (GatePeakStrength >= DIRECTION_GATE_PASS_SIGNAL))
        {
            direction_gate_begin_clear();
        }
        else
        {
            DirectionGate.phase = DIRECTION_GATE_WAIT_SIGNAL;
            GatePeakStrength = 0;
            GateHasLastAngle = 0;
            direction_gate_stop();
        }
    }
#endif
}
