#include "fan_controller.h"
#include "can.h"
#include "main.h"
#include "tim.h"
#include "usart.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define FAN_DEFAULT_TEMP_OFF_DECI_C       350
#define FAN_DEFAULT_TEMP_ON_DECI_C        400
#define FAN_DEFAULT_TEMP_FULL_DECI_C      600
#define FAN_DEFAULT_MIN_DUTY_PCT          30U
#define FAN_DEFAULT_RAMP_UP_PCT_PER_SEC   20U
#define FAN_DEFAULT_RAMP_DOWN_PCT_PER_SEC 50U
#define FAN_DEFAULT_FALLBACK1_DUTY_PCT    50U
#define FAN_DEFAULT_FALLBACK2_DUTY_PCT    50U
#define FAN_DEFAULT_STALE_HOLD_MS          5000U

#define FAN_TEMP_VALID_MIN_DECI_C         (-400)
#define FAN_TEMP_VALID_MAX_DECI_C         2000
#define FAN_CONTROL_INTERVAL_MS           100U
#define FAN_CAN_INTERVAL_MS               500U
#define FAN_UART_INTERVAL_MS              1000U
#define FAN_TEMP_TIMEOUT_MS               2000U
#define FAN_STARTUP_WAIT_MS               3000U
#define FAN_STAGED_START_DELAY_MS          2500U
#define FAN_TACH_TIMEOUT_MS                1000U
#define FAN_STALL_CONFIRM_MS               5000U
#define FAN_TACH_MIN_PERIOD_US             2500U
#define FAN_TACH_MAX_PERIOD_US             65000U
#define FAN_TACH_RPM_NUMERATOR             30000000UL

#define FAN_COMMAND_LEASE_MAX_SEC          60U
#define FAN_POLICY_HOLD_MAX_SEC            30U
#define FAN_POLICY_RAMP_MIN_PCT_PER_SEC    10U
#define FAN_POLICY_RAMP_MAX_PCT_PER_SEC    100U

#define FAN_CMD_SET_CONTROL                0x01U
#define FAN_CMD_SET_CURVE                  0x02U
#define FAN_CMD_SET_FAILSAFE               0x03U
#define FAN_CMD_RESTORE_DEFAULTS           0x04U
#define FAN_CMD_QUERY                      0x05U
#define FAN_CMD_RESTORE_KEY                0xA5U

#define FAN_CMD_RESULT_OK                  0U
#define FAN_CMD_RESULT_BAD_CRC             1U
#define FAN_CMD_RESULT_BAD_LENGTH          2U
#define FAN_CMD_RESULT_BAD_VALUE           3U
#define FAN_CMD_RESULT_UNSUPPORTED         4U
#define FAN_CMD_RESULT_EXPIRED             5U

#define FAN_FAULT_TACH1                    (1U << 0)
#define FAN_FAULT_TACH2                    (1U << 1)
#define FAN_FAULT_TACH3                    (1U << 2)
#define FAN_FAULT_MOTOR_TEMP_STALE         (1U << 3)
#define FAN_FAULT_CTRL_TEMP_STALE          (1U << 4)
#define FAN_FAULT_PWM1_START               (1U << 5)
#define FAN_FAULT_PWM2_START               (1U << 6)
#define FAN_FAULT_TACH_START               (1U << 7)

typedef enum
{
    FAN_MODE_AUTO = 0U,
    FAN_MODE_MANUAL = 1U,
    FAN_MODE_OFF = 2U
} fan_mode_t;

typedef enum
{
    FAN_FAILSAFE_HOLD_LAST = 0U,
    FAN_FAILSAFE_FALLBACK = 1U,
    FAN_FAILSAFE_FULL = 2U
} fan_failsafe_t;

typedef struct
{
    volatile uint16_t last_capture;
    volatile uint16_t rpm;
    volatile uint32_t last_pulse_tick;
    volatile uint8_t have_capture;
} fan_tach_t;

typedef struct
{
    int16_t motor_max_deci_c;
    int16_t inverter_max_deci_c;
    int16_t igbt_max_deci_c;
    uint32_t motor_tick;
    uint32_t inverter_tick;
    uint32_t igbt_tick;
    uint8_t motor_valid;
    uint8_t inverter_valid;
    uint8_t igbt_valid;
} fan_temp_snapshot_t;

typedef struct
{
    int16_t temp_off_deci_c;
    int16_t temp_on_deci_c;
    int16_t temp_full_deci_c;
    uint8_t min_duty_pct;
    uint8_t ramp_up_pct_per_sec;
    uint8_t ramp_down_pct_per_sec;
    fan_failsafe_t failsafe;
    uint8_t fallback1_duty_pct;
    uint8_t fallback2_duty_pct;
    uint32_t stale_hold_ms;
} fan_policy_t;

static fan_tach_t g_tach[3];

static volatile int16_t g_motor_max_deci_c;
static volatile int16_t g_inverter_max_deci_c;
static volatile int16_t g_igbt_max_deci_c;
static volatile uint32_t g_motor_temp_tick;
static volatile uint32_t g_inverter_temp_tick;
static volatile uint32_t g_igbt_temp_tick;
static volatile uint8_t g_motor_temp_valid;
static volatile uint8_t g_inverter_temp_valid;
static volatile uint8_t g_igbt_temp_valid;

static volatile uint8_t g_pending_command[8];
static volatile uint8_t g_pending_command_dlc;
static volatile uint8_t g_command_pending;

static fan_policy_t g_policy;
static fan_mode_t g_mode;
static uint8_t g_manual1_duty_pct;
static uint8_t g_manual2_duty_pct;
static uint32_t g_remote_deadline_tick;
static uint8_t g_control_sequence;

static uint8_t g_pwm1_duty_pct;
static uint8_t g_pwm2_duty_pct;
static uint8_t g_target1_duty_pct;
static uint8_t g_target2_duty_pct;
static uint8_t g_group1_running;
static uint8_t g_group2_running;
static uint8_t g_last_auto_target1;
static uint8_t g_last_auto_target2;
static uint8_t g_have_auto_target1;
static uint8_t g_have_auto_target2;
static uint32_t g_motor_last_fresh_tick;
static uint32_t g_controller_last_fresh_tick;

static uint8_t g_init_faults;
static uint32_t g_start_tick;
static uint32_t g_last_control_tick;
static uint32_t g_last_can_tick;
static uint32_t g_last_uart_tick;
static uint32_t g_last_group_start_tick;
static uint32_t g_group1_run_start_tick;
static uint32_t g_group2_run_start_tick;
static uint8_t g_group_start_recorded;

static uint8_t g_ack_opcode;
static uint8_t g_ack_sequence;
static uint8_t g_ack_result;
static uint8_t g_ack_pending;
static uint8_t g_policy_status_pending;

static void fan_debug_print(const char *format, ...)
{
    char buffer[192];
    va_list args;
    int length;
    uint16_t send_length;

    va_start(args, format);
    length = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    if (length <= 0)
    {
        return;
    }

    send_length = (uint16_t)((length < (int)sizeof(buffer)) ?
                             length : ((int)sizeof(buffer) - 1));
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)buffer, send_length, 100U);
}

static int16_t read_i16_le(const uint8_t *data)
{
    return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

static void put_u16_be(uint8_t *data, uint8_t index, uint16_t value)
{
    data[index] = (uint8_t)(value >> 8);
    data[index + 1U] = (uint8_t)value;
}

static uint8_t crc8_sae_j1850(const uint8_t *data, uint8_t length)
{
    uint8_t crc = 0xFFU;

    for (uint8_t i = 0U; i < length; ++i)
    {
        crc ^= data[i];
        for (uint8_t bit = 0U; bit < 8U; ++bit)
        {
            crc = (uint8_t)((crc & 0x80U) != 0U ?
                            (uint8_t)((crc << 1) ^ 0x1DU) :
                            (uint8_t)(crc << 1));
        }
    }
    return (uint8_t)(crc ^ 0xFFU);
}

static void load_default_policy(void)
{
    g_policy.temp_off_deci_c = FAN_DEFAULT_TEMP_OFF_DECI_C;
    g_policy.temp_on_deci_c = FAN_DEFAULT_TEMP_ON_DECI_C;
    g_policy.temp_full_deci_c = FAN_DEFAULT_TEMP_FULL_DECI_C;
    g_policy.min_duty_pct = FAN_DEFAULT_MIN_DUTY_PCT;
    g_policy.ramp_up_pct_per_sec = FAN_DEFAULT_RAMP_UP_PCT_PER_SEC;
    g_policy.ramp_down_pct_per_sec = FAN_DEFAULT_RAMP_DOWN_PCT_PER_SEC;
    g_policy.failsafe = FAN_FAILSAFE_FALLBACK;
    g_policy.fallback1_duty_pct = FAN_DEFAULT_FALLBACK1_DUTY_PCT;
    g_policy.fallback2_duty_pct = FAN_DEFAULT_FALLBACK2_DUTY_PCT;
    g_policy.stale_hold_ms = FAN_DEFAULT_STALE_HOLD_MS;
}

static uint8_t decode_max_temperature(const uint8_t data[8], int16_t *max_temp)
{
    uint8_t valid_count = 0U;
    int16_t maximum = FAN_TEMP_VALID_MIN_DECI_C;

    for (uint8_t i = 0U; i < 4U; ++i)
    {
        int16_t value = read_i16_le(&data[i * 2U]);
        if ((value < FAN_TEMP_VALID_MIN_DECI_C) ||
            (value > FAN_TEMP_VALID_MAX_DECI_C))
        {
            continue;
        }
        if ((valid_count == 0U) || (value > maximum))
        {
            maximum = value;
        }
        valid_count++;
    }

    if (valid_count == 0U)
    {
        return 0U;
    }
    *max_temp = maximum;
    return 1U;
}

static fan_temp_snapshot_t get_temperature_snapshot(void)
{
    fan_temp_snapshot_t snapshot;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    snapshot.motor_max_deci_c = g_motor_max_deci_c;
    snapshot.inverter_max_deci_c = g_inverter_max_deci_c;
    snapshot.igbt_max_deci_c = g_igbt_max_deci_c;
    snapshot.motor_tick = g_motor_temp_tick;
    snapshot.inverter_tick = g_inverter_temp_tick;
    snapshot.igbt_tick = g_igbt_temp_tick;
    snapshot.motor_valid = g_motor_temp_valid;
    snapshot.inverter_valid = g_inverter_temp_valid;
    snapshot.igbt_valid = g_igbt_temp_valid;
    if (primask == 0U)
    {
        __enable_irq();
    }
    return snapshot;
}

static uint8_t temperature_is_fresh(uint8_t valid, uint32_t timestamp, uint32_t now)
{
    return (uint8_t)((valid != 0U) && ((now - timestamp) <= FAN_TEMP_TIMEOUT_MS));
}

static uint8_t compute_auto_duty(int16_t temp_deci_c, uint8_t was_running)
{
    uint32_t duty;

    if (was_running != 0U)
    {
        if (temp_deci_c < g_policy.temp_off_deci_c)
        {
            return 0U;
        }
    }
    else if (temp_deci_c < g_policy.temp_on_deci_c)
    {
        return 0U;
    }

    if (temp_deci_c >= g_policy.temp_full_deci_c)
    {
        return 100U;
    }
    if (temp_deci_c <= g_policy.temp_on_deci_c)
    {
        return g_policy.min_duty_pct;
    }

    duty = g_policy.min_duty_pct;
    duty += ((uint32_t)(temp_deci_c - g_policy.temp_on_deci_c) *
             (100U - g_policy.min_duty_pct)) /
            (uint32_t)(g_policy.temp_full_deci_c - g_policy.temp_on_deci_c);
    return (uint8_t)duty;
}

static uint8_t compute_stale_target(uint8_t have_last, uint8_t last_target,
                                    uint8_t fallback, uint32_t last_fresh_tick,
                                    uint32_t now)
{
    if (have_last == 0U)
    {
        if ((now - g_start_tick) < FAN_STARTUP_WAIT_MS)
        {
            return 0U;
        }
    }
    else if ((now - last_fresh_tick) <= g_policy.stale_hold_ms)
    {
        return last_target;
    }

    switch (g_policy.failsafe)
    {
        case FAN_FAILSAFE_HOLD_LAST:
            return (uint8_t)(have_last != 0U ? last_target : fallback);

        case FAN_FAILSAFE_FALLBACK:
            if ((have_last != 0U) && (last_target > fallback))
            {
                return last_target;
            }
            return fallback;

        case FAN_FAILSAFE_FULL:
            return 100U;

        default:
            return 0U;
    }
}

static void set_pwm_duty(TIM_HandleTypeDef *htim, uint32_t channel,
                         uint8_t target_duty, uint8_t *current_duty)
{
    uint32_t ticks;
    uint32_t compare;

    if (target_duty > 100U)
    {
        target_duty = 100U;
    }
    if (*current_duty == target_duty)
    {
        return;
    }

    /* 2N7002 驱动为反相：风扇占空比越高，MCU 的高电平时间越短。 */
    ticks = __HAL_TIM_GET_AUTORELOAD(htim) + 1U;
    compare = (ticks * (100U - target_duty) + 50U) / 100U;
    __HAL_TIM_SET_COMPARE(htim, channel, compare);
    *current_duty = target_duty;
}

static uint8_t ramp_towards(uint8_t current, uint8_t target, uint32_t elapsed_ms)
{
    uint32_t rate;
    uint32_t step;

    if (current == target)
    {
        return current;
    }

    rate = (target > current) ?
           g_policy.ramp_up_pct_per_sec : g_policy.ramp_down_pct_per_sec;
    step = (rate * elapsed_ms) / 1000U;
    if (step == 0U)
    {
        step = 1U;
    }

    if (target > current)
    {
        uint32_t next = (uint32_t)current + step;
        return (uint8_t)(next < target ? next : target);
    }
    return (uint8_t)(step < ((uint32_t)current - target) ?
                     ((uint32_t)current - step) : target);
}

static uint8_t start_is_delayed(uint8_t current, uint8_t target, uint32_t now)
{
    return (uint8_t)((current == 0U) && (target > 0U) &&
                     (g_group_start_recorded != 0U) &&
                     ((now - g_last_group_start_tick) < FAN_STAGED_START_DELAY_MS));
}

static void update_outputs(uint8_t target1, uint8_t target2,
                           uint32_t now, uint32_t elapsed_ms)
{
    uint8_t old1 = g_pwm1_duty_pct;
    uint8_t old2 = g_pwm2_duty_pct;
    uint8_t next1 = old1;
    uint8_t next2 = old2;

    if (start_is_delayed(old1, target1, now) == 0U)
    {
        next1 = ramp_towards(old1, target1, elapsed_ms);
    }
    if ((old1 == 0U) && (next1 > 0U))
    {
        g_last_group_start_tick = now;
        g_group_start_recorded = 1U;
    }

    if (start_is_delayed(old2, target2, now) == 0U)
    {
        next2 = ramp_towards(old2, target2, elapsed_ms);
    }
    if ((old2 == 0U) && (next2 > 0U))
    {
        g_last_group_start_tick = now;
        g_group_start_recorded = 1U;
    }

    set_pwm_duty(&htim1, TIM_CHANNEL_1, next1, &g_pwm1_duty_pct);
    set_pwm_duty(&htim4, TIM_CHANNEL_3, next2, &g_pwm2_duty_pct);

    if ((old1 < g_policy.min_duty_pct) &&
        (g_pwm1_duty_pct >= g_policy.min_duty_pct))
    {
        g_group1_run_start_tick = now;
    }
    if (g_pwm1_duty_pct < g_policy.min_duty_pct)
    {
        g_group1_run_start_tick = now;
    }
    if ((old2 < g_policy.min_duty_pct) &&
        (g_pwm2_duty_pct >= g_policy.min_duty_pct))
    {
        g_group2_run_start_tick = now;
    }
    if (g_pwm2_duty_pct < g_policy.min_duty_pct)
    {
        g_group2_run_start_tick = now;
    }

    g_group1_running = (uint8_t)(g_pwm1_duty_pct > 0U);
    g_group2_running = (uint8_t)(g_pwm2_duty_pct > 0U);
}

static uint8_t take_pending_command(uint8_t command[8], uint8_t *dlc)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    if (g_command_pending == 0U)
    {
        if (primask == 0U)
        {
            __enable_irq();
        }
        return 0U;
    }
    for (uint8_t i = 0U; i < 8U; ++i)
    {
        command[i] = g_pending_command[i];
    }
    *dlc = g_pending_command_dlc;
    g_command_pending = 0U;
    if (primask == 0U)
    {
        __enable_irq();
    }
    return 1U;
}

static void set_ack(uint8_t opcode, uint8_t sequence, uint8_t result)
{
    g_ack_opcode = opcode;
    g_ack_sequence = sequence;
    g_ack_result = result;
    g_ack_pending = 1U;
}

static void process_command(uint32_t now)
{
    uint8_t command[8];
    uint8_t dlc;
    uint8_t result = FAN_CMD_RESULT_OK;

    /* 先把上一条命令的应答发出，避免连续命令覆盖应答序号。 */
    if (g_ack_pending != 0U)
    {
        return;
    }
    if (take_pending_command(command, &dlc) == 0U)
    {
        return;
    }
    if (dlc != 8U)
    {
        set_ack(command[0], command[1], FAN_CMD_RESULT_BAD_LENGTH);
        return;
    }
    if (crc8_sae_j1850(command, 7U) != command[7])
    {
        set_ack(command[0], command[1], FAN_CMD_RESULT_BAD_CRC);
        return;
    }

    switch (command[0])
    {
        case FAN_CMD_SET_CONTROL:
            if ((command[2] > FAN_MODE_OFF) ||
                (command[3] > 100U) || (command[4] > 100U) ||
                (command[6] != 0U) ||
                ((command[2] != FAN_MODE_AUTO) &&
                 ((command[5] == 0U) || (command[5] > FAN_COMMAND_LEASE_MAX_SEC))) ||
                ((command[2] == FAN_MODE_OFF) &&
                 ((command[3] != 0U) || (command[4] != 0U))))
            {
                result = FAN_CMD_RESULT_BAD_VALUE;
                break;
            }
            g_mode = (fan_mode_t)command[2];
            g_control_sequence = command[1];
            if (g_mode == FAN_MODE_AUTO)
            {
                g_manual1_duty_pct = 0U;
                g_manual2_duty_pct = 0U;
            }
            else
            {
                g_manual1_duty_pct = (g_mode == FAN_MODE_MANUAL) ? command[3] : 0U;
                g_manual2_duty_pct = (g_mode == FAN_MODE_MANUAL) ? command[4] : 0U;
                g_remote_deadline_tick = now + ((uint32_t)command[5] * 1000U);
            }
            break;

        case FAN_CMD_SET_CURVE:
            if ((command[2] >= command[3]) || (command[3] >= command[4]) ||
                (command[4] > 150U) ||
                (command[5] < 10U) || (command[5] > 100U) ||
                (command[6] < FAN_POLICY_RAMP_MIN_PCT_PER_SEC) ||
                (command[6] > FAN_POLICY_RAMP_MAX_PCT_PER_SEC))
            {
                result = FAN_CMD_RESULT_BAD_VALUE;
                break;
            }
            g_policy.temp_off_deci_c = (int16_t)((uint16_t)command[2] * 10U);
            g_policy.temp_on_deci_c = (int16_t)((uint16_t)command[3] * 10U);
            g_policy.temp_full_deci_c = (int16_t)((uint16_t)command[4] * 10U);
            g_policy.min_duty_pct = command[5];
            g_policy.ramp_up_pct_per_sec = command[6];
            g_policy_status_pending |= (1U << 0);
            break;

        case FAN_CMD_SET_FAILSAFE:
            if ((command[2] > FAN_FAILSAFE_FULL) ||
                (command[3] > 100U) || (command[4] > 100U) ||
                (command[5] > FAN_POLICY_HOLD_MAX_SEC) ||
                (command[6] < FAN_POLICY_RAMP_MIN_PCT_PER_SEC) ||
                (command[6] > FAN_POLICY_RAMP_MAX_PCT_PER_SEC))
            {
                result = FAN_CMD_RESULT_BAD_VALUE;
                break;
            }
            g_policy.failsafe = (fan_failsafe_t)command[2];
            g_policy.fallback1_duty_pct = command[3];
            g_policy.fallback2_duty_pct = command[4];
            g_policy.stale_hold_ms = (uint32_t)command[5] * 1000U;
            g_policy.ramp_down_pct_per_sec = command[6];
            g_policy_status_pending |= (1U << 1);
            break;

        case FAN_CMD_RESTORE_DEFAULTS:
            if ((command[2] != FAN_CMD_RESTORE_KEY) ||
                (command[3] != 0U) || (command[4] != 0U) ||
                (command[5] != 0U) || (command[6] != 0U))
            {
                result = FAN_CMD_RESULT_BAD_VALUE;
                break;
            }
            load_default_policy();
            g_mode = FAN_MODE_AUTO;
            g_manual1_duty_pct = 0U;
            g_manual2_duty_pct = 0U;
            g_policy_status_pending |= (1U << 0) | (1U << 1);
            break;

        case FAN_CMD_QUERY:
            g_policy_status_pending |= (1U << 0) | (1U << 1);
            break;

        default:
            result = FAN_CMD_RESULT_UNSUPPORTED;
            break;
    }

    set_ack(command[0], command[1], result);
}

static void update_remote_lease(uint32_t now)
{
    if ((g_mode != FAN_MODE_AUTO) &&
        ((int32_t)(now - g_remote_deadline_tick) >= 0))
    {
        g_mode = FAN_MODE_AUTO;
        g_manual1_duty_pct = 0U;
        g_manual2_duty_pct = 0U;
        set_ack(FAN_CMD_SET_CONTROL, g_control_sequence, FAN_CMD_RESULT_EXPIRED);
    }
}

static void update_tach_timeout(uint32_t now)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    for (uint8_t i = 0U; i < 3U; ++i)
    {
        if ((g_tach[i].have_capture != 0U) &&
            ((now - g_tach[i].last_pulse_tick) > FAN_TACH_TIMEOUT_MS))
        {
            g_tach[i].rpm = 0U;
            g_tach[i].have_capture = 0U;
        }
    }
    if (primask == 0U)
    {
        __enable_irq();
    }
}

static void get_rpm_snapshot(uint16_t rpm[3])
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    rpm[0] = g_tach[0].rpm;
    rpm[1] = g_tach[1].rpm;
    rpm[2] = g_tach[2].rpm;
    if (primask == 0U)
    {
        __enable_irq();
    }
}

static uint8_t build_fault_flags(uint32_t now, uint8_t motor_fresh,
                                 uint8_t inverter_fresh, uint8_t igbt_fresh,
                                 const uint16_t rpm[3])
{
    uint8_t faults = g_init_faults;

    if (motor_fresh == 0U)
    {
        faults |= FAN_FAULT_MOTOR_TEMP_STALE;
    }
    if ((inverter_fresh == 0U) || (igbt_fresh == 0U))
    {
        faults |= FAN_FAULT_CTRL_TEMP_STALE;
    }
    if ((g_pwm1_duty_pct >= g_policy.min_duty_pct) &&
        ((now - g_group1_run_start_tick) >= FAN_STALL_CONFIRM_MS))
    {
        if (rpm[0] == 0U) faults |= FAN_FAULT_TACH1;
        if (rpm[1] == 0U) faults |= FAN_FAULT_TACH2;
    }
    if ((g_pwm2_duty_pct >= g_policy.min_duty_pct) &&
        ((now - g_group2_run_start_tick) >= FAN_STALL_CONFIRM_MS) &&
        (rpm[2] == 0U))
    {
        faults |= FAN_FAULT_TACH3;
    }
    return faults;
}

static uint8_t send_can(uint32_t id, const uint8_t data[8])
{
    CAN_TxHeaderTypeDef header = {0};
    uint32_t mailbox;

    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0U)
    {
        return 0U;
    }

    header.StdId = id;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = 8U;
    header.TransmitGlobalTime = DISABLE;
    return (uint8_t)(HAL_CAN_AddTxMessage(&hcan, &header,
                                          (uint8_t *)data, &mailbox) == HAL_OK);
}

static void send_command_ack(void)
{
    uint8_t ack[8];

    if (g_ack_pending == 0U)
    {
        return;
    }
    ack[0] = g_ack_opcode;
    ack[1] = g_ack_sequence;
    ack[2] = g_ack_result;
    ack[3] = (uint8_t)((uint8_t)g_mode | ((uint8_t)g_policy.failsafe << 4));
    ack[4] = g_pwm1_duty_pct;
    ack[5] = g_pwm2_duty_pct;
    ack[6] = g_target1_duty_pct;
    ack[7] = g_target2_duty_pct;
    if (send_can(FAN_CAN_ID_COMMAND_ACK, ack) != 0U)
    {
        g_ack_pending = 0U;
    }
}

static void send_one_policy_status(uint32_t now)
{
    uint8_t data[8] = {0};

    if ((g_policy_status_pending & (1U << 0)) != 0U)
    {
        data[0] = (uint8_t)(g_policy.temp_off_deci_c / 10);
        data[1] = (uint8_t)(g_policy.temp_on_deci_c / 10);
        data[2] = (uint8_t)(g_policy.temp_full_deci_c / 10);
        data[3] = g_policy.min_duty_pct;
        data[4] = g_policy.ramp_up_pct_per_sec;
        if (send_can(FAN_CAN_ID_CURVE_STATUS, data) != 0U)
        {
            g_policy_status_pending &= (uint8_t)~(1U << 0);
        }
        return;
    }

    if ((g_policy_status_pending & (1U << 1)) != 0U)
    {
        uint32_t lease_remaining_sec = 0U;
        if ((g_mode != FAN_MODE_AUTO) &&
            ((int32_t)(g_remote_deadline_tick - now) > 0))
        {
            lease_remaining_sec = (g_remote_deadline_tick - now + 999U) / 1000U;
            if (lease_remaining_sec > 255U)
            {
                lease_remaining_sec = 255U;
            }
        }
        data[0] = (uint8_t)g_policy.failsafe;
        data[1] = g_policy.fallback1_duty_pct;
        data[2] = g_policy.fallback2_duty_pct;
        data[3] = (uint8_t)(g_policy.stale_hold_ms / 1000U);
        data[4] = g_policy.ramp_down_pct_per_sec;
        data[5] = (uint8_t)g_mode;
        data[6] = (uint8_t)lease_remaining_sec;
        if (send_can(FAN_CAN_ID_FAILSAFE_STATUS, data) != 0U)
        {
            g_policy_status_pending &= (uint8_t)~(1U << 1);
        }
    }
}

static void send_status(uint32_t now, const fan_temp_snapshot_t *temp,
                        uint8_t motor_fresh, uint8_t inverter_fresh,
                        uint8_t igbt_fresh)
{
    uint8_t status[8] = {0};
    uint8_t diagnostic[8] = {0};
    uint16_t rpm[3];
    int16_t controller_temp = 0;
    uint8_t controller_valid = 0U;
    uint8_t faults;

    get_rpm_snapshot(rpm);
    faults = build_fault_flags(now, motor_fresh, inverter_fresh, igbt_fresh, rpm);

    put_u16_be(status, 0U, rpm[0]);
    put_u16_be(status, 2U, rpm[1]);
    put_u16_be(status, 4U, rpm[2]);
    status[6] = g_pwm1_duty_pct;
    status[7] = g_pwm2_duty_pct;

    if (inverter_fresh != 0U)
    {
        controller_temp = temp->inverter_max_deci_c;
        controller_valid = 1U;
    }
    if ((igbt_fresh != 0U) &&
        ((controller_valid == 0U) || (temp->igbt_max_deci_c > controller_temp)))
    {
        controller_temp = temp->igbt_max_deci_c;
        controller_valid = 1U;
    }

    diagnostic[0] = faults;
    diagnostic[1] = (uint8_t)((motor_fresh != 0U ? (1U << 0) : 0U) |
                              (inverter_fresh != 0U ? (1U << 1) : 0U) |
                              (igbt_fresh != 0U ? (1U << 2) : 0U) |
                              (g_group1_running != 0U ? (1U << 3) : 0U) |
                              (g_group2_running != 0U ? (1U << 4) : 0U) |
                              ((uint8_t)g_mode << 5));
    put_u16_be(diagnostic, 2U,
               (uint16_t)(motor_fresh != 0U ? temp->motor_max_deci_c : 0x7FFF));
    put_u16_be(diagnostic, 4U,
               (uint16_t)(controller_valid != 0U ? controller_temp : 0x7FFF));
    diagnostic[6] = g_target1_duty_pct;
    diagnostic[7] = g_target2_duty_pct;

    (void)send_can(FAN_CAN_ID_STATUS, status);
    (void)send_can(FAN_CAN_ID_DIAGNOSTIC, diagnostic);
}

static void handle_capture(uint8_t index, uint16_t capture)
{
    fan_tach_t *tach = &g_tach[index];
    uint32_t now = HAL_GetTick();
    uint16_t delta;
    uint32_t instant_rpm;

    if ((tach->have_capture == 0U) ||
        ((now - tach->last_pulse_tick) > FAN_TACH_TIMEOUT_MS))
    {
        tach->last_capture = capture;
        tach->last_pulse_tick = now;
        tach->have_capture = 1U;
        return;
    }

    delta = (uint16_t)(capture - tach->last_capture);
    if (delta < FAN_TACH_MIN_PERIOD_US)
    {
        return;
    }

    tach->last_capture = capture;
    tach->last_pulse_tick = now;
    if (delta > FAN_TACH_MAX_PERIOD_US)
    {
        return;
    }

    instant_rpm = FAN_TACH_RPM_NUMERATOR / delta;
    if (tach->rpm == 0U)
    {
        tach->rpm = (uint16_t)instant_rpm;
    }
    else
    {
        tach->rpm = (uint16_t)(((uint32_t)tach->rpm * 3U + instant_rpm) / 4U);
    }
}

void FanController_Init(void)
{
    uint32_t ticks1 = __HAL_TIM_GET_AUTORELOAD(&htim1) + 1U;
    uint32_t ticks4 = __HAL_TIM_GET_AUTORELOAD(&htim4) + 1U;
    HAL_StatusTypeDef tach2_status;
    HAL_StatusTypeDef tach3_ch1_status;
    HAL_StatusTypeDef tach3_ch2_status;

    memset(g_tach, 0, sizeof(g_tach));
    load_default_policy();
    g_mode = FAN_MODE_AUTO;
    g_manual1_duty_pct = 0U;
    g_manual2_duty_pct = 0U;
    g_pwm1_duty_pct = 0U;
    g_pwm2_duty_pct = 0U;
    g_target1_duty_pct = 0U;
    g_target2_duty_pct = 0U;
    g_group1_running = 0U;
    g_group2_running = 0U;
    g_have_auto_target1 = 0U;
    g_have_auto_target2 = 0U;
    g_init_faults = 0U;
    g_group_start_recorded = 0U;
    g_command_pending = 0U;
    g_ack_pending = 0U;
    g_ack_sequence = 0U;
    g_control_sequence = 0U;
    g_policy_status_pending = 0U;

    /* PWM 启动前先写入关闭值，防止定时器启动瞬间产生全速脉冲。 */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ticks1);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, ticks4);

    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
    {
        g_init_faults |= FAN_FAULT_PWM1_START;
    }
    if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3) != HAL_OK)
    {
        g_init_faults |= FAN_FAULT_PWM2_START;
    }
    tach2_status = HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
    tach3_ch1_status = HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
    tach3_ch2_status = HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
    if ((tach2_status != HAL_OK) || (tach3_ch1_status != HAL_OK) ||
        (tach3_ch2_status != HAL_OK))
    {
        g_init_faults |= FAN_FAULT_TACH_START;
    }

    g_start_tick = HAL_GetTick();
    g_last_control_tick = g_start_tick;
    g_last_can_tick = g_start_tick;
    g_last_uart_tick = g_start_tick;
    g_motor_last_fresh_tick = g_start_tick;
    g_controller_last_fresh_tick = g_start_tick;
    g_group1_run_start_tick = g_start_tick;
    g_group2_run_start_tick = g_start_tick;
}

void FanController_Update(void)
{
    uint32_t now = HAL_GetTick();
    fan_temp_snapshot_t temp;
    uint8_t motor_fresh;
    uint8_t inverter_fresh;
    uint8_t igbt_fresh;

    update_remote_lease(now);
    process_command(now);
    /* 命令报文不等待500ms周期，发送邮箱空闲后立即应答。 */
    send_command_ack();
    send_one_policy_status(now);
    temp = get_temperature_snapshot();
    motor_fresh = temperature_is_fresh(temp.motor_valid, temp.motor_tick, now);
    inverter_fresh = temperature_is_fresh(temp.inverter_valid, temp.inverter_tick, now);
    igbt_fresh = temperature_is_fresh(temp.igbt_valid, temp.igbt_tick, now);
    update_tach_timeout(now);

    if ((now - g_last_control_tick) >= FAN_CONTROL_INTERVAL_MS)
    {
        uint32_t elapsed_ms = now - g_last_control_tick;
        uint8_t target1;
        uint8_t target2;

        g_last_control_tick = now;
        if (g_mode == FAN_MODE_MANUAL)
        {
            target1 = g_manual1_duty_pct;
            target2 = g_manual2_duty_pct;
        }
        else if (g_mode == FAN_MODE_OFF)
        {
            target1 = 0U;
            target2 = 0U;
        }
        else
        {
            if (motor_fresh != 0U)
            {
                target1 = compute_auto_duty(temp.motor_max_deci_c, g_group1_running);
                g_last_auto_target1 = target1;
                g_have_auto_target1 = 1U;
                g_motor_last_fresh_tick = now;
            }
            else
            {
                target1 = compute_stale_target(g_have_auto_target1,
                                               g_last_auto_target1,
                                               g_policy.fallback1_duty_pct,
                                               g_motor_last_fresh_tick, now);
            }

            if ((inverter_fresh != 0U) || (igbt_fresh != 0U))
            {
                int16_t controller_temp;
                if (inverter_fresh != 0U)
                {
                    controller_temp = temp.inverter_max_deci_c;
                }
                else
                {
                    controller_temp = temp.igbt_max_deci_c;
                }
                if ((igbt_fresh != 0U) && (temp.igbt_max_deci_c > controller_temp))
                {
                    controller_temp = temp.igbt_max_deci_c;
                }
                target2 = compute_auto_duty(controller_temp, g_group2_running);
                g_last_auto_target2 = target2;
                g_have_auto_target2 = 1U;
                g_controller_last_fresh_tick = now;
            }
            else
            {
                target2 = compute_stale_target(g_have_auto_target2,
                                               g_last_auto_target2,
                                               g_policy.fallback2_duty_pct,
                                               g_controller_last_fresh_tick, now);
            }
        }

        g_target1_duty_pct = target1;
        g_target2_duty_pct = target2;
        update_outputs(target1, target2, now, elapsed_ms);
    }

    if ((now - g_last_can_tick) >= FAN_CAN_INTERVAL_MS)
    {
        g_last_can_tick = now;
        send_status(now, &temp, motor_fresh, inverter_fresh, igbt_fresh);
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    }

    if ((now - g_last_uart_tick) >= FAN_UART_INTERVAL_MS)
    {
        uint16_t rpm[3];
        uint8_t faults;
        g_last_uart_tick = now;
        get_rpm_snapshot(rpm);
        faults = build_fault_flags(now, motor_fresh, inverter_fresh, igbt_fresh, rpm);
        fan_debug_print(
            "FAN: mode=%u safe=%u duty=%u/%u target=%u/%u rpm=%u/%u/%u "
            "temp_dC=%d/%d/%d faults=0x%02X\r\n",
            g_mode, g_policy.failsafe,
            g_pwm1_duty_pct, g_pwm2_duty_pct,
            g_target1_duty_pct, g_target2_duty_pct,
            rpm[0], rpm[1], rpm[2],
            temp.motor_max_deci_c, temp.inverter_max_deci_c,
            temp.igbt_max_deci_c, faults);
    }
}

void FanController_OnCanFrame(const CAN_RxHeaderTypeDef *header,
                              const uint8_t data[8])
{
    int16_t maximum;
    uint32_t now;

    if ((header == NULL) || (data == NULL) ||
        (header->IDE != CAN_ID_STD) || (header->RTR != CAN_RTR_DATA))
    {
        return;
    }

    if (header->StdId == FAN_CAN_ID_COMMAND)
    {
        uint8_t dlc = (uint8_t)(header->DLC <= 8U ? header->DLC : 8U);
        for (uint8_t i = 0U; i < 8U; ++i)
        {
            g_pending_command[i] = (uint8_t)(i < dlc ? data[i] : 0U);
        }
        g_pending_command_dlc = dlc;
        g_command_pending = 1U;
        return;
    }

    if ((header->DLC < 8U) || (decode_max_temperature(data, &maximum) == 0U))
    {
        return;
    }

    now = HAL_GetTick();
    switch (header->StdId)
    {
        case FAN_CAN_ID_MOTOR_TEMP:
            g_motor_max_deci_c = maximum;
            g_motor_temp_tick = now;
            g_motor_temp_valid = 1U;
            break;
        case FAN_CAN_ID_INVERTER_TEMP:
            g_inverter_max_deci_c = maximum;
            g_inverter_temp_tick = now;
            g_inverter_temp_valid = 1U;
            break;
        case FAN_CAN_ID_IGBT_TEMP:
            g_igbt_max_deci_c = maximum;
            g_igbt_temp_tick = now;
            g_igbt_temp_valid = 1U;
            break;
        default:
            break;
    }
}

void FanController_OnInputCapture(TIM_HandleTypeDef *htim)
{
    if ((htim->Instance == TIM2) && (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1))
    {
        handle_capture(0U, (uint16_t)HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1));
    }
    else if (htim->Instance == TIM3)
    {
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
        {
            handle_capture(1U, (uint16_t)HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1));
        }
        else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
        {
            handle_capture(2U, (uint16_t)HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2));
        }
    }
}
