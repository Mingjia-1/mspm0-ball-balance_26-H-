/** @file drive_base.c @brief Proven TB6612 mapping plus encoder ISR. */
#include "bsp/drive_base.h"
#include "config/hardware_config.h"
#include "ti_msp_dl_config.h"

static volatile int32_t g_leftCount;
static volatile int32_t g_rightCount;
static volatile int32_t g_leftDelta;
static volatile int32_t g_rightDelta;

static int16_t clampPercent(int16_t value)
{
    if (value > 100) return 100;
    if (value < -100) return -100;
    return value;
}

static int32_t absolute32(int32_t value)
{
    return (value < 0) ? -value : value;
}

static void setPwm(DL_TIMER_CC_INDEX index, uint16_t duty)
{
    uint32_t compare = H_MOTOR_PWM_PERIOD_COUNTS -
        ((H_MOTOR_PWM_PERIOD_COUNTS * (uint32_t)duty) / 100U);
    DL_TimerG_setCaptureCompareValue(MOTOR_PWM_INST, compare, index);
}

static void setRight(int16_t duty)
{
    duty = clampPercent((int16_t)(duty * H_RIGHT_MOTOR_POLARITY));
    setPwm(GPIO_MOTOR_PWM_C0_IDX, 0U);
    DL_GPIO_clearPins(MOTOR_DIR_RIGHT_IN1_PORT, MOTOR_DIR_RIGHT_IN1_PIN);
    DL_GPIO_clearPins(MOTOR_DIR_RIGHT_IN2_PORT, MOTOR_DIR_RIGHT_IN2_PIN);
    if (duty > 0) {
        DL_GPIO_setPins(MOTOR_DIR_RIGHT_IN1_PORT, MOTOR_DIR_RIGHT_IN1_PIN);
        setPwm(GPIO_MOTOR_PWM_C0_IDX, (uint16_t)duty);
    } else if (duty < 0) {
        DL_GPIO_setPins(MOTOR_DIR_RIGHT_IN2_PORT, MOTOR_DIR_RIGHT_IN2_PIN);
        setPwm(GPIO_MOTOR_PWM_C0_IDX, (uint16_t)(-duty));
    }
}

static void setLeft(int16_t duty)
{
    duty = clampPercent((int16_t)(duty * H_LEFT_MOTOR_POLARITY));
    setPwm(GPIO_MOTOR_PWM_C1_IDX, 0U);
    DL_GPIO_clearPins(MOTOR_DIR_LEFT_IN1_PORT, MOTOR_DIR_LEFT_IN1_PIN);
    DL_GPIO_clearPins(MOTOR_DIR_LEFT_IN2_PORT, MOTOR_DIR_LEFT_IN2_PIN);
    if (duty > 0) {
        DL_GPIO_setPins(MOTOR_DIR_LEFT_IN1_PORT, MOTOR_DIR_LEFT_IN1_PIN);
        setPwm(GPIO_MOTOR_PWM_C1_IDX, (uint16_t)duty);
    } else if (duty < 0) {
        DL_GPIO_setPins(MOTOR_DIR_LEFT_IN2_PORT, MOTOR_DIR_LEFT_IN2_PIN);
        setPwm(GPIO_MOTOR_PWM_C1_IDX, (uint16_t)(-duty));
    }
}

void DriveBase_init(void)
{
    DriveBase_stop();
    DriveBase_resetEncoders();
    NVIC_ClearPendingIRQ(ENCODER_INT_IRQN);
    NVIC_EnableIRQ(ENCODER_INT_IRQN);
    DL_TimerG_startCounter(MOTOR_PWM_INST);
}

void DriveBase_setDuty(int16_t leftPercent, int16_t rightPercent)
{
    if ((leftPercent == 0) && (rightPercent == 0)) {
        DriveBase_stop();
        return;
    }
    setLeft(leftPercent);
    setRight(rightPercent);
    DL_GPIO_setPins(MOTOR_DIR_STBY_PORT, MOTOR_DIR_STBY_PIN);
}

void DriveBase_stop(void)
{
    setPwm(GPIO_MOTOR_PWM_C0_IDX, 0U);
    setPwm(GPIO_MOTOR_PWM_C1_IDX, 0U);
    DL_GPIO_clearPins(MOTOR_DIR_RIGHT_IN1_PORT, MOTOR_DIR_RIGHT_IN1_PIN);
    DL_GPIO_clearPins(MOTOR_DIR_RIGHT_IN2_PORT, MOTOR_DIR_RIGHT_IN2_PIN);
    DL_GPIO_clearPins(MOTOR_DIR_LEFT_IN1_PORT, MOTOR_DIR_LEFT_IN1_PIN);
    DL_GPIO_clearPins(MOTOR_DIR_LEFT_IN2_PORT, MOTOR_DIR_LEFT_IN2_PIN);
    DL_GPIO_clearPins(MOTOR_DIR_STBY_PORT, MOTOR_DIR_STBY_PIN);
}

void DriveBase_brake(void)
{
    /*
     * TB6612 short-brake truth table: STBY high, PWM high and both direction
     * inputs high.  PWM is disabled while changing the direction pins to avoid
     * a transient drive pulse.
     */
    setPwm(GPIO_MOTOR_PWM_C0_IDX, 0U);
    setPwm(GPIO_MOTOR_PWM_C1_IDX, 0U);
    DL_GPIO_setPins(MOTOR_DIR_RIGHT_IN1_PORT, MOTOR_DIR_RIGHT_IN1_PIN);
    DL_GPIO_setPins(MOTOR_DIR_RIGHT_IN2_PORT, MOTOR_DIR_RIGHT_IN2_PIN);
    DL_GPIO_setPins(MOTOR_DIR_LEFT_IN1_PORT, MOTOR_DIR_LEFT_IN1_PIN);
    DL_GPIO_setPins(MOTOR_DIR_LEFT_IN2_PORT, MOTOR_DIR_LEFT_IN2_PIN);
    DL_GPIO_setPins(MOTOR_DIR_STBY_PORT, MOTOR_DIR_STBY_PIN);
    setPwm(GPIO_MOTOR_PWM_C0_IDX, 100U);
    setPwm(GPIO_MOTOR_PWM_C1_IDX, 100U);
}

void DriveBase_resetEncoders(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    g_leftCount = 0;
    g_rightCount = 0;
    g_leftDelta = 0;
    g_rightDelta = 0;
    if (primask == 0U) __enable_irq();
}

DriveBaseEncoderCounts DriveBase_getEncoderCounts(void)
{
    DriveBaseEncoderCounts result;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    result.left = g_leftCount;
    result.right = g_rightCount;
    if (primask == 0U) __enable_irq();
    return result;
}

DriveBaseEncoderCounts DriveBase_takeEncoderDelta(void)
{
    DriveBaseEncoderCounts result;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    result.left = g_leftDelta;
    result.right = g_rightDelta;
    g_leftDelta = 0;
    g_rightDelta = 0;
    if (primask == 0U) __enable_irq();
    return result;
}

int32_t DriveBase_getAverageDistanceEdges(void)
{
    DriveBaseEncoderCounts count = DriveBase_getEncoderCounts();
    return (absolute32(count.left) + absolute32(count.right)) / 2L;
}

void GROUP1_IRQHandler(void)
{
    uint32_t iidx;
    do {
        iidx = DL_GPIO_getPendingInterrupt(ENCODER_PORT);
        if ((iidx == ENCODER_LEFT_A_IIDX) ||
            (iidx == ENCODER_LEFT_B_IIDX)) {
            g_leftCount += H_LEFT_ENCODER_POLARITY;
            g_leftDelta += H_LEFT_ENCODER_POLARITY;
        } else if ((iidx == ENCODER_RIGHT_A_IIDX) ||
                   (iidx == ENCODER_RIGHT_B_IIDX)) {
            g_rightCount += H_RIGHT_ENCODER_POLARITY;
            g_rightDelta += H_RIGHT_ENCODER_POLARITY;
        }
    } while (iidx != DL_GPIO_IIDX_NO_INTR);
}
