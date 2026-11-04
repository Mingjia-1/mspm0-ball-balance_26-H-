/**
 * @file stepper_d36a.c
 * @brief Non-blocking D36A pulse scheduler with acceleration limiting.
 */
#include "bsp/stepper_d36a.h"

#include "config/hardware_config.h"
#include "ti_msp_dl_config.h"

static volatile int32_t g_positionSteps;
static volatile int32_t g_targetSteps;
static volatile int32_t g_currentRatePps;
static volatile int32_t g_requestedRatePps;
static volatile int32_t g_accelerationPps2;
static volatile uint32_t g_phaseAccumulator;
static volatile uint8_t g_enabled;
static volatile uint8_t g_pulseHigh;
static volatile uint8_t g_directionLevel;
static volatile uint8_t g_directionKnown;
static volatile uint8_t g_directionLevelForUp;
static volatile uint8_t g_rawDirectionMode;
static volatile int8_t g_stepSign;

static int32_t absolute32(int32_t value)
{
    return (value < 0) ? -value : value;
}

static void writeEnable(bool enabled)
{
#if H_STEPPER_ENABLE_ACTIVE_LOW
    if (enabled) {
        DL_GPIO_clearPins(
            SYSTEM_IO_STEPPER1_EN_N_PORT, SYSTEM_IO_STEPPER1_EN_N_PIN);
    } else {
        DL_GPIO_setPins(
            SYSTEM_IO_STEPPER1_EN_N_PORT, SYSTEM_IO_STEPPER1_EN_N_PIN);
    }
#else
    if (enabled) {
        DL_GPIO_setPins(
            SYSTEM_IO_STEPPER1_EN_N_PORT, SYSTEM_IO_STEPPER1_EN_N_PIN);
    } else {
        DL_GPIO_clearPins(
            SYSTEM_IO_STEPPER1_EN_N_PORT, SYSTEM_IO_STEPPER1_EN_N_PIN);
    }
#endif
}

static void writeDirection(uint8_t high)
{
    if (high != 0U) {
        DL_GPIO_setPins(
            SYSTEM_IO_STEPPER1_DIR_PORT, SYSTEM_IO_STEPPER1_DIR_PIN);
    } else {
        DL_GPIO_clearPins(
            SYSTEM_IO_STEPPER1_DIR_PORT, SYSTEM_IO_STEPPER1_DIR_PIN);
    }
    g_directionLevel = high;
}

void StepperD36a_init(void)
{
    g_positionSteps = 0;
    g_targetSteps = 0;
    g_currentRatePps = 0;
    g_requestedRatePps = 0;
    g_accelerationPps2 = 0;
    g_phaseAccumulator = 0U;
    g_enabled = 0U;
    g_pulseHigh = 0U;
    g_directionKnown = 0U;
    g_directionLevelForUp = 0U;
    g_rawDirectionMode = 0U;
    g_stepSign = 0;
    DL_GPIO_clearPins(BEAM_IO_STEP1_PORT, BEAM_IO_STEP1_PIN);
    writeDirection(0U);
    writeEnable(false);
    NVIC_ClearPendingIRQ(STEP_TIMER_INST_INT_IRQN);
    NVIC_EnableIRQ(STEP_TIMER_INST_INT_IRQN);
}

void StepperD36a_enable(void)
{
    writeEnable(true);
    g_enabled = 1U;
}

void StepperD36a_disable(void)
{
    g_requestedRatePps = 0;
    g_currentRatePps = 0;
    g_stepSign = 0;
    g_enabled = 0U;
    writeEnable(false);
    DL_GPIO_clearPins(BEAM_IO_STEP1_PORT, BEAM_IO_STEP1_PIN);
    g_pulseHigh = 0U;
}

void StepperD36a_emergencyStop(void)
{
    g_targetSteps = g_positionSteps;
    StepperD36a_disable();
}

void StepperD36a_setDirectionLevelForUp(bool high)
{
    g_directionLevelForUp = high ? 1U : 0U;
    g_directionKnown = 1U;
    g_rawDirectionMode = 0U;
}

bool StepperD36a_directionKnown(void)
{
    return g_directionKnown != 0U;
}

bool StepperD36a_directionLevelForUp(void)
{
    return g_directionLevelForUp != 0U;
}

void StepperD36a_setPositionSteps(int32_t positionSteps)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    g_positionSteps = positionSteps;
    g_targetSteps = positionSteps;
    if (primask == 0U) __enable_irq();
}

int32_t StepperD36a_positionSteps(void)
{
    return g_positionSteps;
}

void StepperD36a_setTargetSteps(
    int32_t targetSteps, int32_t maxRatePps, int32_t accelerationPps2)
{
    uint32_t primask;

    if (!g_directionKnown) return;
    if (targetSteps < 0) targetSteps = 0;
    if (targetSteps > H_BEAM_MAX_POSITION_STEPS) {
        targetSteps = H_BEAM_MAX_POSITION_STEPS;
    }
    if (maxRatePps < 1) maxRatePps = 1;
    if (accelerationPps2 < 1) accelerationPps2 = 1;

    primask = __get_PRIMASK();
    __disable_irq();
    g_targetSteps = targetSteps;
    g_requestedRatePps = maxRatePps;
    g_accelerationPps2 = accelerationPps2;
    g_rawDirectionMode = 0U;
    if (primask == 0U) __enable_irq();
    StepperD36a_enable();
}

void StepperD36a_runDirection(
    bool directionLevel, int32_t stepCount, int32_t maxRatePps,
    int32_t accelerationPps2)
{
    uint32_t primask;
    int32_t signedTarget;

    if (stepCount < 0) stepCount = -stepCount;
    if (maxRatePps < 1) maxRatePps = 1;
    if (accelerationPps2 < 1) accelerationPps2 = 1;
    signedTarget = g_positionSteps +
        ((directionLevel ? 1 : -1) * stepCount);

    primask = __get_PRIMASK();
    __disable_irq();
    writeDirection(directionLevel ? 1U : 0U);
    g_stepSign = directionLevel ? 1 : -1;
    g_targetSteps = signedTarget;
    g_requestedRatePps = maxRatePps;
    g_accelerationPps2 = accelerationPps2;
    g_rawDirectionMode = 1U;
    if (primask == 0U) __enable_irq();
    StepperD36a_enable();
}

bool StepperD36a_busy(void)
{
    return (g_positionSteps != g_targetSteps) ||
        (g_currentRatePps != 0);
}

void StepperD36a_tick10ms(void)
{
    int32_t remaining = g_targetSteps - g_positionSteps;
    int32_t desiredSign = (remaining > 0) ? 1 : ((remaining < 0) ? -1 : 0);
    int32_t rateStep = (g_accelerationPps2 + 99) / 100;
    int32_t brakingSteps;
    uint8_t desiredDirection;

    if (rateStep < 1) rateStep = 1;
    if ((desiredSign == 0) || (g_enabled == 0U)) {
        if (g_currentRatePps > rateStep) g_currentRatePps -= rateStep;
        else g_currentRatePps = 0;
        return;
    }

    /*
     * Position commands map signed travel through the learned DIR level.
     * Homing probes intentionally bypass that mapping so both raw levels can
     * be tested again on every arm cycle.
     */
    if ((g_directionKnown != 0U) && (g_rawDirectionMode == 0U)) {
        desiredDirection = (desiredSign > 0) ?
            g_directionLevelForUp : (uint8_t)!g_directionLevelForUp;
    } else {
        desiredDirection = (desiredSign > 0) ? 1U : 0U;
    }

    if ((g_currentRatePps != 0) &&
        (g_stepSign != desiredSign)) {
        g_currentRatePps =
            (g_currentRatePps > rateStep) ?
            (g_currentRatePps - rateStep) : 0;
        return;
    }

    if (g_currentRatePps == 0) {
        writeDirection(desiredDirection);
        g_stepSign = (int8_t)desiredSign;
    }

    brakingSteps = (g_currentRatePps * g_currentRatePps) /
        (2 * g_accelerationPps2);
    if (absolute32(remaining) <= (brakingSteps + 1)) {
        if (g_currentRatePps > rateStep) g_currentRatePps -= rateStep;
        else g_currentRatePps = 1;
    } else if (g_currentRatePps > g_requestedRatePps) {
        g_currentRatePps -= rateStep;
        if (g_currentRatePps < g_requestedRatePps) {
            g_currentRatePps = g_requestedRatePps;
        }
    } else if (g_currentRatePps < g_requestedRatePps) {
        g_currentRatePps += rateStep;
        if (g_currentRatePps > g_requestedRatePps) {
            g_currentRatePps = g_requestedRatePps;
        }
    }
}

void StepperD36a_timerIsr(void)
{
    int32_t remaining;

    if (g_pulseHigh != 0U) {
        DL_GPIO_clearPins(BEAM_IO_STEP1_PORT, BEAM_IO_STEP1_PIN);
        g_pulseHigh = 0U;
    }
    if ((g_enabled == 0U) || (g_currentRatePps <= 0) ||
        (g_stepSign == 0)) return;

    remaining = g_targetSteps - g_positionSteps;
    if (((remaining > 0) && (g_stepSign < 0)) ||
        ((remaining < 0) && (g_stepSign > 0)) ||
        (remaining == 0)) {
        g_currentRatePps = 0;
        g_stepSign = 0;
        return;
    }

    g_phaseAccumulator += (uint32_t)g_currentRatePps;
    if (g_phaseAccumulator >= H_STEPPER_TIMER_HZ) {
        g_phaseAccumulator -= H_STEPPER_TIMER_HZ;
        DL_GPIO_setPins(BEAM_IO_STEP1_PORT, BEAM_IO_STEP1_PIN);
        g_pulseHigh = 1U;
        g_positionSteps += g_stepSign;
    }
}

void STEP_TIMER_INST_IRQHandler(void)
{
    if (DL_TimerG_getPendingInterrupt(STEP_TIMER_INST) ==
        DL_TIMER_IIDX_ZERO) {
        StepperD36a_timerIsr();
    }
}
