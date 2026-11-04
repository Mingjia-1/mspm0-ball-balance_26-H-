/** @file system_app.c @brief Cooperative H-task vehicle and beam application. */
#include "app/system_app.h"
#include "app/competition_modes.h"
#include "app/task3_balance.h"
#include "app/task456_balance.h"
#include "bsp/drive_base.h"
#include "bsp/limit_switch.h"
#include "bsp/line_sensor_8ch.h"
#include "bsp/oled_display.h"
#include "bsp/stepper_d36a.h"
#include "bsp/user_input.h"
#include "config/control_config.h"
#include "config/hardware_config.h"
#include "control/line_controller.h"
#include "control/wheel_speed_controller.h"
#include "ti_msp_dl_config.h"

static volatile uint32_t g_milliseconds;
static uint32_t g_lastControlMs;
static uint32_t g_lastTask3Ms;
static uint32_t g_lastTask456Ms;
static uint32_t g_lastOledMs;
static CompetitionModes g_modes;
static LineController g_lineController;
static WheelSpeedController g_speedController;
static LineSensorReading g_sensor;
static int32_t g_distanceMm;
static int32_t g_lineError;

static uint32_t g_brakeStartMs;
static uint32_t g_brakeZeroMs;
static int32_t g_brakeLeftEdges;
static int32_t g_brakeRightEdges;
static uint8_t g_brakeWindowTicks;
static uint8_t g_brakeZeroWindows;
static uint8_t g_brakeActive;
static uint8_t g_brakeHold;
static uint8_t g_brakeReverseDone;
static uint8_t g_buzzerActive;
static uint32_t g_buzzerOffMs;

static int32_t absolute32(int32_t value)
{
    return (value < 0) ? -value : value;
}

static int32_t distanceMmFromEdges(int32_t edges)
{
    return (edges * 1000L) / H_ENCODER_EDGES_PER_METER;
}

static int32_t task4RampedWheelTarget(
    int32_t nominalTarget, uint32_t elapsedMs)
{
    int32_t rampBaseSpeed;
    int32_t scaledTarget;

    if ((nominalTarget <= 0) ||
        (elapsedMs >= H_TASK4_ACCEL_RAMP_MS)) {
        return nominalTarget;
    }
    rampBaseSpeed = H_TASK4_START_SPEED +
        (H_BALANCE_BASE_SPEED - H_TASK4_START_SPEED) *
        (int32_t)elapsedMs / (int32_t)H_TASK4_ACCEL_RAMP_MS;
    scaledTarget =
        (nominalTarget * rampBaseSpeed +
         H_BALANCE_BASE_SPEED / 2L) / H_BALANCE_BASE_SPEED;
    if (scaledTarget < H_MIN_WHEEL_SPEED) {
        scaledTarget = H_MIN_WHEEL_SPEED;
    }
    return scaledTarget;
}

static void resetBrakeState(void)
{
    g_brakeStartMs = 0U;
    g_brakeZeroMs = 0U;
    g_brakeLeftEdges = 0;
    g_brakeRightEdges = 0;
    g_brakeWindowTicks = 0U;
    g_brakeZeroWindows = 0U;
    g_brakeActive = 0U;
    g_brakeHold = 0U;
    g_brakeReverseDone = 0U;
}

static void resetBrakeWindow(void)
{
    g_brakeLeftEdges = 0;
    g_brakeRightEdges = 0;
    g_brakeWindowTicks = 0U;
}

static uint8_t brakeWindowIsZero(void)
{
    return ((g_brakeLeftEdges <= H_BRAKE_ZERO_EDGE_THRESHOLD) &&
            (g_brakeRightEdges <= H_BRAKE_ZERO_EDGE_THRESHOLD)) ? 1U : 0U;
}

static void startFinishBeep(uint32_t now)
{
    DL_GPIO_setPins(SYSTEM_IO_BUZZER_PORT, SYSTEM_IO_BUZZER_PIN);
    g_buzzerActive = 1U;
    g_buzzerOffMs = now + H_FINISH_BEEP_MS;
}

static void processTask2Brake(uint32_t now,
    DriveBaseEncoderCounts delta)
{
    uint8_t zeroWindow;

    if (g_brakeActive == 0U) {
        g_brakeActive = 1U;
        g_brakeStartMs = now;
        g_brakeZeroMs = 0U;
        g_brakeZeroWindows = 0U;
        g_brakeReverseDone = 0U;
        resetBrakeWindow();
        WheelSpeedController_reset(&g_speedController);
        startFinishBeep(now);
    }

    g_brakeLeftEdges += absolute32(delta.left);
    g_brakeRightEdges += absolute32(delta.right);
    g_brakeWindowTicks++;

    if (g_brakeReverseDone == 0U) {
        DriveBase_setDuty(-H_BRAKE_REVERSE_DUTY_PERCENT,
            -H_BRAKE_REVERSE_DUTY_PERCENT);
        if (g_brakeWindowTicks < H_BRAKE_SPEED_WINDOW_TICKS) return;

        zeroWindow = brakeWindowIsZero();
        if (((uint32_t)(now - g_brakeStartMs) >= H_BRAKE_MIN_REVERSE_MS) &&
            (zeroWindow != 0U)) {
            g_brakeReverseDone = 1U;
            g_brakeZeroMs = now;
            g_brakeZeroWindows = 1U;
        } else if ((uint32_t)(now - g_brakeStartMs) >=
                   H_BRAKE_MAX_REVERSE_MS) {
            g_brakeReverseDone = 1U;
            g_brakeZeroMs = 0U;
            g_brakeZeroWindows = 0U;
        }
        resetBrakeWindow();
        if (g_brakeReverseDone == 0U) return;
        DriveBase_brake();
        return;
    }

    DriveBase_brake();
    if (g_brakeWindowTicks < H_BRAKE_SPEED_WINDOW_TICKS) return;

    zeroWindow = brakeWindowIsZero();
    if (zeroWindow != 0U) {
        if (g_brakeZeroWindows == 0U) g_brakeZeroMs = now;
        if (g_brakeZeroWindows < H_BRAKE_ZERO_WINDOWS) {
            g_brakeZeroWindows++;
        }
    } else {
        g_brakeZeroWindows = 0U;
        g_brakeZeroMs = 0U;
    }

    resetBrakeWindow();
    if (g_brakeZeroWindows >= H_BRAKE_ZERO_WINDOWS) {
        CompetitionModes_brakeComplete(&g_modes, g_brakeZeroMs);
        g_brakeActive = 0U;
        g_brakeHold = 1U;
        DriveBase_brake();
    }
}

static void disableReservedOutputs(void)
{
    DL_GPIO_clearPins(SYSTEM_IO_STEPPER2_EN_N_PORT,
        SYSTEM_IO_STEPPER2_EN_N_PIN);
    DL_GPIO_clearPins(SYSTEM_IO_STEPPER2_DIR_PORT,
        SYSTEM_IO_STEPPER2_DIR_PIN);
    DL_GPIO_clearPins(SYSTEM_IO_BUZZER_PORT, SYSTEM_IO_BUZZER_PIN);
}

void SystemApp_init(void)
{
    g_milliseconds = 0U;
    g_lastControlMs = 0U;
    g_lastTask3Ms = 0U;
    g_lastTask456Ms = 0U;
    g_lastOledMs = 0U;
    g_distanceMm = 0;
    g_lineError = 0;
    g_buzzerActive = 0U;
    g_buzzerOffMs = 0U;
    resetBrakeState();
    disableReservedOutputs();
    DriveBase_init();
    StepperD36a_init();
    LimitSwitch_init();
    LineSensor8ch_init();
    UserInput_init();
    Task3Balance_init();
    Task456Balance_init();
    LineController_reset(&g_lineController);
    WheelSpeedController_reset(&g_speedController);
    CompetitionModes_init(&g_modes);
    g_sensor = LineSensor8ch_read();
    (void)OledDisplay_init();
}

void SystemApp_tick1ms(void)
{
    g_milliseconds++;
    LineSensor8ch_tick1ms();
    LimitSwitch_tick1ms();
    UserInput_tick1ms();
}

void SystemApp_process(void)
{
    uint32_t now = g_milliseconds;
    uint8_t events = UserInput_takeEvents();
    CompetitionTask oldTask = g_modes.task;
    CompetitionState oldState = g_modes.state;
    uint8_t balanceReady;

    if (g_modes.task == COMPETITION_TASK_3) {
        balanceReady = Task3Balance_readyToStart() ? 1U : 0U;
    } else if (g_modes.task >= COMPETITION_TASK_4) {
        balanceReady = Task456Balance_readyToStart() ? 1U : 0U;
    } else {
        balanceReady = 1U;
    }

    if ((g_buzzerActive != 0U) &&
        ((int32_t)(now - g_buzzerOffMs) >= 0)) {
        DL_GPIO_clearPins(SYSTEM_IO_BUZZER_PORT, SYSTEM_IO_BUZZER_PIN);
        g_buzzerActive = 0U;
    }

    g_sensor = LineSensor8ch_read();
    CompetitionModes_handleKeys(
        &g_modes, events, now, g_sensor, balanceReady);

    if ((oldTask == COMPETITION_TASK_3) &&
        ((g_modes.task != COMPETITION_TASK_3) ||
         (g_modes.state == COMPETITION_STATE_ESTOP))) {
        Task3Balance_stop();
    }
    if ((oldTask >= COMPETITION_TASK_4) &&
        ((g_modes.task != oldTask) ||
         (g_modes.state == COMPETITION_STATE_ESTOP))) {
        Task456Balance_stop();
    }
    if ((g_modes.task == COMPETITION_TASK_3) &&
        (g_modes.state == COMPETITION_STATE_ARMED) &&
        ((oldTask != COMPETITION_TASK_3) ||
         (oldState != COMPETITION_STATE_ARMED))) {
        Task3Balance_arm(now);
        g_lastTask3Ms = now;
    }
    if ((g_modes.task >= COMPETITION_TASK_4) &&
        (g_modes.state == COMPETITION_STATE_ARMED) &&
        ((oldTask != g_modes.task) ||
         (oldState != COMPETITION_STATE_ARMED))) {
        Task456Balance_arm((uint8_t)g_modes.task, now);
        g_lastTask456Ms = now;
    }

    if (CompetitionModes_takeStartEvent(&g_modes) != 0U) {
        DriveBase_resetEncoders();
        LineController_reset(&g_lineController);
        WheelSpeedController_reset(&g_speedController);
        resetBrakeState();
        g_distanceMm = 0;
        g_lastControlMs = now;
        if (g_modes.task == COMPETITION_TASK_3) {
            Task3Balance_start(now);
        } else if (g_modes.task >= COMPETITION_TASK_4) {
            Task456Balance_start(now);
        }
    }

    if ((g_modes.task == COMPETITION_TASK_3) &&
        (g_modes.state != COMPETITION_STATE_SELECT) &&
        (g_modes.state != COMPETITION_STATE_ESTOP) &&
        ((uint32_t)(now - g_lastTask3Ms) >=
         H_TASK3_CONTROL_PERIOD_MS)) {
        g_lastTask3Ms += H_TASK3_CONTROL_PERIOD_MS;
        Task3Balance_tick10ms(now);
        if (Task3Balance_complete()) {
            if (g_modes.state == COMPETITION_STATE_RUNNING) {
                CompetitionModes_task3Complete(&g_modes, now);
                startFinishBeep(now);
            }
        } else if (Task3Balance_faulted()) {
            CompetitionModes_controlFault(&g_modes);
        }
    }

    if ((g_modes.task >= COMPETITION_TASK_4) &&
        (g_modes.state != COMPETITION_STATE_SELECT) &&
        (g_modes.state != COMPETITION_STATE_ESTOP) &&
        ((uint32_t)(now - g_lastTask456Ms) >=
         H_TASK456_CONTROL_PERIOD_MS)) {
        g_lastTask456Ms += H_TASK456_CONTROL_PERIOD_MS;
        Task456Balance_tick10ms(now);
        if (Task456Balance_faulted()) {
            CompetitionModes_controlFault(&g_modes);
        }
    }

    if ((uint32_t)(now - g_lastControlMs) >= H_CONTROL_PERIOD_MS) {
        DriveBaseEncoderCounts delta;
        CompetitionState stateBeforeStep = g_modes.state;
        g_lastControlMs += H_CONTROL_PERIOD_MS;
        delta = DriveBase_takeEncoderDelta();
        g_distanceMm = distanceMmFromEdges(DriveBase_getAverageDistanceEdges());

        CompetitionModes_step(&g_modes, now, g_sensor, g_distanceMm);
        if ((stateBeforeStep == COMPETITION_STATE_RUNNING) &&
            (g_modes.state == COMPETITION_STATE_COMPLETE) &&
            (g_modes.task >= COMPETITION_TASK_4)) {
            startFinishBeep(now);
        }
        if (CompetitionModes_shouldDrive(&g_modes) != 0U) {
            LineControllerOutput line = LineController_step(&g_lineController,
                g_sensor, CompetitionModes_baseSpeed(&g_modes));
            g_lineError = line.error;
            if (g_modes.task == COMPETITION_TASK_4) {
                line.leftTarget = task4RampedWheelTarget(
                    line.leftTarget, g_modes.elapsedMs);
                line.rightTarget = task4RampedWheelTarget(
                    line.rightTarget, g_modes.elapsedMs);
            }
            if (line.fault != 0U) {
                CompetitionModes_lineFault(&g_modes);
                WheelSpeedController_stop(&g_speedController);
            } else {
                WheelSpeedController_step(&g_speedController, delta,
                    line.leftTarget, line.rightTarget);
            }
        } else if (CompetitionModes_shouldBrake(&g_modes) != 0U) {
            processTask2Brake(now, delta);
        } else if ((g_brakeHold != 0U) &&
                   (g_modes.task == COMPETITION_TASK_2) &&
                   (g_modes.state == COMPETITION_STATE_COMPLETE)) {
            DriveBase_brake();
        } else {
            resetBrakeState();
            WheelSpeedController_stop(&g_speedController);
        }
    }

    if ((uint32_t)(now - g_lastOledMs) >= H_OLED_PERIOD_MS) {
        OledDisplayModel model;
        const char *stateText = CompetitionModes_stateText(&g_modes);
        g_lastOledMs += H_OLED_PERIOD_MS;
        if ((g_modes.task == COMPETITION_TASK_3) &&
            ((g_modes.state == COMPETITION_STATE_ARMED) ||
             (g_modes.state == COMPETITION_STATE_RUNNING) ||
             (g_modes.state == COMPETITION_STATE_COMPLETE) ||
             (g_modes.state == COMPETITION_STATE_CONTROL_FAULT))) {
            stateText = Task3Balance_stateText();
        } else if ((g_modes.task >= COMPETITION_TASK_4) &&
                   (g_modes.state != COMPETITION_STATE_SELECT) &&
                   (g_modes.state != COMPETITION_STATE_ESTOP)) {
            stateText = Task456Balance_stateText();
        }
        model.taskNumber = (uint8_t)g_modes.task;
        model.stateText = stateText;
        model.elapsedMs = g_modes.elapsedMs;
        if (g_modes.task == COMPETITION_TASK_3) {
            model.distanceMm = Task3Balance_ballPositionMm();
            model.lineError = Task3Balance_ballVelocityMmPerSecond();
        } else if (g_modes.task >= COMPETITION_TASK_4) {
            model.distanceMm = Task456Balance_ballPositionMm();
            model.lineError = Task456Balance_ballVelocityMmPerSecond();
        } else {
            model.distanceMm = g_distanceMm;
            model.lineError = g_lineError;
        }
        OledDisplay_update(&model);
    }
}
