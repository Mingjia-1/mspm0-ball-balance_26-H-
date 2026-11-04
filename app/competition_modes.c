/** @file competition_modes.c @brief Button UI and H-task completion gates. */
#include "app/competition_modes.h"
#include "bsp/user_input.h"
#include "config/control_config.h"
#include "config/hardware_config.h"

static int32_t absolute32(int32_t value)
{
    return (value < 0) ? -value : value;
}

static uint32_t timeoutForTask(CompetitionTask task)
{
    if (task == COMPETITION_TASK_2) return H_TASK2_TIMEOUT_MS;
    if (task == COMPETITION_TASK_3) return H_TASK3_TIMEOUT_MS;
    if (task == COMPETITION_TASK_4) return H_TASK4_TIMEOUT_MS;
    return H_BALANCE_LAP_TIMEOUT_MS;
}

static void nextTask(CompetitionModes *modes)
{
    if (modes->task == COMPETITION_TASK_2) modes->task = COMPETITION_TASK_3;
    else if (modes->task == COMPETITION_TASK_3) modes->task = COMPETITION_TASK_4;
    else if (modes->task == COMPETITION_TASK_4) modes->task = COMPETITION_TASK_5;
    else if (modes->task == COMPETITION_TASK_5) modes->task = COMPETITION_TASK_6;
    else modes->task = COMPETITION_TASK_2;
}

static uint8_t startPositionValid(const CompetitionModes *modes,
    LineSensorReading sensor, uint8_t balanceReady)
{
    if (modes->task == COMPETITION_TASK_3) return balanceReady;
    if ((modes->task >= COMPETITION_TASK_4) &&
        (balanceReady == 0U)) return 0U;
    if (sensor.marker != 0U) return 1U;
    if ((sensor.visible != 0U) &&
        (absolute32(sensor.error - H_LINE_TARGET_ERROR) <= 1500L)) return 1U;
    return 0U;
}

static void beginRun(CompetitionModes *modes, uint32_t nowMs)
{
    modes->state = COMPETITION_STATE_RUNNING;
    modes->startMs = nowMs;
    modes->elapsedMs = 0U;
    modes->startLineExited = 0U;
    modes->exitStableTicks = 0U;
    modes->markerStableTicks = 0U;
    modes->startEvent = 1U;
}

void CompetitionModes_init(CompetitionModes *modes)
{
    modes->task = COMPETITION_TASK_2;
    modes->state = COMPETITION_STATE_SELECT;
    modes->startMs = 0U;
    modes->elapsedMs = 0U;
    modes->startLineExited = 0U;
    modes->exitStableTicks = 0U;
    modes->markerStableTicks = 0U;
    modes->startEvent = 0U;
}

void CompetitionModes_handleKeys(CompetitionModes *modes, uint8_t keyEvents,
    uint32_t nowMs, LineSensorReading sensor, uint8_t balanceReady)
{
    if ((keyEvents & USER_KEY_ESTOP_RESET) != 0U) {
        if (modes->state == COMPETITION_STATE_ESTOP) {
            modes->state = COMPETITION_STATE_SELECT;
            modes->elapsedMs = 0U;
        } else {
            modes->state = COMPETITION_STATE_ESTOP;
        }
        return;
    }

    if ((keyEvents & USER_KEY_MODE) != 0U) {
        if ((modes->state != COMPETITION_STATE_RUNNING) &&
            (modes->state != COMPETITION_STATE_BRAKING) &&
            (modes->state != COMPETITION_STATE_PAUSED) &&
            (modes->state != COMPETITION_STATE_ARMED)) {
            nextTask(modes);
            modes->state = COMPETITION_STATE_SELECT;
            modes->elapsedMs = 0U;
        }
    }

    if ((keyEvents & USER_KEY_CONFIRM) != 0U) {
        if ((modes->state == COMPETITION_STATE_SELECT) ||
            (modes->state == COMPETITION_STATE_COMPLETE) ||
            (modes->state == COMPETITION_STATE_TIMEOUT) ||
            (modes->state == COMPETITION_STATE_LINE_FAULT) ||
            (modes->state == COMPETITION_STATE_CONTROL_FAULT)) {
            modes->state = COMPETITION_STATE_ARMED;
            modes->elapsedMs = 0U;
        } else if ((modes->state == COMPETITION_STATE_ARMED) &&
                   (startPositionValid(modes, sensor, balanceReady) != 0U)) {
            beginRun(modes, nowMs);
        }
    }

    if ((keyEvents & USER_KEY_START_PAUSE) != 0U) {
        if ((modes->state == COMPETITION_STATE_ARMED) &&
            (startPositionValid(modes, sensor, balanceReady) != 0U)) {
            beginRun(modes, nowMs);
        } else if (modes->state == COMPETITION_STATE_RUNNING) {
            modes->state = COMPETITION_STATE_PAUSED;
        } else if (modes->state == COMPETITION_STATE_PAUSED) {
            modes->state = COMPETITION_STATE_RUNNING;
        }
    }
}

void CompetitionModes_step(CompetitionModes *modes, uint32_t nowMs,
    LineSensorReading sensor, int32_t distanceMm)
{
    if ((modes->state != COMPETITION_STATE_RUNNING) &&
        (modes->state != COMPETITION_STATE_BRAKING) &&
        (modes->state != COMPETITION_STATE_PAUSED)) return;

    modes->elapsedMs = nowMs - modes->startMs;
    if ((modes->task != COMPETITION_TASK_3) &&
        (modes->elapsedMs > timeoutForTask(modes->task))) {
        modes->state = COMPETITION_STATE_TIMEOUT;
        return;
    }
    if ((modes->state == COMPETITION_STATE_PAUSED) ||
        (modes->state == COMPETITION_STATE_BRAKING) ||
        (modes->task == COMPETITION_TASK_3)) return;

    if (sensor.marker == 0U) {
        modes->markerStableTicks = 0U;
        if (modes->exitStableTicks < H_START_LINE_EXIT_TICKS) {
            modes->exitStableTicks++;
        }
        if (modes->exitStableTicks >= H_START_LINE_EXIT_TICKS) {
            modes->startLineExited = 1U;
        }
    } else if (modes->startLineExited != 0U) {
        if (modes->markerStableTicks < H_MARKER_STABLE_TICKS) {
            modes->markerStableTicks++;
        }
    }

    if (modes->task == COMPETITION_TASK_4) {
        if (distanceMm >= H_AB_STOP_DISTANCE_MM) {
            modes->state = COMPETITION_STATE_COMPLETE;
        }
        return;
    }

    if ((modes->startLineExited != 0U) &&
        (modes->elapsedMs >= H_FULL_LAP_MIN_TIME_MS) &&
        (modes->markerStableTicks >= H_MARKER_STABLE_TICKS)) {
        if (modes->task == COMPETITION_TASK_2) {
            modes->state = COMPETITION_STATE_BRAKING;
        } else {
            modes->state = COMPETITION_STATE_COMPLETE;
        }
    }
}

void CompetitionModes_brakeComplete(CompetitionModes *modes, uint32_t nowMs)
{
    if (modes->state == COMPETITION_STATE_BRAKING) {
        modes->elapsedMs = nowMs - modes->startMs;
        modes->state = COMPETITION_STATE_COMPLETE;
    }
}

void CompetitionModes_task3Complete(CompetitionModes *modes, uint32_t nowMs)
{
    if ((modes->task == COMPETITION_TASK_3) &&
        (modes->state == COMPETITION_STATE_RUNNING)) {
        modes->elapsedMs = nowMs - modes->startMs;
        modes->state = COMPETITION_STATE_COMPLETE;
    }
}

void CompetitionModes_controlFault(CompetitionModes *modes)
{
    if ((modes->state == COMPETITION_STATE_ARMED) ||
        (modes->state == COMPETITION_STATE_RUNNING)) {
        modes->state = COMPETITION_STATE_CONTROL_FAULT;
    }
}

void CompetitionModes_lineFault(CompetitionModes *modes)
{
    if (modes->state == COMPETITION_STATE_RUNNING) {
        modes->state = COMPETITION_STATE_LINE_FAULT;
    }
}

uint8_t CompetitionModes_takeStartEvent(CompetitionModes *modes)
{
    uint8_t event = modes->startEvent;
    modes->startEvent = 0U;
    return event;
}

uint8_t CompetitionModes_shouldDrive(const CompetitionModes *modes)
{
    return ((modes->state == COMPETITION_STATE_RUNNING) &&
            (modes->task != COMPETITION_TASK_3)) ? 1U : 0U;
}

uint8_t CompetitionModes_shouldBrake(const CompetitionModes *modes)
{
    return (modes->state == COMPETITION_STATE_BRAKING) ? 1U : 0U;
}

int32_t CompetitionModes_baseSpeed(const CompetitionModes *modes)
{
    return (modes->task == COMPETITION_TASK_2) ?
        H_TASK2_BASE_SPEED : H_BALANCE_BASE_SPEED;
}

const char *CompetitionModes_stateText(const CompetitionModes *modes)
{
    switch (modes->state) {
    case COMPETITION_STATE_SELECT: return "SELECT";
    case COMPETITION_STATE_ARMED: return "ARMED";
    case COMPETITION_STATE_RUNNING: return "RUN";
    case COMPETITION_STATE_BRAKING: return "A HIT";
    case COMPETITION_STATE_PAUSED: return "PAUSE";
    case COMPETITION_STATE_COMPLETE: return "DONE";
    case COMPETITION_STATE_TIMEOUT: return "TIMEOUT";
    case COMPETITION_STATE_LINE_FAULT: return "LINE ERR";
    case COMPETITION_STATE_CONTROL_FAULT: return "CTRL ERR";
    case COMPETITION_STATE_ESTOP: return "ESTOP";
    default: return "UNKNOWN";
    }
}
