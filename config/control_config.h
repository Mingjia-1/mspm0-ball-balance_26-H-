/** @file control_config.h @brief H-topic line following, task 3 and UI tuning. */
#ifndef CONFIG_CONTROL_CONFIG_H_
#define CONFIG_CONTROL_CONFIG_H_

/*
 * The GPIO sensor is sampled by the 1 kHz SysTick interrupt.  A five-sample
 * majority vote rejects single glitches while the line controller consumes a
 * new filtered reading every 5 ms (200 Hz).
 */
#define H_CONTROL_PERIOD_MS                 (5U)
#define H_OLED_PERIOD_MS                    (80U)
#define H_BUTTON_DEBOUNCE_MS                (15U)
#define H_SENSOR_OVERSAMPLE_COUNT           (5U)

/*
 * Encoder speed controller.  The PI loop is updated every 25 ms, while its
 * measured counts are normalized to the original 50 ms target units.
 */
#define H_SPEED_WINDOW_TICKS                (5U)
#define H_SPEED_REFERENCE_WINDOW_MS         (50L)
#define H_ENCODER_EDGES_PER_SPEED_COUNT     (4L)
#define H_SPEED_TARGET_MAX                  (50L)
#define H_SPEED_FF_BASE_PERCENT             (3L)
#define H_SPEED_FF_NUMERATOR                (1L)
#define H_SPEED_FF_DIVISOR                  (2L)
#define H_SPEED_KP_NUMERATOR                (1L)
#define H_SPEED_KP_DIVISOR                  (3L)
#define H_SPEED_KI_DIVISOR                  (160L)
#define H_SPEED_INTEGRAL_LIMIT              (640L)
#define H_SPEED_MIN_OUTPUT_PERCENT          (4L)
#define H_SPEED_MAX_OUTPUT_PERCENT          (40L)

/*
 * Cascaded line control.  The outer 200 Hz PD loop continuously targets the
 * D4/D5 centre and produces wheel-speed targets for the inner 40 Hz PI loops.
 */
#define H_TASK2_BASE_SPEED                  (40L)
#define H_BALANCE_BASE_SPEED                (24L)

/*
 * Task 4 starts gently so the beam loop can counter vehicle acceleration.
 * Wheel targets are scaled as a complete pair, preserving steering ratio.
 */
#define H_TASK4_START_SPEED                   (8L)
#define H_TASK4_ACCEL_RAMP_MS              (1200U)
#define H_TASK4_START_DOWN_FF_UM             (900L)
#define H_TASK4_START_FF_HOLD_MS             (250U)
#define H_TASK2_BEND_ENTRY_SPEED            (30L)
#define H_TASK2_CURVE_SPEED                 (26L)
#define H_TASK2_EDGE_SPEED                  (16L)
#define H_BALANCE_BEND_ENTRY_SPEED          (24L)
#define H_BALANCE_CURVE_SPEED               (19L)
#define H_BALANCE_EDGE_SPEED                (14L)
#define H_MIN_WHEEL_SPEED                   (4L)
#define H_MAX_WHEEL_SPEED                   (48L)
#define H_LINE_TARGET_ERROR                 (0L)
#define H_CENTER_DEADBAND                   (100L)
#define H_BEND_ENTRY_ERROR                  (450L)
#define H_CURVE_ERROR                       (1000L)
#define H_EDGE_ERROR                        (2000L)
#define H_LINE_KP_NUMERATOR                 (1L)
#define H_LINE_KP_DIVISOR                   (120L)
#define H_LINE_KD_NUMERATOR                 (1L)
#define H_LINE_KD_DIVISOR                   (180L)
#define H_LINE_D_MAX                        (8L)
#define H_LINE_MAX_STEERING                 (30L)
#define H_LINE_STEERING_SLEW                (10L)

/* 50 ticks at 5 ms retains a brief 250 ms recovery window. */
#define H_LOST_LINE_TIMEOUT_TICKS           (50U)
#define H_MARKER_MIN_ACTIVE                 (4U)
#define H_MARKER_STABLE_TICKS               (1U)
#define H_START_LINE_EXIT_TICKS             (10U)

/*
 * Task-2 adaptive stop sequence.  Reverse torque is applied while encoder
 * edges are measured in 20 ms windows.  The first near-zero window, or the
 * 70 ms safety limit, switches immediately to TB6612 short braking.
 */
#define H_BRAKE_REVERSE_DUTY_PERCENT        (28)
#define H_BRAKE_MIN_REVERSE_MS              (10U)
#define H_BRAKE_MAX_REVERSE_MS              (70U)
#define H_BRAKE_SPEED_WINDOW_TICKS          (4U)
#define H_BRAKE_ZERO_EDGE_THRESHOLD         (1L)
#define H_BRAKE_ZERO_WINDOWS                (2U)
#define H_FINISH_BEEP_MS                    (150U)

/*
 * Task 3 vision-trajectory controller.
 *
 * Position and velocity come from MaixCam. Logical actuator level is only a
 * safety reference; it is not used to decide whether the ball is stable.
 * Positive right-end lift accelerates the ball toward negative position.
 */
#define H_TASK3_CONTROL_PERIOD_MS             (10U)
#define H_TASK3_TIMEOUT_MS                  (5000U)
#define H_TASK3_POSITIVE_TARGET_MM            (50L)
#define H_TASK3_NEGATIVE_TARGET_MM           (-50L)
#define H_TASK3_START_WINDOW_MM               (12L)
#define H_TASK3_WAYPOINT_WINDOW_MM             (5L)
#define H_TASK3_FINAL_WINDOW_MM                (5L)
#define H_TASK3_SETTLE_VELOCITY_MM_S          (10L)
#define H_TASK3_FINAL_STABLE_MS              (300U)
#define H_TASK3_VISION_ABORT_MS             (1500U)
#define H_TASK3_LOW_CONFIDENCE_HOLD_MS       (300U)

/* Remaining-distance velocity profile and position/velocity feedback. */
#define H_TASK3_PROFILE_MAX_SPEED_MM_S        (25L)
#define H_TASK3_CAPTURE_MAX_SPEED_MM_S        (14L)
#define H_TASK3_PROFILE_BRAKE_ACCEL_MM_S2     (90L)
#define H_TASK3_ACTUATOR_LAG_MS               (900L)
/* Visual PD: limited P drive plus reference-velocity tracking. */
#define H_TASK3_POSITION_KP_NUM                (3L)
#define H_TASK3_POSITION_KP_DEN                (1L)
#define H_TASK3_POSITION_P_LIMIT_MM_S2         (80L)
#define H_TASK3_VELOCITY_KD_NUM                (4L)
#define H_TASK3_VELOCITY_KD_DEN                (1L)
#define H_TASK3_ACCEL_LIMIT_MM_S2             (120L)
#define H_TASK3_ACCEL_TO_LIFT_UM               (20L)
#define H_TASK3_COMMAND_MAX_UM                (3200L)
#define H_TASK3_NEGATIVE_BRAKE_WINDOW_MM        (25L)
/* New direct crank: soften only the +5 -> -5 drive/capture/guard path. */
#define H_TASK3_NEGATIVE_DRIVE_ACCEL_LIMIT_MM_S2 (90L)
#define H_TASK3_NEGATIVE_CAPTURE_ACCEL_LIMIT_MM_S2 (40L)
#define H_TASK3_NEGATIVE_DRIVE_COMMAND_MAX_UM   (2200L)
#define H_TASK3_NEGATIVE_REVERSE_BRAKE_MAX_UM   (1200L)
#define H_TASK3_NEGATIVE_CAPTURE_COMMAND_MAX_UM (1400L)
#define H_TASK3_NEGATIVE_DRIVE_PROBE_MAX_UM     (2600L)
#define H_TASK3_NEGATIVE_CAPTURE_PROBE_MAX_UM   (1800L)
#define H_TASK3_NEGATIVE_COMMAND_SLEW_UM_PER_TICK (100L)
#define H_TASK3_NEGATIVE_GUARD_LIFT_UM          (1400L)
#define H_TASK3_NEGATIVE_GUARD_SLEW_UM_PER_TICK  (200L)
#define H_TASK3_CAPTURE_ACCEL_LIMIT_MM_S2        (70L)
#define H_TASK3_CAPTURE_COMMAND_MAX_UM          (2400L)
#define H_TASK3_CAPTURE_PROBE_MAX_UM            (2400L)
#define H_TASK3_COMMAND_SLEW_UM_PER_TICK       (160L)
#define H_TASK3_RETURN_SLEW_UM_PER_TICK        (400L)
#define H_TASK3_POSITION_DEADBAND_MM             (4L)
#define H_TASK3_HOLD_SPEED_MM_S                  (8L)

/*
 * Independent visual safety envelope. It is deliberately outside the
 * trajectory/PID tuning: predicted outward travel latches a recovery tilt
 * until the camera proves that the ball is moving back toward the pipe.
 */
#define H_TASK3_GUARD_BOUNDARY_MM               (65L)
#define H_TASK3_GUARD_LOOKAHEAD_MS             (900L)
#define H_TASK3_GUARD_TRIGGER_SPEED_MM_S        (10L)
#define H_TASK3_GUARD_RELEASE_SPEED_MM_S         (6L)
#define H_TASK3_GUARD_LIFT_UM                  (3200L)

/*
 * Adaptive stick-slip handling. Progress is measured over time instead of
 * repeatedly restarting a frame counter. BREAKAWAY ramps only in the needed
 * direction and exits only after measured motion; a hard timeout prevents an
 * endless one-microstep oscillation when the ball is mechanically trapped.
 */
#define H_TASK3_STALL_SPEED_MM_S                 (4L)
#define H_TASK3_MOTION_RELEASE_SPEED_MM_S        (12L)
#define H_TASK3_PROGRESS_WINDOW_MS              (450U)
#define H_TASK3_MIN_PROGRESS_MM                   (2L)
#define H_TASK3_BREAKAWAY_EXIT_PROGRESS_MM        (3L)
#define H_TASK3_BREAKAWAY_EXIT_SPEED_MM_S         (8L)
#define H_TASK3_BREAKAWAY_TIMEOUT_MS            (1800U)
#define H_TASK3_PROBE_MIN_UM                     (700L)
#define H_TASK3_PROBE_RAMP_UM_PER_100MS          (120L)
#define H_TASK3_PROBE_MAX_UM                    (3200L)
#define H_TASK3_LEARNED_FRICTION_INITIAL_UM      (900L)
#define H_TASK3_LEARNED_FRICTION_MIN_UM          (500L)
#define H_TASK3_LEARNED_FRICTION_MAX_UM         (3200L)
#define H_TASK3_DYNAMIC_FRICTION_PERCENT          (25L)
#define H_TASK3_OBSERVER_MIN_SPEED_MM_S            (12L)
#define H_TASK3_OBSERVER_SAMPLE_LIMIT_MM_S2        (200L)
#define H_TASK3_OBSERVER_LIMIT_MM_S2                (40L)
#define H_TASK3_OBSERVER_FILTER_DIVISOR              (4L)
#define H_TASK3_MODEL_ACCEL_NUM                     (28L)
#define H_TASK3_MODEL_ACCEL_DEN                   (1000L)
#define H_TASK3_DISTURBANCE_TO_LIFT_UM              (10L)

/*
 * Tasks 4/5/6 hold controller. Task 6 latches the measured ball position at
 * start, so the same loop can hold any judge-selected point in the safe span.
 */
#define H_TASK456_CONTROL_PERIOD_MS              (10U)
#define H_TASK456_CENTER_TARGET_MM                 (0L)
#define H_TASK456_CENTER_START_WINDOW_MM          (10L)
#define H_TASK456_TARGET_LIMIT_MM                (100L)
#define H_TASK456_SETTLE_VELOCITY_MM_S            (10L)
#define H_TASK456_VISION_ABORT_MS                (500U)
#define H_TASK456_GUARD_BOUNDARY_MM              (115L)
#define H_TASK456_COMMAND_MAX_UM                (3200L)

/* Independent task 4/5/6 visual hold loop; no task-3 trajectory state. */
#define H_TASK456_POSITION_KP_NUM                  (2L)
#define H_TASK456_POSITION_KP_DEN                  (1L)
#define H_TASK456_VELOCITY_KD_NUM                  (4L)
#define H_TASK456_VELOCITY_KD_DEN                  (1L)
#define H_TASK456_ACCEL_LIMIT_MM_S2              (110L)
#define H_TASK456_ACCEL_TO_LIFT_UM                (20L)
#define H_TASK456_POSITION_DEADBAND_MM             (4L)
#define H_TASK456_HOLD_SPEED_MM_S                  (8L)
#define H_TASK456_STALL_SPEED_MM_S                 (4L)
#define H_TASK456_PROGRESS_WINDOW_MS             (500U)
#define H_TASK456_MIN_PROGRESS_MM                  (2L)
#define H_TASK456_BREAKAWAY_EXIT_PROGRESS_MM       (3L)
#define H_TASK456_BREAKAWAY_EXIT_SPEED_MM_S        (8L)
#define H_TASK456_BREAKAWAY_MIN_UM               (900L)
#define H_TASK456_BREAKAWAY_RAMP_UM_PER_100MS    (120L)
#define H_TASK456_BREAKAWAY_MAX_UM              (2800L)
#define H_TASK456_COMMAND_SLEW_UM_PER_TICK       (180L)
#define H_TASK456_GUARD_LOOKAHEAD_MS             (500L)
#define H_TASK456_GUARD_TRIGGER_SPEED_MM_S         (8L)

/*
 * JY61P is mounted on the vehicle with its pitch axis along the beam. The
 * angle at A is captured as zero. At 273.1 mm beam length, 1 mdeg changes the
 * endpoint height by about 4.77 um. The initial correction is deliberately
 * limited; flip ANGLE_SIGN after a static nose-up test if the mounting sign
 * is opposite. A stale IMU disables only feedforward, never visual feedback.
 */
#define H_TASK456_IMU_AXIS_ROLL                    (0U)
#define H_TASK456_IMU_AXIS_PITCH                   (1U)
#define H_TASK456_IMU_ANGLE_AXIS     H_TASK456_IMU_AXIS_PITCH
#define H_TASK456_IMU_ANGLE_SIGN                   (1L)
#define H_TASK456_ANGLE_TO_LIFT_NUM              (477L)
#define H_TASK456_ANGLE_TO_LIFT_DEN              (100L)
#define H_TASK456_ANGLE_FF_MAX_UM                 (800L)
#define H_TASK456_ANGLE_INPUT_LIMIT_MDEG        (12000L)
/* D36A limits for the direct crank; keep first motion deliberately gentle. */
#define H_STEPPER_RUN_MAX_PPS                    (120L)
#define H_STEPPER_RUN_ACCEL_PPS2                 (400L)
#define H_STEPPER_SERVO_MAX_PPS                  (240L)
#define H_STEPPER_SERVO_ACCEL_PPS2              (1200L)
#define H_STEPPER_HOME_PPS                        (60L)
#define H_STEPPER_HOME_ACCEL_PPS2                (200L)
#define H_STEPPER_PROBE_TIMEOUT_MS              (9000U)
#define H_STEPPER_RETURN_TIMEOUT_MS             (9000U)
#define H_STEPPER_HOME_TIMEOUT_MS               (9000U)
/*
 * A timeout is a fault indication, never a substitute for detecting A.
 * Keeping task 2 above the old 20 s value makes marker failures observable.
 */
#define H_TASK2_TIMEOUT_MS                  (30000U)
#define H_TASK4_TIMEOUT_MS                  (8000U)
#define H_BALANCE_LAP_TIMEOUT_MS            (30000U)
/* Start-line exit + 3 s rejects the initial A line before a full lap. */
#define H_FULL_LAP_MIN_TIME_MS              (3000U)

#endif

