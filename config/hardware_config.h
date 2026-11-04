/** @file hardware_config.h @brief Confirmed wiring and crank-link mechanics. */
#ifndef CONFIG_HARDWARE_CONFIG_H_
#define CONFIG_HARDWARE_CONFIG_H_

#define H_MOTOR_PWM_PERIOD_COUNTS       (1600U)
#define H_LEFT_MOTOR_POLARITY           (-1)
#define H_RIGHT_MOTOR_POLARITY          (-1)
#define H_LEFT_ENCODER_POLARITY         (1L)
#define H_RIGHT_ENCODER_POLARITY        (1L)
#define H_LINE_SENSOR_ACTIVE_LOW        (0U)
#define H_LINE_SENSOR_REVERSED          (0U)

/*
 * Preliminary MG513X value used only by the A-to-B stop and lap plausibility
 * gate. Calibrate it on a measured 1.000 m straight before competition:
 * H_ENCODER_EDGES_PER_METER = average(left_edges, right_edges).
 */
#define H_ENCODER_EDGES_PER_METER       (7600L)
#define H_AB_STOP_DISTANCE_MM           (1550L)
#define H_FULL_LAP_MIN_DISTANCE_MM      (4800L)

/*
 * D36A axis 1 / 42 mm stepper / direct crank-link actuator.
 *
 * The supplied WHEELTEC D36A V1.5 manual defines ON as 1 and OFF as 0.
 * SW1=ON, SW2=OFF, SW3=OFF selects 8 microsteps, so one motor revolution is
 * 1600 pulses. The mechanism drawing specifies 57.1 mm total output travel;
 * its equivalent crank radius is therefore 28.55 mm. Around the manually
 * levelled mid-stroke point, output lift is locally linear with crank angle.
 */
#define H_STEPPER_FULL_STEPS_PER_REV    (200L)
#define H_STEPPER_MICROSTEPS            (8L)
#define H_STEPPER_STEPS_PER_REV         (1600L)
#define H_CRANK_OUTPUT_STROKE_UM       (57100L)
#define H_CRANK_RADIUS_UM              (28550L)
#define H_CRANK_CIRCUMFERENCE_UM      (179385L)
#define H_STEPPER_ENABLE_ACTIVE_LOW     (0U)
#define H_STEPPER_TIMER_HZ              (20000UL)

/*
 * No physical home/limit switch is installed. These inactive-level values
 * remain defined only because the optional homing driver shares this header.
 */
#define H_HOME_SWITCH_ACTIVE_HIGH       (1U)
#define H_HOME_DEBOUNCE_MS              (8U)

/*
 * There is no absolute sensor or limit switch. The operator manually levels
 * the beam before arming; firmware assigns that physical point to the logical
 * centre below. Zero and maximum represent +/-90 degrees from this centre,
 * while closed-loop motion is deliberately restricted to +/-64 microsteps.
 */
#define H_BEAM_LEVEL_OFFSET_STEPS        (400L)
#define H_BEAM_MAX_POSITION_STEPS        (800L)
#define H_BEAM_DIRECTION_PROBE_STEPS      (64L)
#define H_BEAM_HOME_BACKOFF_STEPS           (8L)
#define H_BEAM_CONTROL_RANGE_STEPS         (64L)
#define H_BEAM_ARM_READY_TOLERANCE          (2L)

#endif
