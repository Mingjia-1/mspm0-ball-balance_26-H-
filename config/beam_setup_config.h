/**
 * @file beam_setup_config.h
 * @brief Commissioning choices for the current no-limit-switch mechanism.
 */
#ifndef CONFIG_BEAM_SETUP_CONFIG_H_
#define CONFIG_BEAM_SETUP_CONFIG_H_

/*
 * Current hardware has no home switch.  The operator must manually level the
 * beam before entering task 3; firmware then treats that point as logical
 * H_BEAM_LEVEL_OFFSET_STEPS.
 */
#define H_BEAM_USE_HOME_SWITCH              (0U)

/*
 * Confirmed for the new right-side crank-link installation: raw DIR=0 raises
 * the beam's right end and raw DIR=1 lowers it. Arming task 3 accepts the
 * manually levelled beam as its logical reference without a jog test.
 */
#define H_STEPPER_UP_DIRECTION_VERIFIED      (1U)
#define H_STEPPER_DIR_LEVEL_FOR_UP           (0U)
#define H_STEPPER_DIRECTION_TEST_STEPS         (8L)
#define H_STEPPER_DIRECTION_TEST_PPS          (40L)
#define H_STEPPER_DIRECTION_TEST_ACCEL_PPS2  (100L)

#endif
