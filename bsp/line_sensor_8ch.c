/** @file line_sensor_8ch.c @brief GPIO sampling, majority filter and centroid. */
#include "bsp/line_sensor_8ch.h"
#include "config/control_config.h"
#include "config/hardware_config.h"
#include "ti_msp_dl_config.h"

static volatile uint8_t g_history[H_SENSOR_OVERSAMPLE_COUNT];
static volatile uint8_t g_writeIndex;
static volatile uint8_t g_sampleCount;
static volatile uint8_t g_majorityMask;

static uint8_t readRaw(void)
{
    return (uint8_t)(
        ((DL_GPIO_readPins(LINE_SENSOR_D1_PORT, LINE_SENSOR_D1_PIN) != 0U) ? 0x01U : 0U) |
        ((DL_GPIO_readPins(LINE_SENSOR_D2_PORT, LINE_SENSOR_D2_PIN) != 0U) ? 0x02U : 0U) |
        ((DL_GPIO_readPins(LINE_SENSOR_D3_PORT, LINE_SENSOR_D3_PIN) != 0U) ? 0x04U : 0U) |
        ((DL_GPIO_readPins(LINE_SENSOR_D4_PORT, LINE_SENSOR_D4_PIN) != 0U) ? 0x08U : 0U) |
        ((DL_GPIO_readPins(LINE_SENSOR_D5_PORT, LINE_SENSOR_D5_PIN) != 0U) ? 0x10U : 0U) |
        ((DL_GPIO_readPins(LINE_SENSOR_D6_PORT, LINE_SENSOR_D6_PIN) != 0U) ? 0x20U : 0U) |
        ((DL_GPIO_readPins(LINE_SENSOR_D7_PORT, LINE_SENSOR_D7_PIN) != 0U) ? 0x40U : 0U) |
        ((DL_GPIO_readPins(LINE_SENSOR_D8_PORT, LINE_SENSOR_D8_PIN) != 0U) ? 0x80U : 0U));
}

static uint8_t reverseBits(uint8_t value)
{
    value = (uint8_t)(((value & 0x55U) << 1) | ((value & 0xAAU) >> 1));
    value = (uint8_t)(((value & 0x33U) << 2) | ((value & 0xCCU) >> 2));
    return (uint8_t)((value << 4) | (value >> 4));
}

static uint8_t majority(void)
{
    uint8_t result = 0U;
    uint8_t bit;
    uint8_t sample;
    for (bit = 0U; bit < 8U; bit++) {
        uint8_t count = 0U;
        for (sample = 0U; sample < g_sampleCount; sample++) {
            if ((g_history[sample] & (uint8_t)(1U << bit)) != 0U) count++;
        }
        if (count > (g_sampleCount / 2U)) result |= (uint8_t)(1U << bit);
    }
    return result;
}

void LineSensor8ch_init(void)
{
    uint8_t initial = readRaw();
    uint8_t i;
    for (i = 0U; i < H_SENSOR_OVERSAMPLE_COUNT; i++) g_history[i] = initial;
    g_writeIndex = 0U;
    g_sampleCount = H_SENSOR_OVERSAMPLE_COUNT;
    g_majorityMask = initial;
}

void LineSensor8ch_tick1ms(void)
{
    g_history[g_writeIndex] = readRaw();
    g_writeIndex++;
    if (g_writeIndex >= H_SENSOR_OVERSAMPLE_COUNT) g_writeIndex = 0U;
    if (g_sampleCount < H_SENSOR_OVERSAMPLE_COUNT) g_sampleCount++;
    g_majorityMask = majority();
}

LineSensorReading LineSensor8ch_read(void)
{
    LineSensorReading result = {0U, 0U, 0U, 0U, 0U, 0};
    int32_t positionSum = 0;
    uint8_t bit;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    result.rawMask = g_majorityMask;
    if (primask == 0U) __enable_irq();

    result.activeMask = (H_LINE_SENSOR_ACTIVE_LOW != 0U) ?
        (uint8_t)(~result.rawMask) : result.rawMask;
    if (H_LINE_SENSOR_REVERSED != 0U) {
        result.activeMask = reverseBits(result.activeMask);
    }
    for (bit = 0U; bit < 8U; bit++) {
        if ((result.activeMask & (uint8_t)(1U << bit)) != 0U) {
            positionSum += (int32_t)(bit + 1U) * 1000L;
            result.activeCount++;
        }
    }
    result.visible = (result.activeCount != 0U) ? 1U : 0U;

    /*
     * Field measurement: the A transverse line activates at least four
     * channels, while bends normally activate only two.  Do not require the
     * four-channel cluster to reach both outer sensor groups: lateral offset
     * at A can legitimately produce D2..D5 or D4..D7.
     */
    result.marker =
        (result.activeCount >= H_MARKER_MIN_ACTIVE) ? 1U : 0U;

    if (result.visible != 0U) {
        result.error = positionSum / (int32_t)result.activeCount - 4500L;
    }
    return result;
}
