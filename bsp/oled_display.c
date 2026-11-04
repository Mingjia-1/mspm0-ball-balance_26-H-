/** @file oled_display.c @brief Non-blocking-page SSD1306 I2C display. */
#include "bsp/oled_display.h"
#include "ti_msp_dl_config.h"

#define OLED_ADDRESS       (0x3CU)
#define OLED_WIDTH         (128U)
#define OLED_TIMEOUT_LOOPS (50000U)

static bool g_online;
static uint8_t g_page;
static uint8_t g_pixels[OLED_WIDTH];

static bool writePacket(const uint8_t *data, uint16_t length)
{
    uint32_t timeout = OLED_TIMEOUT_LOOPS;
    while (((DL_I2C_getControllerStatus(OLED_I2C_INST) &
             DL_I2C_CONTROLLER_STATUS_IDLE) == 0U) && (timeout-- != 0U)) {}
    if (timeout == 0U) return false;
    DL_I2C_flushControllerTXFIFO(OLED_I2C_INST);
    DL_I2C_fillControllerTXFIFO(OLED_I2C_INST, data, length);
    DL_I2C_startControllerTransfer(OLED_I2C_INST, OLED_ADDRESS,
        DL_I2C_CONTROLLER_DIRECTION_TX, length);
    DL_Common_delayCycles(100U);
    timeout = OLED_TIMEOUT_LOOPS;
    while (((DL_I2C_getControllerStatus(OLED_I2C_INST) &
             DL_I2C_CONTROLLER_STATUS_BUSY) != 0U) && (timeout-- != 0U)) {}
    if (timeout == 0U) return false;
    return (DL_I2C_getControllerStatus(OLED_I2C_INST) &
            DL_I2C_CONTROLLER_STATUS_ERROR) == 0U;
}

static bool command(uint8_t value)
{
    uint8_t packet[2] = {0x00U, value};
    return writePacket(packet, 2U);
}

static const uint8_t *glyph(char c)
{
    static const uint8_t blank[5] = {0, 0, 0, 0, 0};
    static const uint8_t punctuation[4][5] = {
        {0, 0x60, 0x60, 0, 0}, {0x08, 0x08, 0x08, 0x08, 0x08},
        {0, 0x36, 0x36, 0, 0}, {0x20, 0x10, 0x08, 0x04, 0x02}
    };
    static const uint8_t digits[10][5] = {
        {0x3E,0x51,0x49,0x45,0x3E},{0,0x42,0x7F,0x40,0},
        {0x42,0x61,0x51,0x49,0x46},{0x21,0x41,0x45,0x4B,0x31},
        {0x18,0x14,0x12,0x7F,0x10},{0x27,0x45,0x45,0x45,0x39},
        {0x3C,0x4A,0x49,0x49,0x30},{0x01,0x71,0x09,0x05,0x03},
        {0x36,0x49,0x49,0x49,0x36},{0x06,0x49,0x49,0x29,0x1E}
    };
    static const uint8_t letters[26][5] = {
        {0x7E,0x11,0x11,0x11,0x7E},{0x7F,0x49,0x49,0x49,0x36},
        {0x3E,0x41,0x41,0x41,0x22},{0x7F,0x41,0x41,0x22,0x1C},
        {0x7F,0x49,0x49,0x49,0x41},{0x7F,0x09,0x09,0x09,0x01},
        {0x3E,0x41,0x49,0x49,0x7A},{0x7F,0x08,0x08,0x08,0x7F},
        {0,0x41,0x7F,0x41,0},{0x20,0x40,0x41,0x3F,0x01},
        {0x7F,0x08,0x14,0x22,0x41},{0x7F,0x40,0x40,0x40,0x40},
        {0x7F,0x02,0x0C,0x02,0x7F},{0x7F,0x04,0x08,0x10,0x7F},
        {0x3E,0x41,0x41,0x41,0x3E},{0x7F,0x09,0x09,0x09,0x06},
        {0x3E,0x41,0x51,0x21,0x5E},{0x7F,0x09,0x19,0x29,0x46},
        {0x46,0x49,0x49,0x49,0x31},{0x01,0x01,0x7F,0x01,0x01},
        {0x3F,0x40,0x40,0x40,0x3F},{0x1F,0x20,0x40,0x20,0x1F},
        {0x3F,0x40,0x38,0x40,0x3F},{0x63,0x14,0x08,0x14,0x63},
        {0x07,0x08,0x70,0x08,0x07},{0x61,0x51,0x49,0x45,0x43}
    };
    if ((c >= '0') && (c <= '9')) return digits[c - '0'];
    if ((c >= 'A') && (c <= 'Z')) return letters[c - 'A'];
    if (c == '.') return punctuation[0];
    if (c == '-') return punctuation[1];
    if (c == ':') return punctuation[2];
    if (c == '/') return punctuation[3];
    return blank;
}

static void putText(const char *text)
{
    uint8_t x = 0U;
    while ((*text != '\0') && (x < 123U)) {
        const uint8_t *g = glyph(*text++);
        uint8_t i;
        for (i = 0U; i < 5U; i++) g_pixels[x++] = g[i];
        g_pixels[x++] = 0U;
    }
}

static void appendUnsigned(char **cursor, uint32_t value, uint8_t digits)
{
    uint32_t divisor = 1U;
    uint8_t i;
    for (i = 1U; i < digits; i++) divisor *= 10U;
    for (i = 0U; i < digits; i++) {
        **cursor = (char)('0' + ((value / divisor) % 10U));
        (*cursor)++;
        divisor /= 10U;
    }
}

static void appendSigned(char **cursor, int32_t value)
{
    if (value < 0) {
        **cursor = '-';
        (*cursor)++;
        value = -value;
    }
    if (value > 9999) value = 9999;
    appendUnsigned(cursor, (uint32_t)value, 4U);
}

static void buildPage(uint8_t page, const OledDisplayModel *model)
{
    char line[22];
    char *p = line;
    uint8_t i;
    for (i = 0U; i < OLED_WIDTH; i++) g_pixels[i] = 0U;

    if (page == 0U) {
        *p++='H'; *p++=' '; *p++='T'; *p++='A'; *p++='S'; *p++='K'; *p++=' ';
        *p++=(char)('0' + model->taskNumber); *p++=' '; *p++='C'; *p++='W';
    } else if (page == 1U) {
        const char *s = model->stateText;
        *p++='S'; *p++='T'; *p++='A'; *p++='T'; *p++='E'; *p++=' ';
        while ((*s != '\0') && ((p - line) < 20)) *p++ = *s++;
    } else if (page == 2U) {
        uint32_t milliseconds = model->elapsedMs;
        if (milliseconds > 999999U) milliseconds = 999999U;
        *p++='T'; *p++='I'; *p++='M'; *p++='E'; *p++=' ';
        appendUnsigned(&p, milliseconds / 1000U, 3U);
        *p++='.';
        appendUnsigned(&p, milliseconds % 1000U, 3U);
        *p++='S';
    } else {
        *p++='D'; *p++=' ';
        appendSigned(&p, model->distanceMm);
        *p++=' '; *p++='E'; *p++=' ';
        appendSigned(&p, model->lineError);
    }
    *p = '\0';
    putText(line);
}

static bool sendPage(uint8_t page)
{
    uint8_t setup[4] = {0x00U, (uint8_t)(0xB0U + page), 0x00U, 0x10U};
    uint8_t packet[8];
    uint16_t x;
    if (!writePacket(setup, 4U)) return false;
    for (x = 0U; x < OLED_WIDTH; x += 7U) {
        uint8_t count = (uint8_t)(((OLED_WIDTH - x) > 7U) ? 7U : (OLED_WIDTH - x));
        uint8_t i;
        packet[0] = 0x40U;
        for (i = 0U; i < count; i++) packet[i + 1U] = g_pixels[x + i];
        if (!writePacket(packet, (uint16_t)(count + 1U))) return false;
    }
    return true;
}

bool OledDisplay_init(void)
{
    static const uint8_t init[] = {
        0xAE,0x20,0x02,0xA1,0xC8,0x81,0x7F,0xA6,0xA8,0x3F,0xD3,0x00,
        0xD5,0x80,0xD9,0xF1,0xDA,0x12,0xDB,0x40,0x8D,0x14,0xAF
    };
    uint8_t i;
    g_online = true;
    g_page = 0U;
    for (i = 0U; i < sizeof(init); i++) {
        if (!command(init[i])) {
            g_online = false;
            break;
        }
    }
    return g_online;
}

void OledDisplay_update(const OledDisplayModel *model)
{
    if (!g_online) return;
    buildPage(g_page, model);
    if (!sendPage(g_page)) {
        g_online = false;
        return;
    }
    g_page = (uint8_t)((g_page + 1U) & 3U);
}

bool OledDisplay_isOnline(void)
{
    return g_online;
}
