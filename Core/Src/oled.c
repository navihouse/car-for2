#include "oled.h"
#include "main.h"
#include "car_config.h"
#include <string.h>

typedef struct { char character; uint8_t columns[5]; } Glyph;
static const Glyph glyphs[] = {
    {'0',{0x3E,0x51,0x49,0x45,0x3E}}, {'1',{0x00,0x42,0x7F,0x40,0x00}},
    {'2',{0x42,0x61,0x51,0x49,0x46}}, {'3',{0x21,0x41,0x45,0x4B,0x31}},
    {'4',{0x18,0x14,0x12,0x7F,0x10}}, {'5',{0x27,0x45,0x45,0x45,0x39}},
    {'6',{0x3C,0x4A,0x49,0x49,0x30}}, {'7',{0x01,0x71,0x09,0x05,0x03}},
    {'8',{0x36,0x49,0x49,0x49,0x36}}, {'9',{0x06,0x49,0x49,0x29,0x1E}},
    {'A',{0x7E,0x11,0x11,0x11,0x7E}}, {'B',{0x7F,0x49,0x49,0x49,0x36}},
    {'C',{0x3E,0x41,0x41,0x41,0x22}}, {'D',{0x7F,0x41,0x41,0x22,0x1C}},
    {'E',{0x7F,0x49,0x49,0x49,0x41}}, {'F',{0x7F,0x09,0x09,0x09,0x01}},
    {'G',{0x3E,0x41,0x49,0x49,0x7A}}, {'H',{0x7F,0x08,0x08,0x08,0x7F}},
    {'I',{0x00,0x41,0x7F,0x41,0x00}}, {'J',{0x20,0x40,0x41,0x3F,0x01}},
    {'K',{0x7F,0x08,0x14,0x22,0x41}}, {'L',{0x7F,0x40,0x40,0x40,0x40}},
    {'M',{0x7F,0x02,0x0C,0x02,0x7F}}, {'N',{0x7F,0x04,0x08,0x10,0x7F}},
    {'O',{0x3E,0x41,0x41,0x41,0x3E}}, {'P',{0x7F,0x09,0x09,0x09,0x06}},
    {'Q',{0x3E,0x41,0x51,0x21,0x5E}}, {'R',{0x7F,0x09,0x19,0x29,0x46}},
    {'S',{0x46,0x49,0x49,0x49,0x31}}, {'T',{0x01,0x01,0x7F,0x01,0x01}},
    {'U',{0x3F,0x40,0x40,0x40,0x3F}}, {'V',{0x1F,0x20,0x40,0x20,0x1F}},
    {'W',{0x3F,0x40,0x38,0x40,0x3F}}, {'X',{0x63,0x14,0x08,0x14,0x63}},
    {'Y',{0x07,0x08,0x70,0x08,0x07}}, {'Z',{0x61,0x51,0x49,0x45,0x43}},
    {':',{0x00,0x36,0x36,0x00,0x00}}, {'.',{0x00,0x60,0x60,0x00,0x00}},
    {'/',{0x20,0x10,0x08,0x04,0x02}}, {'-',{0x08,0x08,0x08,0x08,0x08}},
    {'+',{0x08,0x08,0x3E,0x08,0x08}}
};

static uint8_t framebuffer[1024];
static uint16_t flush_offset;
static bool ready;
static bool flushing;

static void DelayUs(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t cycles = (SystemCoreClock / 1000000U) * us;
    while ((uint32_t)(DWT->CYCCNT - start) < cycles) { }
}

static void SDA(bool high)
{
    HAL_GPIO_WritePin(CAR_OLED_PORT, CAR_OLED_SDA_PIN, high ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void ClockLow(void)
{
    HAL_GPIO_WritePin(CAR_OLED_PORT, CAR_OLED_SCL_PIN, GPIO_PIN_RESET);
}

static bool ClockHigh(void)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t timeout = (SystemCoreClock / 1000000U) * 100U;
    HAL_GPIO_WritePin(CAR_OLED_PORT, CAR_OLED_SCL_PIN, GPIO_PIN_SET);
    while (HAL_GPIO_ReadPin(CAR_OLED_PORT, CAR_OLED_SCL_PIN) == GPIO_PIN_RESET)
    {
        if ((uint32_t)(DWT->CYCCNT - start) >= timeout) { return false; }
    }
    return true;
}

static bool BusStart(void)
{
    SDA(true);
    if (!ClockHigh()) { return false; }
    DelayUs(2U);
    if (HAL_GPIO_ReadPin(CAR_OLED_PORT, CAR_OLED_SDA_PIN) == GPIO_PIN_RESET) { return false; }
    SDA(false);
    DelayUs(2U);
    ClockLow();
    return true;
}

static void BusStop(void)
{
    ClockLow();
    SDA(false);
    DelayUs(2U);
    (void)ClockHigh();
    DelayUs(2U);
    SDA(true);
    DelayUs(2U);
}

static bool WriteByte(uint8_t value)
{
    bool ack;
    for (uint8_t i = 0U; i < 8U; ++i)
    {
        SDA((value & 0x80U) != 0U);
        DelayUs(2U);
        if (!ClockHigh()) { return false; }
        DelayUs(2U);
        ClockLow();
        value <<= 1;
    }
    SDA(true);
    DelayUs(2U);
    if (!ClockHigh()) { return false; }
    DelayUs(2U);
    ack = HAL_GPIO_ReadPin(CAR_OLED_PORT, CAR_OLED_SDA_PIN) == GPIO_PIN_RESET;
    ClockLow();
    return ack;
}

static bool WritePacket(uint8_t control, const uint8_t *data, uint16_t size)
{
    bool ok = BusStart();
    if (ok) { ok = WriteByte((uint8_t)(CAR_OLED_ADDRESS << 1)); }
    if (ok) { ok = WriteByte(control); }
    for (uint16_t i = 0U; ok && i < size; ++i) { ok = WriteByte(data[i]); }
    BusStop();
    return ok;
}

bool OLED_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    static const uint8_t commands[] = {
        0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40, 0x8D, 0x14,
        0x20, 0x02, 0xA1, 0xC8, 0xDA, 0x12, 0x81, 0x7F, 0xD9, 0xF1,
        0xDB, 0x40, 0xA4, 0xA6, 0x2E, 0xAF
    };
    ready = flushing = false;
    __HAL_RCC_GPIOB_CLK_ENABLE();
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    HAL_GPIO_WritePin(CAR_OLED_PORT, CAR_OLED_SCL_PIN | CAR_OLED_SDA_PIN, GPIO_PIN_SET);
    gpio.Pin = CAR_OLED_SCL_PIN | CAR_OLED_SDA_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_OD;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(CAR_OLED_PORT, &gpio);
    HAL_Delay(20U); /* 只在停车状态初始化或重试，不用于行驶中的整帧刷新。 */
    SDA(true);
    for (uint8_t i = 0U; i < 9U; ++i)
    {
        ClockLow();
        DelayUs(2U);
        if (!ClockHigh()) { return false; }
        DelayUs(2U);
    }
    BusStop();
    ready = WritePacket(0x00U, commands, sizeof(commands));
    flush_offset = 0U;
    OLED_Clear();
    return ready;
}

bool OLED_IsReady(void) { return ready; }
bool OLED_CanRender(void) { return ready && !flushing; }
void OLED_Clear(void) { memset(framebuffer, 0, sizeof(framebuffer)); }

void OLED_Text(uint8_t page, const char *text)
{
    uint8_t x = 0U;
    if (page >= 8U) { return; }
    memset(&framebuffer[(uint16_t)page * 128U], 0, 128U);
    while (*text != '\0' && x <= 122U)
    {
        for (uint16_t i = 0U; i < sizeof(glyphs) / sizeof(glyphs[0]); ++i)
        {
            if (glyphs[i].character == *text)
            {
                memcpy(&framebuffer[(uint16_t)page * 128U + x], glyphs[i].columns, 5U);
                break;
            }
        }
        ++text;
        x += 6U;
    }
}

void OLED_StartFrame(void)
{
    if (OLED_CanRender()) { flush_offset = 0U; flushing = true; }
}

void OLED_Task(void)
{
    bool ok = true;
    if (!ready || !flushing) { return; }
    if ((flush_offset % 128U) == 0U)
    {
        uint8_t commands[] = {(uint8_t)(0xB0U + flush_offset / 128U), 0x00U, 0x10U};
        ok = WritePacket(0x00U, commands, sizeof(commands));
    }
    if (ok) { ok = WritePacket(0x40U, &framebuffer[flush_offset], 16U); }
    if (!ok)
    {
        ready = flushing = false;
        return;
    }
    flush_offset += 16U;
    if (flush_offset >= sizeof(framebuffer)) { flushing = false; }
}
