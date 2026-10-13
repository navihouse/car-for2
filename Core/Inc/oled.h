#ifndef OLED_H
#define OLED_H
#include <stdbool.h>
#include <stdint.h>

/* SSD1306 128x64，七位 I2C 地址，外接上拉至 3.3V。 */
bool OLED_Init(void);
bool OLED_IsReady(void);
bool OLED_CanRender(void);
void OLED_Clear(void);
void OLED_Text(uint8_t page, const char *text);
void OLED_StartFrame(void);
/* 每次只发送 16 字节像素，不整帧阻塞主循环。 */
void OLED_Task(void);
#endif
