#ifndef BUTTONS_H
#define BUTTONS_H

#include <stdbool.h>
#include <stdint.h>

#define CAR_KEY_START  (1U << 0)
#define CAR_KEY_SPEED  (1U << 1)
#define CAR_KEY_LAPS   (1U << 2)

typedef struct
{
    bool raw;
    bool stable;
    bool armed;
    uint32_t changed_at;
} DebouncedKey;

typedef struct
{
    DebouncedKey keys[3];
} CarButtons;

void Buttons_Init(CarButtons *buttons, uint32_t now, uint8_t pressed_mask);
uint8_t Buttons_Update(CarButtons *buttons, uint32_t now, uint8_t pressed_mask);

#endif
