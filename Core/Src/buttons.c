#include "buttons.h"
#include "car_config.h"

void Buttons_Init(CarButtons *buttons, uint32_t now, uint8_t pressed_mask)
{
    for (uint8_t i = 0U; i < 3U; ++i)
    {
        bool pressed = (pressed_mask & (1U << i)) != 0U;
        buttons->keys[i] = (DebouncedKey){pressed, pressed, !pressed, now};
    }
}

uint8_t Buttons_Update(CarButtons *buttons, uint32_t now, uint8_t pressed_mask)
{
    uint8_t events = 0U;
    for (uint8_t i = 0U; i < 3U; ++i)
    {
        DebouncedKey *key = &buttons->keys[i];
        bool pressed = (pressed_mask & (1U << i)) != 0U;
        if (pressed != key->raw)
        {
            key->raw = pressed;
            key->changed_at = now;
        }
        if ((uint32_t)(now - key->changed_at) >= CAR_BUTTON_DEBOUNCE_MS &&
            key->stable != key->raw)
        {
            key->stable = key->raw;
            if (!key->stable)
            {
                key->armed = true;
            }
            else if (key->armed)
            {
                key->armed = false;
                events |= (uint8_t)(1U << i);
            }
        }
    }
    return events;
}
