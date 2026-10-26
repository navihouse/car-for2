#ifndef TEST_TIM_H
#define TEST_TIM_H
#include <stdint.h>
typedef struct { uint16_t counter; } TIM_HandleTypeDef;
extern TIM_HandleTypeDef htim4, htim8;
#define TIM_CHANNEL_ALL 0U
#define HAL_OK 0
#define __HAL_TIM_GET_COUNTER(timer) ((timer)->counter)
int HAL_TIM_Encoder_Start(TIM_HandleTypeDef *, uint32_t);
int HAL_TIM_Encoder_Stop(TIM_HandleTypeDef *, uint32_t);
uint32_t HAL_GetTick(void);
void Error_Handler(void);
#endif
