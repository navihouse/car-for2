#include <assert.h>
#include <stdio.h>
#include "tim.h"
#include "encoder.h"
TIM_HandleTypeDef htim4,htim8;
int HAL_TIM_Encoder_Start(TIM_HandleTypeDef *t,uint32_t ch) { (void)t; (void)ch; return HAL_OK; }
int HAL_TIM_Encoder_Stop(TIM_HandleTypeDef *t,uint32_t ch) { (void)t; (void)ch; return HAL_OK; }
uint32_t HAL_GetTick(void) { return UINT32_MAX-5U; }
void Error_Handler(void) { assert(0); }
int main(void)
{
    htim4.counter=65530U; htim8.counter=5U; Encoder_Init();
    htim4.counter=4U; htim8.counter=65531U;
    Encoder_Update(4U); /* tick and both 16-bit counters wrap */
    assert(encoder_data.left_total==10 && encoder_data.right_total==10);
    assert(encoder_data.left_cps==1000.0f && encoder_data.right_cps==1000.0f);
    assert(Encoder_LastSampleTime()==4U && encoder_data.sample_count==1U);
    htim4.counter=65530U; htim8.counter=5U; Encoder_Update(14U);
    assert(encoder_data.left_total==0 && encoder_data.right_total==0);
    assert(encoder_data.left_delta==-10 && encoder_data.right_delta==-10);
    puts("real encoder: signed travel, counter wrap, millis wrap and cadence passed");
    return 0;
}
