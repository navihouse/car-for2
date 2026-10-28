#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "imu_heading.h"
#include "imu_config.h"

static const int16_t bias_raw=-1520; /* -23.2061 dps at +/-500 dps */
static uint32_t Calibrate(ImuHeading *h, uint32_t start)
{
    ImuHeading_Reset(h);
    for (uint32_t i=0;i<IMU_CAL_SAMPLES;i++) {
        uint32_t result=ImuHeading_Sample(h,bias_raw,1U,start+i*IMU_POLL_MS);
        assert(result==(i+1U==IMU_CAL_SAMPLES ? HEADING_READY : HEADING_CALIBRATING));
    }
    assert(h->ready && h->samples==IMU_CAL_SAMPLES);
    assert(fabsf(h->bias_raw-(float)bias_raw)<0.01f);
    return start+(IMU_CAL_SAMPLES-1U)*IMU_POLL_MS;
}
static void TestMeasuredBiasAndScale(void)
{
    ImuHeading h;
    uint32_t now=Calibrate(&h,0U);
    assert(fabsf(h.bias_raw/IMU_GYRO_LSB_PER_DPS-IMU_Z_BIAS_PRIOR_DPS)>0.1f);
    for (uint32_t i=0;i<200U;i++) {
        now+=IMU_POLL_MS;
        assert(ImuHeading_Sample(&h,(int16_t)(bias_raw+5895),0U,now)==HEADING_READY);
    }
    assert(fabsf(h.rate_dps-90.0f)<0.01f);
    assert(fabsf(h.yaw_deg-90.0f)<0.3f);
    ImuHeading_Zero(&h);assert(h.yaw_deg==0.0f);
    now+=IMU_POLL_MS;
    assert(ImuHeading_Sample(&h,(int16_t)(bias_raw-5895),0U,now)==HEADING_READY);
    assert(fabsf(h.rate_dps+90.0f)<0.01f);
}
static void TestStationaryAndReferenceGates(void)
{
    ImuHeading h;ImuHeading_Reset(&h);
    for (uint32_t i=0;i<500U;i++) ImuHeading_Sample(&h,bias_raw,0U,i*5U);
    assert(!h.ready && !h.samples && h.reject_reason==2U);
    for (uint32_t i=0;i<500U;i++) ImuHeading_Sample(&h,0,1U,2500U+i*5U);
    assert(!h.ready && !h.samples && h.reject_reason==1U);
    for (uint32_t i=0;i<200U;i++) ImuHeading_Sample(&h,bias_raw,1U,5000U+i*5U);
    assert(h.samples==200U);
    ImuHeading_Sample(&h,bias_raw,0U,6000U);
    assert(!h.samples && !h.ready);
}
static void TestVarianceGapAndRails(void)
{
    ImuHeading h;ImuHeading_Reset(&h);
    for (uint32_t i=0;i<400U;i++)
        ImuHeading_Sample(&h,(int16_t)(bias_raw+(i%2U ? 100 : -100)),1U,i*5U);
    assert(!h.ready && !h.samples && h.reject_reason==1U);
    ImuHeading_Reset(&h);
    ImuHeading_Sample(&h,bias_raw,1U,0U);
    ImuHeading_Sample(&h,bias_raw,1U,5U);
    ImuHeading_Sample(&h,bias_raw,1U,40U);
    assert(h.samples==1U && h.restarts==1U && h.reject_reason==3U);
    uint32_t now=Calibrate(&h,100U);
    assert(ImuHeading_Sample(&h,bias_raw,0U,now+31U)==HEADING_GAP);
    assert(!h.ready);
    now=Calibrate(&h,100U);
    assert(ImuHeading_Sample(&h,-32768,0U,now+5U)==HEADING_SATURATED);
    assert(!h.ready);
    ImuHeading_Reset(&h);
    assert(ImuHeading_Sample(&h,32767,1U,0U)==HEADING_SATURATED);
    assert(!h.ready);
}
static void TestTimingAndWrap(void)
{
    ImuHeading h;
    uint32_t now=Calibrate(&h,UINT32_MAX-1000U);
    uint32_t count=h.samples;
    assert(ImuHeading_Sample(&h,bias_raw,0U,now)==HEADING_READY);
    assert(h.samples==count && h.yaw_deg==0.0f);
    ImuHeading_Reset(&h);
    for (uint32_t i=0;i<400U;i++) ImuHeading_Sample(&h,bias_raw,1U,i);
    assert(!h.ready); /* 400 samples alone cannot bypass stationary duration. */
}
static void TestManualGyroScale(void)
{
    GyroCheck h;GyroCheck_Begin(&h,0U);
    int16_t axes[3]={0,0,bias_raw};
    for (uint32_t now=0U;now<=4990U;now+=10U) GyroCheck_Sample(&h,axes,now);
    assert(h.state==GYRO_TEST_READOUT && fabsf(h.bias-(float)bias_raw)<0.01f);
    axes[2]=(int16_t)(bias_raw+655);
    GyroCheck_Sample(&h,axes,5000U);
    assert(fabsf(h.rate-10.0f)<0.01f);
}
int main(void)
{
    TestMeasuredBiasAndScale();TestStationaryAndReferenceGates();
    TestVarianceGapAndRails();TestTimingAndWrap();TestManualGyroScale();
    puts("IMU: measured bias, +/-500 scale, stationary/noise gates, gap, rail and clock wrap passed");
    return 0;
}
