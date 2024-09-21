#ifndef __DRIVER_FMU_H__
#define __DRIVER_FMU_H__

#include "main.h"
#include "COM.h"

/// @brief 计算通信频率
typedef struct _FREQ_calc_
{
	uint32_t last_got_frame_time_ms;  // 最后一次收到此帧的时间，单位：ms
	uint32_t period_ms;  // 周期
	float freq_Hz;  // 频率
}FREQ_calc_t;

void update_freq_calc(FREQ_calc_t *p_freq_calc);

void FMU_Init(void);
void FMU_decode(uint8_t data_in);
void FMU_task(void *argument);

#endif
