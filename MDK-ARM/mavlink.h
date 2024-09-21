#ifndef __DRIVER_FMU_H__
#define __DRIVER_FMU_H__

#include "main.h"
#include "COM.h"

/// @brief ����ͨ��Ƶ��
typedef struct _FREQ_calc_
{
	uint32_t last_got_frame_time_ms;  // ���һ���յ���֡��ʱ�䣬��λ��ms
	uint32_t period_ms;  // ����
	float freq_Hz;  // Ƶ��
}FREQ_calc_t;

void update_freq_calc(FREQ_calc_t *p_freq_calc);

void FMU_Init(void);
void FMU_decode(uint8_t data_in);
void FMU_task(void *argument);

#endif
