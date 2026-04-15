// header for the dshot300 protocol for brushless motor control

#ifndef __DSHOT_H__
#define __DSHOT_H__

#include "tim.h"
#include <stdint.h>

// DShot300 timing (timer clock = 100 MHz, period = 332 -> 300.3 kHz)
#define MOTOR_BIT_0      125   // ~1.25 µs high time
#define MOTOR_BIT_1      250   // ~2.50 µs high time

#define DSHOT_DMA_BUFFER_SIZE  18   // 16 bits + 2 reset pulses per motor
#define DSHOT_BURST_BUFFER_SIZE (DSHOT_DMA_BUFFER_SIZE * 2)  // 36 half-words

// Motor outputs
#define MOTOR_1_TIM         (&htim1)
#define MOTOR_1_CHANNEL     TIM_CHANNEL_1
#define MOTOR_2_TIM         (&htim1)
#define MOTOR_2_CHANNEL     TIM_CHANNEL_2

void dshot_init(void);
void dshot_write(uint16_t motor1_throttle, uint16_t motor2_throttle);

#endif /* __DSHOT_H__ */
