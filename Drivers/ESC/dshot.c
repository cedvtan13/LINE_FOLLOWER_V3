#include "dshot.h"
#include "dma.h"

static uint16_t dshot_prepare_packet(uint16_t throttle);
static void dshot_prepare_buffer(uint16_t throttle1, uint16_t throttle2);
void dshot_init(void);
void dshot_write(uint16_t motor1_throttle, uint16_t motor2_throttle);
void HAL_DMA_XferCpltCallback(DMA_HandleTypeDef *hdma);

// DMA handle for TIM1_UP (defined in dma.c)
extern DMA_HandleTypeDef hdma_tim1_up;

// Interleaved DMA buffer: motor1_bit0, motor2_bit0, motor1_bit1, motor2_bit1, ...
static uint16_t dshot_dmabuffer[DSHOT_BURST_BUFFER_SIZE];

// ------------------------------------------------------------------
// DShot packet preparation (throttle: 48..2047, 0 = disarm)
// Returns 16-bit DShot frame with telemetry bit = 0.
// ------------------------------------------------------------------
static uint16_t dshot_prepare_packet(uint16_t throttle) {
    // Clamp throttle to valid DShot range
    if (throttle < 48) throttle = 48;
    if (throttle > 2047) throttle = 2047;

    uint16_t packet = (throttle << 1) | 0;   // telemetry bit = 0

    // CRC4: XOR of nibbles 0..11 (three nibbles)
    unsigned csum = 0;
    unsigned csum_data = packet;
    for (int i = 0; i < 3; i++) {
        csum ^= csum_data;
        csum_data >>= 4;
    }
    csum &= 0x0F;
    packet = (packet << 4) | csum;

    return packet;
}

// ------------------------------------------------------------------
// Fill interleaved DMA buffer for both motors
// ------------------------------------------------------------------
static void dshot_prepare_buffer(uint16_t throttle1, uint16_t throttle2) {
    uint16_t pkt1 = dshot_prepare_packet(throttle1);
    uint16_t pkt2 = dshot_prepare_packet(throttle2);

    for (int i = 0; i < 16; i++) {
        // Bit value for motor1
        dshot_dmabuffer[2*i]   = (pkt1 & 0x8000) ? MOTOR_BIT_1 : MOTOR_BIT_0;
        // Bit value for motor2
        dshot_dmabuffer[2*i+1] = (pkt2 & 0x8000) ? MOTOR_BIT_1 : MOTOR_BIT_0;
        pkt1 <<= 1;
        pkt2 <<= 1;
    }

    // Reset pulses (at least 2 µs low) – two zeros for each motor
    dshot_dmabuffer[32] = 0;
    dshot_dmabuffer[33] = 0;
    dshot_dmabuffer[34] = 0;   // extra safety
    dshot_dmabuffer[35] = 0;
}

// ------------------------------------------------------------------
// Initialize DShot
// ------------------------------------------------------------------
void dshot_init(void) {
    // Start PWM channels (both on TIM1)
    HAL_TIM_PWM_Start(MOTOR_1_TIM, MOTOR_1_CHANNEL);
    HAL_TIM_PWM_Start(MOTOR_2_TIM, MOTOR_2_CHANNEL);

    // Enable main output (required for advanced timers)
    __HAL_TIM_MOE_ENABLE(MOTOR_1_TIM);

    // Configure DMA burst on TIM1:
    // - 2 transfers per update event
    // - Base address = CCR1
    TIM1->DCR = TIM_DMABURSTLENGTH_2TRANSFERS | TIM_DMABASE_CCR1;

    // Enable update DMA request (UDE bit in DIER)
    TIM1->DIER |= TIM_DIER_UDE;
}

// ------------------------------------------------------------------
// Write throttle values to both motors (blocking until previous frame sent)
// ------------------------------------------------------------------
void dshot_write(uint16_t motor1_throttle, uint16_t motor2_throttle) {
    // Prepare interleaved buffer
    dshot_prepare_buffer(motor1_throttle, motor2_throttle);

    // Start DMA transfer to TIM1->DMAR (burst port)
    HAL_DMA_Start_IT(&hdma_tim1_up,
                     (uint32_t)dshot_dmabuffer,
                     (uint32_t)&TIM1->DMAR,
                     DSHOT_BURST_BUFFER_SIZE);

    // The DMA request is already enabled via TIM1->DIER (UDE bit set in init)
    // No extra enable call needed.
}

// ------------------------------------------------------------------
// DMA Transfer Complete Callback
// ------------------------------------------------------------------
void HAL_DMA_XferCpltCallback(DMA_HandleTypeDef *hdma) {
    if (hdma->Instance == hdma_tim1_up.Instance) {
        // Toggle LED on PC13 to confirm DMA activity
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

        // Optionally disable further DMA requests if you want single-shot.
        // TIM1->DIER &= ~TIM_DIER_UDE;
        // (For continuous update, leave UDE enabled.)
    }
}
