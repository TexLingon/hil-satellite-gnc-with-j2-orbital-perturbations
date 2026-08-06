#include "stm32f1xx_hal.h"
#include "clock_settings.h"
#include "UART_settings.h"

IWDG_HandleTypeDef hiwdg;
TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart1_rx;

volatile uint8_t adc_wd_reset = 0;

uint32_t valid_packets_count = 0;
uint32_t crc_errors_count = 0;

uint32_t timestamp = 0;
float altitude = 0.0f;
uint8_t engine_state = 0;
uint8_t cmd_maneuver = 0;
uint8_t adcs_state = 0;
uint8_t alignment_factor = 0;

uint8_t tx_buffer[7];
uint8_t rx_byte;
uint8_t mcu_ring_buffer[11] = {0};
uint8_t CRC8_Calculate(const uint8_t *data, uint8_t length);

int main(void) {
    HAL_Init();

    SystemClock_Config();

    MX_DMA_Init();
    MX_GPIO_Init();
    MX_USART1_UART_Init();

    for (uint8_t i = 0; i < 3; i++) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
        HAL_Delay(80);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
        HAL_Delay(80);
    }

    HAL_Delay(200);

    MX_IWDG_Init();

    HAL_UART_Receive_DMA(&huart1, &rx_byte, 1);

    while (1) {
        if (adc_wd_reset > 0) {
            HAL_IWDG_Refresh(&hiwdg);
            adc_wd_reset = 0;
        }

        __WFI();
    }
}

void SysTick_Handler(void) {
    HAL_IncTick();
}

void TIM2_IRQHandler(void) {
    HAL_TIM_IRQHandler(&htim2);
}

void DMA1_Channel4_IRQHandler(void) {
    HAL_DMA_IRQHandler(&hdma_usart1_tx);
}

void DMA1_Channel5_IRQHandler(void) {
    HAL_DMA_IRQHandler(&hdma_usart1_rx);
}

void USART1_IRQHandler(void) {
    HAL_UART_IRQHandler(&huart1); 
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {

        for (int i = 0; i < 10; i++) {
            mcu_ring_buffer[i] = mcu_ring_buffer[i+1];
        }

        mcu_ring_buffer[10] = rx_byte;

        uint8_t calc_crc = CRC8_Calculate(mcu_ring_buffer, 10);
        uint8_t recv_crc = mcu_ring_buffer[10];

        if (calc_crc == recv_crc) {

            valid_packets_count++;

            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

            uint8_t rx_offset = 0;
            memcpy(&altitude, &mcu_ring_buffer[rx_offset], sizeof(altitude));
            rx_offset += sizeof(altitude);
            memcpy(&timestamp, &mcu_ring_buffer[rx_offset], sizeof(timestamp));
            rx_offset += sizeof(timestamp);
            memcpy(&alignment_factor, &mcu_ring_buffer[rx_offset], sizeof(alignment_factor));
            rx_offset += sizeof(alignment_factor);
            memcpy(&cmd_maneuver, &mcu_ring_buffer[rx_offset], sizeof(cmd_maneuver));

            if (cmd_maneuver == 1) {
                adcs_state = 1;

                if (timestamp % 2 == 0) {
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
                } else {
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
                }
            } else {
                adcs_state = 0;
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
            }

            if ((cmd_maneuver == 1) && (alignment_factor >= 250)) {
                engine_state = 1;
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
            } else {
                engine_state = 0;
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
            }

            memset(tx_buffer, 0, 7);
            uint8_t tx_offset = 0;

            memcpy(&tx_buffer[tx_offset], &timestamp, sizeof(timestamp));
            tx_offset += sizeof(timestamp);

            memcpy(&tx_buffer[tx_offset], &engine_state, sizeof(engine_state));
            tx_offset += sizeof(engine_state);

            memcpy(&tx_buffer[tx_offset], &adcs_state, sizeof(adcs_state));
            tx_offset += sizeof(adcs_state);

            tx_buffer[tx_offset] = CRC8_Calculate(tx_buffer, tx_offset);

            HAL_UART_Transmit_DMA(&huart1, tx_buffer, 7);

            adc_wd_reset = 1;
        } else {
            crc_errors_count++;
        }

        HAL_UART_Receive_DMA(&huart1, &rx_byte, 1);
    }
}

uint8_t CRC8_Calculate(const uint8_t *data, uint8_t length) {
    uint8_t crc = 0x00;
    for (uint8_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}
