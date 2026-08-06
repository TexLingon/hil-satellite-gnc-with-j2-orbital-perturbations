#include "stm32f1xx_hal.h"
#include <string.h>

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;

void MX_USART1_UART_Init(void);
void MX_DMA_Init(void);
