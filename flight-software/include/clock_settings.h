#include "stm32f1xx_hal.h"

extern TIM_HandleTypeDef htim2;
extern IWDG_HandleTypeDef hiwdg;

void MX_IWDG_Init(void);

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_TIM2_Init(void);
