/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
void SystemClock_Config(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
FLASH_OBProgramInitTypeDef OBInit;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
#if 0
  /*HW semaphore Clock enable*/
  __HAL_RCC_HSEM_CLK_ENABLE();
  /* Activate HSEM notification for Cortex-M4*/
  HAL_HSEM_ActivateNotification(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0));
  /*
  Domain D2 goes to STOP mode (Cortex-M4 in deep-sleep) waiting for Cortex-M7 to
  perform system initialization (system clock config, external memory configuration.. )
  */
  HAL_PWREx_ClearPendingEvent();
  HAL_PWREx_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFE, PWR_D2_DOMAIN);
  /* Clear HSEM flag */
  __HAL_HSEM_CLEAR_FLAG(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0));
#endif
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  SystemClock_Config();
  /* USER CODE END Init */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */
  for(int i=0; i<10; i++)
  {
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
    HAL_Delay(50);
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
    HAL_Delay(500);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}


/***************************************************************************************************************/
/***************************************************************************************************************/
/***************************************************************************************************************/
int _FLASH_Unlock(void)
{
  if(READ_BIT(FLASH->CR1, FLASH_CR_LOCK) != 0U)
  {
    /* Authorize the FLASH Bank1 Registers access */
    WRITE_REG(FLASH->KEYR1, FLASH_KEY1);
    WRITE_REG(FLASH->KEYR1, FLASH_KEY2);

    /* Verify Flash Bank1 is unlocked */
    if (READ_BIT(FLASH->CR1, FLASH_CR_LOCK) != 0U)
    {
      return -1;
    }
  }

  if(READ_BIT(FLASH->CR2, FLASH_CR_LOCK) != 0U)
  {
    /* Authorize the FLASH Bank2 Registers access */
    WRITE_REG(FLASH->KEYR2, FLASH_KEY1);
    WRITE_REG(FLASH->KEYR2, FLASH_KEY2);

    /* Verify Flash Bank2 is unlocked */
    if (READ_BIT(FLASH->CR2, FLASH_CR_LOCK) != 0U)
    {
      return -1;
    }
  }
  return 0;
}

int _FLASH_Lock(void)
{
  /* Set the LOCK Bit to lock the FLASH Bank1 Control Register access */
  SET_BIT(FLASH->CR1, FLASH_CR_LOCK);

  /* Verify Flash Bank1 is locked */
  if (READ_BIT(FLASH->CR1, FLASH_CR_LOCK) == 0U)
  {
    return -1;
  }

  /* Set the LOCK Bit to lock the FLASH Bank2 Control Register access */
  SET_BIT(FLASH->CR2, FLASH_CR_LOCK);

  /* Verify Flash Bank2 is locked */
  if (READ_BIT(FLASH->CR2, FLASH_CR_LOCK) == 0U)
  {
    return -1;
  }

  return 0;
}

int _FLASH_OB_Unlock(void)
{
  if(READ_BIT(FLASH->OPTCR, FLASH_OPTCR_OPTLOCK) != 0U)
  {
    /* Authorizes the Option Byte registers programming */
    WRITE_REG(FLASH->OPTKEYR, FLASH_OPT_KEY1);
    WRITE_REG(FLASH->OPTKEYR, FLASH_OPT_KEY2);

    /* Verify that the Option Bytes are unlocked */
    if (READ_BIT(FLASH->OPTCR, FLASH_OPTCR_OPTLOCK) != 0U)
    {
      return -1;
    }
  }

  return 0;
}


int _FLASH_OB_Lock(void)
{
  /* Set the OPTLOCK Bit to lock the FLASH Option Byte Registers access */
  SET_BIT(FLASH->OPTCR, FLASH_OPTCR_OPTLOCK);

  /* Verify that the Option Bytes are locked */
  if (READ_BIT(FLASH->OPTCR, FLASH_OPTCR_OPTLOCK) == 0U)
  {
    return -1;
  }

  return 0;
}

int _FLASH_WaitForLastOperation(uint32_t Timeout, uint32_t Bank)
{
  /* Wait for the FLASH operation to complete by polling on QW flag to be reset.
     Even if the FLASH operation fails, the QW flag will be reset and an error
     flag will be set */

  uint32_t errorflag = 0;
  uint32_t tickstart;

  tickstart = Timeout;
  if(Bank == FLASH_BANK_1)
  {
    while(READ_BIT(FLASH->SR1, FLASH_FLAG_QW_BANK1) == FLASH_FLAG_QW_BANK1)
    {
      LL_mDelay(1);

      if(tickstart-- == 0)
      {
        return -1;
      }
    }
  }
  else
  {
    while(READ_BIT(FLASH->SR2, FLASH_FLAG_QW_BANK2) == FLASH_FLAG_QW_BANK2)
    {
      LL_mDelay(1);

      if(tickstart-- == 0)
      {
        return -1;
      }
    }
  }


  /* Get Error Flags */
  if (Bank == FLASH_BANK_1)
  {
    errorflag = FLASH->SR1 & FLASH_FLAG_ALL_ERRORS_BANK1;
  }
  else
  {
    errorflag = (FLASH->SR2 & FLASH_FLAG_ALL_ERRORS_BANK2) | 0x80000000U;
  }

  /* In case of error reported in Flash SR1 or SR2 register */
  if((errorflag & 0x7FFFFFFFU) != 0U)
  {
    /* Clear error programming flags */
    if(Bank == FLASH_BANK_1)
    {
      WRITE_REG(FLASH->CCR1, errorflag);
    }
    else
    {
      WRITE_REG(FLASH->CCR2, errorflag);
    }
    return -1;
  }

  /* Check FLASH End of Operation flag  */
  if(Bank == FLASH_BANK_1)
  {
    if(READ_BIT(FLASH->SR1, FLASH_FLAG_EOP_BANK1) == FLASH_FLAG_EOP_BANK1)
    {
      /* Clear FLASH End of Operation pending bit */
      WRITE_REG(FLASH->CCR1, FLASH_FLAG_EOP_BANK1);
    }
  }
  else
  {
    if(READ_BIT(FLASH->SR2, FLASH_FLAG_EOP_BANK2) == FLASH_FLAG_EOP_BANK2)
    {
      /* Clear FLASH End of Operation pending bit */
      WRITE_REG(FLASH->CCR2, FLASH_FLAG_EOP_BANK2);
    }
  }

  return 0;
}

int _FLASH_CRC_WaitForLastOperation(uint32_t Timeout, uint32_t Bank)
{
  uint32_t tickstart;

  /* Select bsyflag depending on Bank */
  /* Wait for the FLASH CRC computation to complete by polling on CRC_BUSY flag to be reset */
  tickstart = Timeout;
  if(Bank == FLASH_BANK_1)
  {
    while(READ_BIT(FLASH->SR1, FLASH_FLAG_CRC_BUSY_BANK1) == FLASH_FLAG_CRC_BUSY_BANK1)
    {
      LL_mDelay(1);

      if(tickstart-- == 0)
      {
        return -1;
      }
    }
  }
  else
  {
    while(READ_BIT(FLASH->SR2, FLASH_FLAG_CRC_BUSY_BANK2) == FLASH_FLAG_CRC_BUSY_BANK2)
    {
      LL_mDelay(1);

      if(tickstart-- == 0)
      {
        return -1;
      }
    }
  }

  /* Check FLASH CRC read error flag  */
  if(Bank == FLASH_BANK_1)
  {
    if(READ_BIT(FLASH->SR1, FLASH_FLAG_CRCRDERR_BANK1) == FLASH_FLAG_CRCRDERR_BANK1)
    {
      /* Clear FLASH CRC read error pending bit */
      WRITE_REG(FLASH->CCR1, FLASH_FLAG_CRCRDERR_BANK1);

      return -1;
    }
  }
  else
  {
    if(READ_BIT(FLASH->SR2, FLASH_FLAG_CRCRDERR_BANK2) == FLASH_FLAG_CRCRDERR_BANK2)
    {
      /* Clear FLASH CRC read error pending bit */
      WRITE_REG(FLASH->CCR2, FLASH_FLAG_CRCRDERR_BANK2);

      return -1;
    }
  }

  return 0;
}


int _FLASH_OB_WaitForLastOperation(uint32_t Timeout)
{
  /* Get timeout */
  uint32_t tickstart;

  /* Wait for the FLASH Option Bytes change operation to complete by polling on OPT_BUSY flag to be reset */
  tickstart = Timeout;
  while(READ_BIT(FLASH->OPTSR_CUR, FLASH_OPTSR_OPT_BUSY) != 0U)
  {
    LL_mDelay(1);

    if(tickstart-- == 0)
    {
      return -1;
    }
  }

  /* Check option byte change error */
  if(READ_BIT(FLASH->OPTSR_CUR, FLASH_OPTSR_OPTCHANGEERR) != 0U)
  {
    /* Clear the OB error flag */
    FLASH->OPTCCR |= FLASH_OPTCCR_CLR_OPTCHANGEERR;

    return -1;
  }

  /* If there is no error flag set */
  return 0;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  volatile uint32_t optr_reg_val = 0;
  volatile uint32_t optr_reg_mask = 0;
  volatile uint32_t UserConfig;

  __DSB();
  __ISB();
  __disable_irq();

  _FLASH_OB_Unlock();
  _FLASH_Unlock();

  UserConfig  = READ_REG(FLASH->OPTSR_CUR);

  if((UserConfig&FLASH_OPTSR_BCM7) == FLASH_OPTSR_BCM7)
  {
    UserConfig &= ~FLASH_OPTSR_BCM7;  //Disable M7 after system reset
  }
  else
  {
    UserConfig |= FLASH_OPTSR_BCM7;   //Enable M7 after system reset
  }

  if(_FLASH_WaitForLastOperation((uint32_t)50000, FLASH_BANK_1) != 0)
  {
    Error_Handler();
  }

  if(_FLASH_WaitForLastOperation((uint32_t)50000, FLASH_BANK_2) != 0)
  {
    Error_Handler();
  }

  optr_reg_val |= (UserConfig & FLASH_OPTSR_BCM7);
  optr_reg_mask |= FLASH_OPTSR_BCM7;

  MODIFY_REG(FLASH->OPTSR_PRG, optr_reg_mask, optr_reg_val);

  /* Wait for CRC computation to be completed */
  if (_FLASH_CRC_WaitForLastOperation((uint32_t)50000, FLASH_BANK_1) != 0)
  {
    Error_Handler();
  }
  else if (_FLASH_CRC_WaitForLastOperation((uint32_t)50000, FLASH_BANK_2) != 0)
  {
    Error_Handler();
  }
  else
  {
    /* Set OPTSTRT Bit */
    SET_BIT(FLASH->OPTCR, FLASH_OPTCR_OPTSTART);

    /* Wait for OB change operation to be completed */
    if(_FLASH_OB_WaitForLastOperation((uint32_t)50000) != 0)
    {
      Error_Handler();
    }
  }

  _FLASH_OB_Lock();
  _FLASH_Lock();

  __NVIC_SystemReset();

  #if 0
  OBInit.Banks = FLASH_BANK_1;
  HAL_FLASHEx_OBGetConfig(&OBInit);

  OBInit.USERType |= OB_USER_BCM7;

  if((OBInit.USERConfig&FLASH_OPTSR_BCM7) == FLASH_OPTSR_BCM7)
  {
    OBInit.USERConfig &= ~FLASH_OPTSR_BCM7;
  }
  else
  {
    OBInit.USERConfig |= FLASH_OPTSR_BCM7;
  }

  USERConfig = OBInit.USERConfig;

  if(HAL_FLASHEx_OBProgram(&OBInit) == HAL_ERROR)
  {
    _FLASH_OB_Lock();
    _FLASH_Lock();
    Error_Handler();
  }

  if(HAL_FLASH_OB_Launch() == HAL_ERROR)
  {
    _FLASH_OB_Lock();
    _FLASH_Lock();
    Error_Handler();
  }

  HAL_FLASHEx_OBGetConfig(&OBInit);
  if(OBInit.USERConfig != USERConfig)
  {
    _FLASH_OB_Lock();
    _FLASH_Lock();
    Error_Handler();
  }

  _FLASH_OB_Lock();
  _FLASH_Lock();
  __NVIC_SystemReset();
#endif
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
