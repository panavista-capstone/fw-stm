/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

/* USER CODE BEGIN PV */
extern const char tps25750x_lowRegion_i2c_array[];
extern int gSizeLowRegionArray;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */
static int usbPdStateMachine(int* err);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile uint8_t buf[256] = {0};
volatile union {
  uint8_t raw[6];
  struct __packed {
    unsigned len : 8;
    unsigned PatchHeaderErr : 1;
    unsigned rsv0 : 1;
    unsigned DeadBatteryFlag : 1;
    unsigned I2cEepromPresent : 1;
    unsigned rsv1 : 6;
    unsigned patchdownloaderr : 1;
    unsigned rsv2 : 8;
    unsigned MasterTSD : 1;
    unsigned rsv3 : 9;
    unsigned PatchConfigSource : 3;
    unsigned rev_id : 8;
  };
} boot_status;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  uint32_t last_toggle = HAL_GetTick();
  int error_detected = 0;
  HAL_Delay(100);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // if (HAL_GetTick() > last_toggle + 1) {
    //   // HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_10 | GPIO_PIN_11);
    //   last_toggle = HAL_GetTick();
    // }
    // Flash USB-PD Chip with firmware
    while (!usbPdStateMachine(&error_detected)) {}

    memset(buf, 0, sizeof(buf));
    if (error_detected == 0) {
      if (HAL_I2C_Mem_Read(&hi2c1, 0b01000000, 0x2D, 1, boot_status.raw, 5+1, -1) != HAL_OK) {
      }
      // HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_10);
      HAL_Delay(100);
    }


    
  }
  /* USER CODE END 3 */
}

typedef enum {
  STATE_IDLE = 0,
  STATE_DELAY,
  STATE_READY_FOR_PATCH,
  STATE_READ_MODE,
  STATE_WRITE_DATA1,
  STATE_WRITE_PBMS,
  STATE_READ_PBMS,
  STATE_READ_DATA1,
  STATE_WRITE_FW,
  STATE_WRITE_PBMC,
  STATE_WRITE_PBME,
  STATE_READ_PBMC,
  STATE_READ_DATA1_PBMC,
  STATE_CHECK_PATCH_LOADED,
  STATE_CHECK_MODE,
  STATE_COMPLETE = 0xFE,
  STATE_ERROR
} usbPdState;

static int usbPdStateMachine(int* err) {
  static usbPdState state = STATE_IDLE;
  static usbPdState nextStateAfterDelay = STATE_IDLE;
  static uint32_t old_time = 0;
  int complete = 0;
  switch (state)
  {
  case STATE_IDLE:
    state = STATE_READY_FOR_PATCH;
    *err = 0;
    break;

  case STATE_DELAY:
    uint32_t time_now = HAL_GetTick();
    if (time_now >= old_time + 10) {
      state = nextStateAfterDelay;
    }
    *err = 0;
    break;
  
  case STATE_READY_FOR_PATCH:
    if (HAL_I2C_Mem_Read(&hi2c1, 0b01000000, 0x14, 1, buf, 11+1, -1) != HAL_OK) {
      state = STATE_ERROR;
      *err = 1;
      break;
    }

    if (buf[11] & 0x02) {
      state = STATE_READ_MODE;
    } else {
      state = STATE_DELAY;
      nextStateAfterDelay = STATE_READY_FOR_PATCH;
      old_time = HAL_GetTick();
    }
    break;

  case STATE_READ_MODE:
    if (HAL_I2C_Mem_Read(&hi2c1, 0b01000000, 0x03, 1, buf, 5, -1) != HAL_OK) {
      state = STATE_ERROR;
      *err = 1;
      break;
    }

    if (buf[0] == 0x04 && buf[1] == 'P' && buf[2] == 'T' && buf[3] == 'C' && buf[4] == 'H') {
      state = STATE_WRITE_DATA1;
    } else {
      state = STATE_DELAY;
      nextStateAfterDelay = STATE_READ_MODE;
      old_time = HAL_GetTick();
    }
    break;
  
  case STATE_WRITE_DATA1:
    memset(buf, 0, sizeof(buf));
    buf[0] = 6;
    *(uint32_t*)(&buf[1]) = gSizeLowRegionArray;
    buf[5] = 0b00001100;
    // buf[5] = 0b000000;
    buf[6] = 0x32;
    if (HAL_I2C_Mem_Write(&hi2c1, 0b01000000, 0x09, 1, buf, 6+1, -1) != HAL_OK) {
      state = STATE_ERROR;
      *err = 1;
      break;
    }
    
    state = STATE_WRITE_PBMS;
    *err = 0;
    break;

  case STATE_WRITE_PBMS:
    memset(buf, 0, sizeof(buf));
    buf[0] = 4;
    buf[1] = 'P';
    buf[2] = 'B';
    buf[3] = 'M';
    buf[4] = 's';
    if (HAL_I2C_Mem_Write(&hi2c1, 0b01000000, 0x08, 1, buf, 4+1, -1) != HAL_OK) {
      state = STATE_ERROR;
      *err = 1;
      break;
    }
    state = STATE_READ_PBMS;
    *err = 0;

    break;

  case STATE_READ_PBMS:
    memset(buf, 0, sizeof(buf));
    if (HAL_I2C_Mem_Read(&hi2c1, 0b01000000, 0x08, 1, buf, 4+1, -1) != HAL_OK) {
      state = STATE_ERROR;
      *err = 1;
      break;
    }

    if (buf[0] == 0x04 && buf[1] == 0x0 && buf[2] == 0x0 && buf[3] == 0x0 && buf[4] == 0x0) {
      state = STATE_READ_DATA1;
    } else {
      state = STATE_DELAY;
      nextStateAfterDelay = STATE_READ_PBMS;
      old_time = HAL_GetTick();
    }
    break;

  case STATE_READ_DATA1:
    memset(buf, 0, sizeof(buf));
    if (HAL_I2C_Mem_Read(&hi2c1, 0b01000000, 0x09, 1, buf, 64+1, -1) != HAL_OK) {
      state = STATE_ERROR;
      *err = 1;
      break;
    }

    if (buf[0] == 0x40 && buf[1] == 0x0) {
      state = STATE_WRITE_FW;
    } else {
      state = STATE_IDLE;
    }
    break;

  case STATE_WRITE_FW:
    int ret;
    HAL_Delay(10);
    if (ret = HAL_I2C_Master_Transmit(&hi2c1, 0b00011000, tps25750x_lowRegion_i2c_array, gSizeLowRegionArray, -1)) {
      state = STATE_WRITE_PBME;
      *err = 1;
      break;
    }
    state = STATE_WRITE_PBMC;
    *err = 0;

    break;

  case STATE_WRITE_PBME:
    memset(buf, 0, sizeof(buf));
    HAL_Delay(1);
    buf[0] = 4;
    buf[1] = 'P';
    buf[2] = 'B';
    buf[3] = 'M';
    buf[4] = 'e';
    if (HAL_I2C_Mem_Write(&hi2c1, 0b01000000, 0x08, 1, buf, 4+1, -1) != HAL_OK) {
      state = STATE_ERROR;
      *err = 1;
      break;
    }
    state = STATE_ERROR;
    *err = 1;

    break;
  
  case STATE_WRITE_PBMC:
    memset(buf, 0, sizeof(buf));
    HAL_Delay(1);
    buf[0] = 4;
    buf[1] = 'P';
    buf[2] = 'B';
    buf[3] = 'M';
    buf[4] = 'c';
    if (HAL_I2C_Mem_Write(&hi2c1, 0b01000000, 0x08, 1, buf, 4+1, -1) != HAL_OK) {
      state = STATE_ERROR;
      *err = 1;
      break;
    }
    state = STATE_READ_PBMC;
    *err = 0;

    break;

  case STATE_READ_PBMC:
    memset(buf, 0, sizeof(buf));
    if (HAL_I2C_Mem_Read(&hi2c1, 0b01000000, 0x08, 1, buf, 4+1, -1) != HAL_OK) {
      state = STATE_ERROR;
      *err = 1;
      break;
    }

    if (buf[0] == 0x04 && buf[1] == 0x0 && buf[2] == 0x0 && buf[3] == 0x0 && buf[4] == 0x0) {
      state = STATE_READ_DATA1_PBMC;
    } else {
      state = STATE_DELAY;
      nextStateAfterDelay = STATE_READ_PBMC;
      old_time = HAL_GetTick();
    }
    break;

  case STATE_READ_DATA1_PBMC:
    memset(buf, 0, sizeof(buf));
    if (HAL_I2C_Mem_Read(&hi2c1, 0b01000000, 0x09, 1, buf, 64+1, -1) != HAL_OK) {
      state = STATE_ERROR;
      *err = 1;
      break;
    }

    if (buf[0] == 64 && buf[1] == 0x0 && buf[2] == 0x0 && buf[3] == 0x0 && buf[4] == 0x0) {
      state = STATE_CHECK_PATCH_LOADED;
    } else {
      state = STATE_ERROR;
    }
    break;

  case STATE_CHECK_PATCH_LOADED:
    memset(buf, 0, sizeof(buf));
    if (HAL_I2C_Mem_Read(&hi2c1, 0b01000000, 0x14, 1, buf, 11+1, -1) != HAL_OK) {
      state = STATE_ERROR;
      *err = 1;
      break;
    }

    if (buf[11] & 0x01) {
      state = STATE_CHECK_MODE;
    } else {
      state = STATE_DELAY;
      nextStateAfterDelay = STATE_CHECK_PATCH_LOADED;
      old_time = HAL_GetTick();
    }
    break;

  case STATE_CHECK_MODE:
    if (HAL_I2C_Mem_Read(&hi2c1, 0b01000000, 0x03, 1, buf, 5, -1) != HAL_OK) {
      state = STATE_ERROR;
      *err = 1;
      break;
    }

    if (buf[0] == 0x04 && buf[1] == 'A' && buf[2] == 'P' && buf[3] == 'P' && buf[4] == ' ') {
      state = STATE_COMPLETE;
    } else {
      state = STATE_DELAY;
      nextStateAfterDelay = STATE_CHECK_MODE;
      old_time = HAL_GetTick();
    }
    break;

  case STATE_COMPLETE:
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
    *err = 0;
    complete = 1;
    break;

  case STATE_ERROR:
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
    *err = 1;
    complete = 1;
    // HAL_I2C_Master_Transmit()

    break;
  
  default:
    break;
  }

  return complete;
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA10 PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
