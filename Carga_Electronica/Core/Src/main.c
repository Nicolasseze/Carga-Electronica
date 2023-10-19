/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "adc_function.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
enum modos {off, corriente_constante, tension_constante, potencia_constante, fusible_electronico};

/* Estructura estado modo de la carga electronica */
//valor para corriente_constante en mA
typedef struct {

	enum modos modo; 					/* Modo de trabajo de la carga electronica */
	float valor;						/* Set point trabajo de la carga electronica */

} CARGA_HandleTypeDef;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//Definiciones DAC - MCP4725
#define MCP4725_ADDR 	0b1100000 << 1
#define MASK_DAC_READ 	0b00001111

//CALIBRACIONES
#define CAL_DAC_VALUE 0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

osThreadId defaultTaskHandle;
osThreadId tarea_carga_conHandle;
osThreadId tarea_errorHandle;
/* USER CODE BEGIN PV */
/* Declaracion de colas */
//
osPoolId mpool1;
osMessageQId QueueCargaControlHandle;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
void StartDefaultTask(void const * argument);
void tarea_carga_control(void const * argument);
void tarea_error_handler(void const * argument);

/* USER CODE BEGIN PFP */
void DAC_init(void);
void DAC_set(float setPoint);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* definition and creation de Cola Control Carga Electronica */
  osPoolDef(mpool1, 2, CARGA_HandleTypeDef);
  mpool1 = osPoolCreate(osPool(mpool1));
  osMessageQDef(QueueCargaControlHandle, 2, CARGA_HandleTypeDef);
  QueueCargaControlHandle = osMessageCreate(osMessageQ(QueueCargaControlHandle), NULL);

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of tarea_carga_con */
  osThreadDef(tarea_carga_con, tarea_carga_control, osPriorityBelowNormal, 0, 128);
  tarea_carga_conHandle = osThreadCreate(osThread(tarea_carga_con), NULL);

  /* definition and creation of tarea_error */
  osThreadDef(tarea_error, tarea_error_handler, osPriorityAboveNormal, 0, 128);
  tarea_errorHandle = osThreadCreate(osThread(tarea_error), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ETH_RST_Pin|ETH_CS_Pin|I_SCALE_Pin|USER_LED_Pin
                          |V_SCALE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : TRIGGER_Pin CONX_Pin */
  GPIO_InitStruct.Pin = TRIGGER_Pin|CONX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : ERROR2_Pin */
  GPIO_InitStruct.Pin = ERROR2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ERROR2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ETH_RST_Pin ETH_CS_Pin I_SCALE_Pin USER_LED_Pin
                           V_SCALE_Pin */
  GPIO_InitStruct.Pin = ETH_RST_Pin|ETH_CS_Pin|I_SCALE_Pin|USER_LED_Pin
                          |V_SCALE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ERROR1_Pin */
  GPIO_InitStruct.Pin = ERROR1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ERROR1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : V_INV_Pin */
  GPIO_InitStruct.Pin = V_INV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(V_INV_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_KEY_Pin */
  GPIO_InitStruct.Pin = USER_KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_KEY_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void DAC_init(void){
	//Leer EEPROM y checkear que este en 0; Sino cambiarlo.
	uint16_t i2cRX;

	uint8_t buffer[5];
	memset(buffer,0,5);
	HAL_I2C_Master_Transmit(&hi2c1,MCP4725_ADDR, buffer, 2,HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(&hi2c1, MCP4725_ADDR, buffer, 5,HAL_MAX_DELAY);
	//Cargo el valor recibido del buffer a una variable para tener el valor de la eeprom.
	i2cRX = (buffer[3]<<8) || buffer[4];
	//Mascareo de rutix
	i2cRX &= MASK_DAC_READ;

	//Si el valor de la eeprom no es 0 lo cargo.
	if(i2cRX){
		buffer[0]=0b01100000;
		buffer[1]=0;
		buffer[2]=0;
		HAL_I2C_Master_Transmit(&hi2c1, MCP4725_ADDR, buffer, 3,HAL_MAX_DELAY);
		do{
			i2cRX = HAL_I2C_Master_Receive(&hi2c1, MCP4725_ADDR, buffer, 1,HAL_MAX_DELAY) && 0b10000000;
		}while(!i2cRX);
	}else{
		//Reset
		buffer[0]=0;
		buffer[1]=0;
		HAL_I2C_Master_Transmit(&hi2c1, MCP4725_ADDR, buffer, 2,HAL_MAX_DELAY);
	}
}
void DAC_set(float setPoint){	//setPoint = valor en mV.
	uint8_t buffer[2];

	if(setPoint<0 || setPoint>=5000)
		return HAL_I2C_Master_Transmit(&hi2c1, MCP4725_ADDR, 0, 2,HAL_MAX_DELAY);

	setPoint=((setPoint+CAL_DAC_VALUE)*4096)/5000;
	buffer[0]=((uint16_t) setPoint)>>8;
	buffer[1]=((uint16_t) setPoint);
	HAL_I2C_Master_Transmit(&hi2c1, MCP4725_ADDR, buffer, 2,HAL_MAX_DELAY);
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
	float var=0;
	DAC_init();
  /* Infinite loop */
  for(;;)
  {


	var=100;
	DAC_set(100);
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_tarea_carga_control */
/**
* @brief Function implementing the tarea_carga_con thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_tarea_carga_control */
void tarea_carga_control(void const * argument)
{
  /* USER CODE BEGIN tarea_carga_control */
	CARGA_HandleTypeDef *ptr;
	osEvent evt;

  float tension=0,corriente=0;
  float valorSet=0;

  while (1){
	  osDelay(1000);
  }
  /* Infinite loop */
  for(;;)
  {
	evt = osMessageGet(QueueCargaControlHandle, osWaitForever);
	if(evt.status == osEventMessage){
		ptr = evt.value.p;
		osPoolFree(mpool1, ptr);
	}
  //recibi modo y valor en *ptr desde martin-eth
  
  //leo ADC
  tension = ADC_read_tension(&hi2c1);
  corriente = ADC_read_corriente(&hi2c1);
  
  //En base al modo decido:
  switch (ptr->modo)
  {
    case corriente_constante:
      //No hago una chota porque es analogico
      DAC_set(ptr->valor/10); //Ajusto de mA -->mV para el DAC
      //EnviarDatosMartin();
      break;
    
    case tension_constante:
		//Necesito saber el valor actual para poder ajustar el dac que modifica la corriente y mantiene la tensión.
		if(tension > ptr->valor){
			valorSet++;
		}else {
			valorSet--;
		}
		DAC_set(valorSet);
	  break;

	case potencia_constante:
		if((tension*corriente) > ptr->valor){
			valorSet++;
			}else {
				valorSet--;
			}
		DAC_set(valorSet);
	  break;

	case fusible_electronico:
		if(corriente>ptr->valor){
			//rutina de CORTAR con error??? && ir a off?
			__asm("nop");
		}
	  break;
    case off:
      break;
    
    default:
      break;
  }
    
  }
  /* USER CODE END tarea_carga_control */
}

/* USER CODE BEGIN Header_tarea_error_handler */
/**
* @brief Function implementing the tarea_error thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_tarea_error_handler */
void tarea_error_handler(void const * argument)
{
  /* USER CODE BEGIN tarea_error_handler */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END tarea_error_handler */
}

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
