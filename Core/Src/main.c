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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
DSI_HandleTypeDef hdsi;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

LTDC_HandleTypeDef hltdc;

OSPI_HandleTypeDef hospi2;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */
#define HARDWARE_VER "1.0.0"
#define FLASH_CLK_PIN       GPIO_PIN_10
#define FLASH_CLK_PORT      GPIOF
#define FLASH_CS_PIN        GPIO_PIN_11
#define FLASH_CS_PORT       GPIOC
#define FLASH_MISO_PIN      GPIO_PIN_9
#define FLASH_MISO_PORT     GPIOF
#define FLASH_MOSI_PIN      GPIO_PIN_8
#define FLASH_MOSI_PORT     GPIOF

uint8_t BOARD_NUMBER = 2;
uint8_t spi3_buf[2] = {0};
uint32_t frame_rate=1000; //delay time (ms)
uint8_t frame_buf_tmp[6400]={0};
uint8_t frame_buf_0[6400]={0};
uint8_t frame_buf_1[6400]={0};
uint8_t frame_buf_flash[6400]={0};
uint8_t play_mode=0;  //0=Static display, 1=Dynamic display
uint8_t play_mode_source=0;  //0=flash, 1=frame_buf_0, 2=frame_buf_1
uint8_t static_flag=0;  //0=buffer_0, 1=buffer_1
uint8_t display_image_number=0;
uint8_t total_image_in_flash=10;
uint8_t image_80x80_rgb888[19200] =	{[0 ... 19199] = 0xFF};


#define BUFFER_SIZE  8192
uint8_t aRxBuffer[BUFFER_SIZE] = { 0 };

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_OCTOSPI2_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_UART4_Init(void);
static void MX_DSIHOST_DSI_Init(void);
static void MX_LTDC_Init(void);
/* USER CODE BEGIN PFP */
static void mipi_config(void);
static void SPI_master2slave(char * buf, char * frame_buf, uint8_t image_flag);
static void Write_Registers_data(uint8_t do_flag);
static void I2C_Control_Voltage(void);
static void LCD_PowerOn(void);
void delay_us(int time);
void delay_100ns(int time);
void FLASH_WriteByte(uint8_t data);
uint8_t FLASH_ReadByte(void);
void write_flash_page(uint8_t *data, uint8_t image_id);
void read_flash_page(uint8_t *data, uint8_t image_id);
void erase_flash_sector(uint8_t image_id);
void reset_flash_software();
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
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_OCTOSPI2_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_UART4_Init();
  MX_DSIHOST_DSI_Init();
  MX_LTDC_Init();
  /* USER CODE BEGIN 2 */
  mipi_config();
  HAL_UART_Transmit(&huart4, "slave start", 11, 1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int frame_buf_count = 0;

	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_SET && HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4) == GPIO_PIN_SET) //switch = 00
	{
		BOARD_NUMBER = 2;
	}
	else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_SET && HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4) == GPIO_PIN_RESET) //switch = 01
	{
		BOARD_NUMBER = 2;
	}
	else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_RESET && HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4) == GPIO_PIN_SET) //switch = 10
	{
		BOARD_NUMBER = 3;
	}
	else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_RESET && HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4) == GPIO_PIN_RESET) //switch = 11
	{
		BOARD_NUMBER = 4;
	}

    __HAL_SPI_ENABLE(&hspi1);
    __HAL_SPI_ENABLE(&hspi3);
	HAL_SPI_Receive_IT(&hspi3, &spi3_buf, 2);

	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if(play_mode_source == 0)
		{
			if(play_mode == 0)
			{
				read_flash_page(&frame_buf_flash, display_image_number);
				frame_buf_count = 0;
				for (int i = 0; i < 6400;  i += 4)
				{
					image_80x80_rgb888[frame_buf_count] = frame_buf_flash[i+1];
					frame_buf_count += 2;
					image_80x80_rgb888[frame_buf_count] = frame_buf_flash[i];
					frame_buf_count += 4;
					image_80x80_rgb888[frame_buf_count] = frame_buf_flash[i+2];
					frame_buf_count += 4;
					image_80x80_rgb888[frame_buf_count] = frame_buf_flash[i+3];
					frame_buf_count += 2;
				}
			}
			else if(play_mode == 1)
			{
				for(int i=0; i<total_image_in_flash; i++)
				{
					if(play_mode_source != 0) break;
					read_flash_page(&frame_buf_flash, i);
					frame_buf_count = 0;
					for (int i = 0; i < 6400;  i += 4)
					{
						image_80x80_rgb888[frame_buf_count] = frame_buf_flash[i+1];
						frame_buf_count += 2;
						image_80x80_rgb888[frame_buf_count] = frame_buf_flash[i];
						frame_buf_count += 4;
						image_80x80_rgb888[frame_buf_count] = frame_buf_flash[i+2];
						frame_buf_count += 4;
						image_80x80_rgb888[frame_buf_count] = frame_buf_flash[i+3];
						frame_buf_count += 2;
					}
					HAL_Delay(frame_rate);
				}
			}
		}
		else if(play_mode_source == 1)
		{
			frame_buf_count = 0;
			for (int i = 0; i < 6400;  i += 4)
			{
				image_80x80_rgb888[frame_buf_count] = frame_buf_0[i+1];
				frame_buf_count += 2;
				image_80x80_rgb888[frame_buf_count] = frame_buf_0[i];
				frame_buf_count += 4;
				image_80x80_rgb888[frame_buf_count] = frame_buf_0[i+2];
				frame_buf_count += 4;
				image_80x80_rgb888[frame_buf_count] = frame_buf_0[i+3];
				frame_buf_count += 2;
			}
		}
		else if(play_mode_source == 2)
		{
			frame_buf_count = 0;
			for (int i = 0; i < 6400;  i += 4)
			{
				image_80x80_rgb888[frame_buf_count] = frame_buf_1[i+1];
				frame_buf_count += 2;
				image_80x80_rgb888[frame_buf_count] = frame_buf_1[i];
				frame_buf_count += 4;
				image_80x80_rgb888[frame_buf_count] = frame_buf_1[i+2];
				frame_buf_count += 4;
				image_80x80_rgb888[frame_buf_count] = frame_buf_1[i+3];
				frame_buf_count += 2;
			}
		}
		HAL_Delay(10);

		/*I2C_Control_Voltage();
		while (1);*/
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 30;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DSIHOST Initialization Function
  * @param None
  * @retval None
  */
static void MX_DSIHOST_DSI_Init(void)
{

  /* USER CODE BEGIN DSIHOST_Init 0 */

  /* USER CODE END DSIHOST_Init 0 */

  DSI_PLLInitTypeDef PLLInit = {0};
  DSI_HOST_TimeoutTypeDef HostTimeouts = {0};
  DSI_PHY_TimerTypeDef PhyTimings = {0};
  DSI_VidCfgTypeDef VidCfg = {0};

  /* USER CODE BEGIN DSIHOST_Init 1 */

  /* USER CODE END DSIHOST_Init 1 */
  hdsi.Instance = DSI;
  hdsi.Init.AutomaticClockLaneControl = DSI_AUTO_CLK_LANE_CTRL_DISABLE;
  hdsi.Init.TXEscapeCkdiv = 2;
  hdsi.Init.NumberOfLanes = DSI_ONE_DATA_LANE;
  PLLInit.PLLNDIV = 50;
  PLLInit.PLLIDF = DSI_PLL_IN_DIV1;
  PLLInit.PLLODF = DSI_PLL_OUT_DIV2;
  if (HAL_DSI_Init(&hdsi, &PLLInit) != HAL_OK)
  {
    Error_Handler();
  }
  HostTimeouts.TimeoutCkdiv = 1;
  HostTimeouts.HighSpeedTransmissionTimeout = 0;
  HostTimeouts.LowPowerReceptionTimeout = 0;
  HostTimeouts.HighSpeedReadTimeout = 0;
  HostTimeouts.LowPowerReadTimeout = 0;
  HostTimeouts.HighSpeedWriteTimeout = 0;
  HostTimeouts.HighSpeedWritePrespMode = DSI_HS_PM_DISABLE;
  HostTimeouts.LowPowerWriteTimeout = 0;
  HostTimeouts.BTATimeout = 0;
  if (HAL_DSI_ConfigHostTimeouts(&hdsi, &HostTimeouts) != HAL_OK)
  {
    Error_Handler();
  }
  PhyTimings.ClockLaneHS2LPTime = 19;
  PhyTimings.ClockLaneLP2HSTime = 15;
  PhyTimings.DataLaneHS2LPTime = 9;
  PhyTimings.DataLaneLP2HSTime = 10;
  PhyTimings.DataLaneMaxReadTime = 0;
  PhyTimings.StopWaitTime = 0;
  if (HAL_DSI_ConfigPhyTimer(&hdsi, &PhyTimings) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DSI_SetLowPowerRXFilter(&hdsi, 10000) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DSI_ConfigErrorMonitor(&hdsi, HAL_DSI_ERROR_NONE) != HAL_OK)
  {
    Error_Handler();
  }
  VidCfg.VirtualChannelID = 0;
  VidCfg.ColorCoding = DSI_RGB888;
  VidCfg.LooselyPacked = DSI_LOOSELY_PACKED_DISABLE;
  VidCfg.Mode = DSI_VID_MODE_NB_EVENTS;
  VidCfg.PacketSize = 80;
  VidCfg.NumberOfChunks = 1;
  VidCfg.NullPacketSize = 0;
  VidCfg.HSPolarity = DSI_HSYNC_ACTIVE_HIGH;
  VidCfg.VSPolarity = DSI_VSYNC_ACTIVE_HIGH;
  VidCfg.DEPolarity = DSI_DATA_ENABLE_ACTIVE_HIGH;
  VidCfg.HorizontalSyncActive = 252;
  VidCfg.HorizontalBackPorch = 252;
  VidCfg.HorizontalLine = 836;
  VidCfg.VerticalSyncActive = 8;
  VidCfg.VerticalBackPorch = 8;
  VidCfg.VerticalFrontPorch = 8;
  VidCfg.VerticalActive = 80;
  VidCfg.LPCommandEnable = DSI_LP_COMMAND_ENABLE;
  VidCfg.LPLargestPacketSize = 28;
  VidCfg.LPVACTLargestPacketSize = 80;
  VidCfg.LPHorizontalFrontPorchEnable = DSI_LP_HFP_ENABLE;
  VidCfg.LPHorizontalBackPorchEnable = DSI_LP_HBP_ENABLE;
  VidCfg.LPVerticalActiveEnable = DSI_LP_VACT_ENABLE;
  VidCfg.LPVerticalFrontPorchEnable = DSI_LP_VFP_ENABLE;
  VidCfg.LPVerticalBackPorchEnable = DSI_LP_VBP_ENABLE;
  VidCfg.LPVerticalSyncActiveEnable = DSI_LP_VSYNC_ENABLE;
  VidCfg.FrameBTAAcknowledgeEnable = DSI_FBTAA_DISABLE;
  if (HAL_DSI_ConfigVideoMode(&hdsi, &VidCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DSI_SetGenericVCID(&hdsi, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DSIHOST_Init 2 */
	LCD_PowerOn();
  /* USER CODE END DSIHOST_Init 2 */

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
  hi2c1.Init.Timing = 0x307075B1;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
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
  hi2c2.Init.Timing = 0x307075B1;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief LTDC Initialization Function
  * @param None
  * @retval None
  */
static void MX_LTDC_Init(void)
{

  /* USER CODE BEGIN LTDC_Init 0 */

  /* USER CODE END LTDC_Init 0 */

  LTDC_LayerCfgTypeDef pLayerCfg = {0};

  /* USER CODE BEGIN LTDC_Init 1 */

  /* USER CODE END LTDC_Init 1 */
  hltdc.Instance = LTDC;
  hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AH;
  hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AH;
  hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
  hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
  hltdc.Init.HorizontalSync = 251;
  hltdc.Init.VerticalSync = 7;
  hltdc.Init.AccumulatedHBP = 503;
  hltdc.Init.AccumulatedVBP = 15;
  hltdc.Init.AccumulatedActiveW = 583;
  hltdc.Init.AccumulatedActiveH = 95;
  hltdc.Init.TotalWidth = 835;
  hltdc.Init.TotalHeigh = 97;
  hltdc.Init.Backcolor.Blue = 255;
  hltdc.Init.Backcolor.Green = 255;
  hltdc.Init.Backcolor.Red = 255;
  if (HAL_LTDC_Init(&hltdc) != HAL_OK)
  {
    Error_Handler();
  }
  pLayerCfg.WindowX0 = 0;
  pLayerCfg.WindowX1 = 80;
  pLayerCfg.WindowY0 = 0;
  pLayerCfg.WindowY1 = 80;
  pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_RGB888;
  pLayerCfg.Alpha = 255;
  pLayerCfg.Alpha0 = 0;
  pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
  pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
  pLayerCfg.FBStartAdress = (uint32_t *)image_80x80_rgb888;
  pLayerCfg.ImageWidth = 80;
  pLayerCfg.ImageHeight = 80;
  pLayerCfg.Backcolor.Blue = 255;
  pLayerCfg.Backcolor.Green = 255;
  pLayerCfg.Backcolor.Red = 255;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LTDC_Init 2 */
  //(uint32_t *)image_80x80_rgb888;
  /* USER CODE END LTDC_Init 2 */

}

/**
  * @brief OCTOSPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OCTOSPI2_Init(void)
{

  /* USER CODE BEGIN OCTOSPI2_Init 0 */

  /* USER CODE END OCTOSPI2_Init 0 */

  OSPIM_CfgTypeDef OSPIM_Cfg_Struct = {0};

  /* USER CODE BEGIN OCTOSPI2_Init 1 */

  /* USER CODE END OCTOSPI2_Init 1 */
  /* OCTOSPI2 parameter configuration*/
  hospi2.Instance = OCTOSPI2;
  hospi2.Init.FifoThreshold = 1;
  hospi2.Init.DualQuad = HAL_OSPI_DUALQUAD_DISABLE;
  hospi2.Init.MemoryType = HAL_OSPI_MEMTYPE_MICRON;
  hospi2.Init.DeviceSize = 32;
  hospi2.Init.ChipSelectHighTime = 1;
  hospi2.Init.FreeRunningClock = HAL_OSPI_FREERUNCLK_DISABLE;
  hospi2.Init.ClockMode = HAL_OSPI_CLOCK_MODE_0;
  hospi2.Init.ClockPrescaler = 1;
  hospi2.Init.SampleShifting = HAL_OSPI_SAMPLE_SHIFTING_NONE;
  hospi2.Init.DelayHoldQuarterCycle = HAL_OSPI_DHQC_DISABLE;
  hospi2.Init.ChipSelectBoundary = 0;
  hospi2.Init.DelayBlockBypass = HAL_OSPI_DELAY_BLOCK_BYPASSED;
  if (HAL_OSPI_Init(&hospi2) != HAL_OK)
  {
    Error_Handler();
  }
  OSPIM_Cfg_Struct.ClkPort = 2;
  OSPIM_Cfg_Struct.NCSPort = 2;
  OSPIM_Cfg_Struct.IOLowPort = HAL_OSPIM_IOPORT_2_LOW;
  if (HAL_OSPIM_Config(&hospi2, &OSPIM_Cfg_Struct, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OCTOSPI2_Init 2 */

  /* USER CODE END OCTOSPI2_Init 2 */

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
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
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
  hspi2.Init.Mode = SPI_MODE_SLAVE;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_SLAVE;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(flash_cs_GPIO_Port, flash_cs_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, flash_mosi_Pin|flash_clk_Pin|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_SET);

  /*Configure GPIO pin : flash_cs_Pin */
  GPIO_InitStruct.Pin = flash_cs_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(flash_cs_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : flash_mosi_Pin flash_clk_Pin */
  GPIO_InitStruct.Pin = flash_mosi_Pin|flash_clk_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : flash_miso_Pin */
  GPIO_InitStruct.Pin = flash_miso_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(flash_miso_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PD11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PE10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PF14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : PE12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PE14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void mipi_config() {
	if (HAL_DSI_Start(&hdsi) != HAL_OK) {
		Error_Handler();
	}
	HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0xF0, 0xC3);
	HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0xF0, 0x96);
	uint8_t cmd3[7] = { 0x00, 0x77, 0x1F, 0x04, 0x2A, 0x80, 0x33 };
	HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, 8, 0xE7, cmd3);
	uint8_t cmd4[3] = { 0xC0, 0x68, 0xE0 };
	HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, 4, 0xA4, cmd4);
	uint8_t cmd5[4] = { 0x42, 0x05, 0x24, 0x03 };
	HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, 5, 0xC3, cmd5);
	uint8_t cmd6[4] = { 0x42, 0x05, 0x24, 0x03 };
	HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, 5, 0xC4, cmd6);
	uint8_t cmd7[12] = { 0x0F, 0xF5, 0x10, 0x13, 0x22, 0x25, 0x10, 0x55, 0x55,
			0x55, 0x55, 0x55 };
	HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, 13, 0xE5, cmd7);
	uint8_t cmd8[12] = { 0x0F, 0xF5, 0x10, 0x13, 0x22, 0x25, 0x10, 0x55, 0x55,
			0x55, 0x55, 0x55 };
	HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, 13, 0xE6, cmd8);
	uint8_t cmd9[7] = { 0x00, 0x55, 0x00, 0x00, 0x00, 0x49, 0x22 };
	HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, 8, 0xEC, cmd9);
	uint8_t cmd10[4] = { 0x88, 0x05, 0x0F, 0x18 };
	HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, 5, 0xC1, cmd10);
	uint8_t cmd11[4] = { 0x88, 0x05, 0x0F, 0x18 };
	HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, 5, 0xC2, cmd11);
	HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x36, 0x00);
	HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x3A, 0x07);
	HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0xC5, 0xBE);
	uint8_t cmd15[14] = { 0xC0, 0x01, 0x04, 0x0B, 0x0B, 0x29, 0x41, 0x55, 0x55,
			0x3D, 0x19, 0x18, 0x24, 0x27 };
	HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, 15, 0xE0, cmd15);
	uint8_t cmd16[14] = { 0xC0, 0x01, 0x05, 0x0B, 0x0C, 0x29, 0x42, 0x55, 0x56,
			0x3E, 0x1A, 0x18, 0x24, 0x28 };
	HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, 15, 0xE1, cmd16);
	HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0xB2, 0x10);
	HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0xB3, 0x01);
	HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0xB4, 0x01);
	uint8_t cmd20[2] = { 0x27, 0x09 };
	HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, 3, 0xB6, cmd20);
	uint8_t cmd21[4] = { 0x00, 0x54, 0x00, 0x54 };
	HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, 5, 0xB5, cmd21);
	uint8_t cmd22[9] = { 0x20, 0x12, 0x40, 0x00, 0x00, 0x2F, 0x2A, 0x0A, 0x00 };
	HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, 10, 0xA5, cmd22);
	uint8_t cmd23[9] = { 0x20, 0x12, 0x40, 0x00, 0x00, 0x2F, 0x2A, 0x0A, 0x00 };
	HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, 10, 0xA6, cmd23);
	uint8_t cmd24[7] = { 0x58, 0x0A, 0x21, 0x00, 0x20, 0x01, 0x00 };
	HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, 8, 0xBA, cmd24);
	uint8_t cmd25[8] = { 0x00, 0x45, 0x00, 0x1F, 0x15, 0x87, 0x07, 0x04 };
	HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, 9, 0xBB, cmd25);
	uint8_t cmd26[8] = { 0x00, 0x45, 0x00, 0x1F, 0x15, 0x87, 0x07, 0x04 };
	HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, 9, 0xBC, cmd26);
	uint8_t cmd27[11] = { 0x11, 0x77, 0xFF, 0xFF, 0x25, 0x34, 0x43, 0x52, 0xFF,
			0xFF, 0xF9 };
	HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, 12, 0xBD, cmd27);
	HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0xED, 0xC3);
	uint8_t cmd29[3] = { 0x40, 0x0F, 0x00 };
	HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, 4, 0xE4, cmd29);
	uint8_t cmd30[9] = { 0x90, 0x00, 0x3F, 0x10, 0x3F, 0x35, 0x7F, 0x7F, 0x25 };
	HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, 10, 0xCC, cmd30);
	HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x35, 0x00);
	HAL_Delay(0);
	HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P0, 0x11, 0x00);
	HAL_Delay(120);
	HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P0, 0x29, 0x00);
	HAL_Delay(120);
	HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x35, 0x00);
}

void SPI_master2slave(char * buf, char * frame_buf, uint8_t image_flag) {
	HAL_SPI_Transmit(&hspi1, &buf[0], 1, 1000);
	HAL_SPI_Transmit(&hspi1, &buf[1], 1, 1000);
	if(image_flag == 1)
	{
		for(int i=0; i< 6400; i++)
		{
			HAL_SPI_Transmit(&hspi1, &frame_buf[i], 1, 1000);
		}
	}
}

void Write_Registers_data(uint8_t do_flag) {
	uint8_t Register_Address[1] = { 0 };
	uint8_t data[1] = { 0 };

	HAL_SPI_Receive(&hspi3, (uint8_t *)Register_Address, 1, 1000);
	HAL_SPI_Receive(&hspi3, (uint8_t *)data, 1, 1000);
	HAL_SPI_Transmit(&hspi1, &Register_Address[0], 1, 1000);
	HAL_SPI_Transmit(&hspi1, &data[0], 1, 1000);
	if(do_flag == 1)
	{
		switch (Register_Address[0]) {
		case 0: //Horizontal Resolution
			break;
		case 1: //Vertical Resolution
			break;
		case 2: //Horizontal blanking (High byte)
			break;
		case 3: //Horizontal blanking (Low byte)
			break;
		case 4: //Vertical blanking (High byte)
			break;
		case 5: //Vertical blanking (Low byte)
			break;
		case 6: //Frame rate x 2 (Hz)
			frame_rate=(uint32_t)data[0]*1000;
			break;
		case 7: //Show SPI flash content length
			break;
		case 8: //Content number of each frame
			break;
		case 9: //Clock rate of SPI
			break;
		case 10: //Clock rate of I2C
			break;
		case 11: //Pixel Mapping  one
			break;
		case 12: //Pixel Mapping one
			break;
		case 13: //Years of Version
			break;
		case 14: //Day of Version
			break;
		case 15: //Month of Version
			break;
		case 16: //Control A
			switch (data[0] & 0b00000011) {
			case 0b00000000:  //Display content of frame buffer (0)
				play_mode_source = 1;
				break;
			case 0b00000001:  //Display content of frame buffer (1)
				play_mode_source = 2;
				break;
			case 0b00000010:  //Display SPI input content
				break;
			case 0b00000011:  //Display Flash content
				play_mode_source = 0;
				break;
			}
			break;
		case 19: //Status
			break;
		}
	}
}

static void I2C_Control_Voltage()
{
	uint8_t data[1] = { 0 };
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	HAL_Delay(100);
	HAL_UART_Transmit(&huart4, "a", 1, 1000);

	data[0] = 0x40;
	if(HAL_I2C_Mem_Write(&hi2c1, (0x74 & 0x7f) << 1, 0x00, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000) == HAL_OK) {
		data[0] = 0x00;
		if(HAL_I2C_Mem_Write(&hi2c1, (0x74 & 0x7f) << 1, 0x01, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000) == HAL_OK) {
			data[0] = 0xA0;
			if(HAL_I2C_Mem_Write(&hi2c1, (0x74 & 0x7f) << 1, 0x06, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000) == HAL_OK) {
			}
		}
	}
	HAL_Delay(200);
	if(HAL_I2C_Mem_Read(&hi2c1, (0x74 & 0x7f) << 1, 0x00, I2C_MEMADD_SIZE_8BIT, &aRxBuffer, 1, 1000) == HAL_OK)
	{
		HAL_UART_Transmit(&huart4, &aRxBuffer, 1, 1000);
	}
	HAL_Delay(200);
	if(HAL_I2C_Mem_Read(&hi2c1, (0x74 & 0x7f) << 1, 0x06, I2C_MEMADD_SIZE_8BIT, &aRxBuffer, 1, 1000) == HAL_OK)
	{
		HAL_UART_Transmit(&huart4, &aRxBuffer, 1, 1000);
	}
	HAL_Delay(200);
	if(HAL_I2C_Mem_Read(&hi2c1, (0x74 & 0x7f) << 1, 0x07, I2C_MEMADD_SIZE_8BIT, &aRxBuffer, 1, 1000) == HAL_OK)
	{
		HAL_UART_Transmit(&huart4, &aRxBuffer, 1, 1000);
	}
	//v1.8
	/*if(HAL_I2C_Mem_Write(&hi2c1, (0x74 & 0x7f) << 1, 0x00, I2C_MEMADD_SIZE_8BIT, 0x64, 1, 1000) == HAL_OK) {
		if(HAL_I2C_Mem_Write(&hi2c1, (0x74 & 0x7f) << 1, 0x06, I2C_MEMADD_SIZE_8BIT, 0xA0, 1, 1000) == HAL_OK) {
			HAL_UART_Transmit(&huart4, "v1.8", 4, 1000);
		}
	}*/

	//v2.5
	/*if(HAL_I2C_Mem_Write(&hi2c1, (0x74 & 0x7f) << 1, 0x00, I2C_MEMADD_SIZE_8BIT, 0xAA, 1, 1000) == HAL_OK) {
		if(HAL_I2C_Mem_Write(&hi2c1, (0x74 & 0x7f) << 1, 0x06, I2C_MEMADD_SIZE_8BIT, 0xA0, 1, 1000) == HAL_OK) {
			HAL_UART_Transmit(&huart4, "v2.5", 4, 1000);
		}
	}}*/

	//v3.3
	/*if(HAL_I2C_Mem_Write(&hi2c1, (0x74 & 0x7f) << 1, 0x00, I2C_MEMADD_SIZE_8BIT, 0xFA, 1, 1000) == HAL_OK) {
		if(HAL_I2C_Mem_Write(&hi2c1, (0x74 & 0x7f) << 1, 0x06, I2C_MEMADD_SIZE_8BIT, 0xA0, 1, 1000) == HAL_OK) {
			HAL_UART_Transmit(&huart4, "v3.3", 4, 1000);
		}
	}*/
}

static void LCD_PowerOn(void)
{
	/* Activate XRES active low */
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET);

	HAL_Delay(20); /* wait 20 ms */

	/* Desactivate XRES */
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_SET);

	/* Wait for 10ms after releasing XRES before sending commands */
	HAL_Delay(120);
}

void delay_us(int time)
{
	int i = 0;
	while (time--)
	{
		i = 13;
		while (i--)
			;
	}
}

void delay_100ns(int time)
{
	int i = 0;
	while (time--)
	{
		i = 1;
		while (i--)
			;
	}
}

int button_count=0;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin==GPIO_PIN_12)
	{
		button_count++;
		delay_us(50000);
		for(int i=0; i<200; i++)
		{
			if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_12) == GPIO_PIN_RESET)
			{
				if(button_count<50 && play_mode==0)
				{
					display_image_number++;
					if(display_image_number >= total_image_in_flash)
						display_image_number = 0;
				}
				button_count = 0;
				return;
			}
			button_count++;
			delay_us(10000);
		}
		if(play_mode==1)
		{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
			play_mode = 0;
		}
		else if(play_mode==0)
		{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
			play_mode = 1;
		}
		button_count = 0;
	}
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef * hspi)
{
	// USB command: Type and command
	switch (spi3_buf[0] & 0b11000000) {
	case 0b00000000:  //Chain SPI functions
		switch (spi3_buf[0] & 0b00111000) {
		case 0b00000000: //Write content of full frame buffer to fram buffer (0)
			switch (spi3_buf[0] & 0b00000111) {
			case 0b00000000: //Command for DIP switch ID = 00
				break;
			case 0b00000001: //Command for DIP switch ID = 01
				if(BOARD_NUMBER==2)
				{
					HAL_SPI_Receive(&hspi3, (uint8_t*) frame_buf_0, 6400, 1000);
					SPI_master2slave(&spi3_buf, &frame_buf_0, 1);
				}
				else
				{
					HAL_SPI_Receive(&hspi3, (uint8_t*) frame_buf_tmp, 6400, 1000);
					SPI_master2slave(&spi3_buf, &frame_buf_tmp, 1);
				}
				break;
			case 0b00000010: //Command for DIP switch ID = 10
				if(BOARD_NUMBER==3)
				{
					HAL_SPI_Receive(&hspi3, (uint8_t*) frame_buf_0, 6400, 1000);
					SPI_master2slave(&spi3_buf, &frame_buf_0, 1);
				}
				else
				{
					HAL_SPI_Receive(&hspi3, (uint8_t*) frame_buf_tmp, 6400, 1000);
					SPI_master2slave(&spi3_buf, &frame_buf_tmp, 1);
				}
				break;
			case 0b00000011: //Command for DIP switch ID = 11
				if(BOARD_NUMBER==4)
				{
					HAL_SPI_Receive(&hspi3, (uint8_t*) frame_buf_0, 6400, 1000);
					SPI_master2slave(&spi3_buf, &frame_buf_0, 1);
				}
				else
				{
					HAL_SPI_Receive(&hspi3, (uint8_t*) frame_buf_tmp, 6400, 1000);
					SPI_master2slave(&spi3_buf, &frame_buf_tmp, 1);
				}
				break;
			default: //Broadcast to every board
				HAL_SPI_Receive(&hspi3, (uint8_t*) frame_buf_0, 6400, 1000);
				SPI_master2slave(&spi3_buf, &frame_buf_0, 1);
				break;
			}
			break;
		case 0b00001000: //Write content of full frame buffer to fram buffer (1)
			switch (spi3_buf[0] & 0b00000111) {
			case 0b00000000: //Command for DIP switch ID = 00
				break;
			case 0b00000001: //Command for DIP switch ID = 01
				if(BOARD_NUMBER==2)
				{
					HAL_SPI_Receive(&hspi3, (uint8_t*) frame_buf_1, 6400, 1000);
					SPI_master2slave(&spi3_buf, &frame_buf_1, 1);
				}
				else
				{
					HAL_SPI_Receive(&hspi3, (uint8_t*) frame_buf_tmp, 6400, 1000);
					SPI_master2slave(&spi3_buf, &frame_buf_tmp, 1);
				}
				break;
			case 0b00000010: //Command for DIP switch ID = 10
				if(BOARD_NUMBER==3)
				{
					HAL_SPI_Receive(&hspi3, (uint8_t*) frame_buf_1, 6400, 1000);
					SPI_master2slave(&spi3_buf, &frame_buf_1, 1);
				}
				else
				{
					HAL_SPI_Receive(&hspi3, (uint8_t*) frame_buf_tmp, 6400, 1000);
					SPI_master2slave(&spi3_buf, &frame_buf_tmp, 1);
				}
				break;
			case 0b00000011: //Command for DIP switch ID = 11
				if(BOARD_NUMBER==4)
				{
					HAL_SPI_Receive(&hspi3, (uint8_t*) frame_buf_1, 6400, 1000);
					SPI_master2slave(&spi3_buf, &frame_buf_1, 1);
				}
				else
				{
					HAL_SPI_Receive(&hspi3, (uint8_t*) frame_buf_tmp, 6400, 1000);
					SPI_master2slave(&spi3_buf, &frame_buf_tmp, 1);
				}
				break;
			default: //Broadcast to every board
				HAL_SPI_Receive(&hspi3, (uint8_t*) frame_buf_1, 6400, 1000);
				SPI_master2slave(&spi3_buf, &frame_buf_1, 1);
				break;
			}
			break;
		case 0b00010000: //Write Registers data
			SPI_master2slave(&spi3_buf, NULL, 0);
			switch (spi3_buf[0] & 0b00000111) {
			case 0b00000000: //Command for DIP switch ID = 00
				Write_Registers_data(0);
				break;
			case 0b00000001: //Command for DIP switch ID = 01
				if(BOARD_NUMBER==2)
				{
					Write_Registers_data(1);
				}
				else
				{
					Write_Registers_data(0);
				}
				break;
			case 0b00000010: //Command for DIP switch ID = 10
				if(BOARD_NUMBER==3)
				{
					Write_Registers_data(1);
				}
				else
				{
					Write_Registers_data(0);
				}
				break;
			case 0b00000011: //Command for DIP switch ID = 11
				if(BOARD_NUMBER==4)
				{
					Write_Registers_data(1);
				}
				else
				{
					Write_Registers_data(0);
				}
				break;
			default: //Broadcast to every board
				Write_Registers_data(1);
				break;
			}
			break;
		case 0b00011000: //Write partial content of frame buffer
			break;
		case 0b00100000: //Read content of full frame buffer to fram buffer (0)
			break;
		case 0b00101000: //Read content of full frame buffer to fram buffer (1)
			break;
		case 0b00110000: //Read Registers data
			break;
		case 0b00111000: //Read partial content of frame buffer
			break;
		}
		break;
	case 0b01000000:  //Master SPI functions
		switch (spi3_buf[0] & 0b00111000) {
		case 0b00000000: //Start SPI write data
			break;
		case 0b00001000: //Continuous write SPI data
			break;
		case 0b00010000: //End SPI write data
			break;
		case 0b00011000: //Start SPI Read data
			break;
		case 0b00100000: //Continuous Read SPI data
			break;
		case 0b00101000: //End SPI Read data
			break;
		}
		break;
	case 0b10000000:  //I2C command
		switch (spi3_buf[0] & 0b00111000) {
		case 0b00000000: //I2C Write Data
			break;
		case 0b00100000: //I2C Read Data
			break;
		}
		break;
	case 0b11000000:  //SPI flash function & Slave SPI
		switch (spi3_buf[0] & 0b00111000) {
		case 0b00000000: //Write data to SPI flash
			switch (spi3_buf[0] & 0b00000111) {
			case 0b00000000: //Command for DIP switch ID = 00
				break;
			case 0b00000001: //Command for DIP switch ID = 01
				HAL_SPI_Receive(&hspi3, (uint8_t*) frame_buf_tmp, 6400, 1000);
				SPI_master2slave(&spi3_buf, &frame_buf_tmp, 1);
				if(BOARD_NUMBER==2)
				{
					erase_flash_sector(spi3_buf[1] - 1);
					write_flash_page(&frame_buf_tmp, spi3_buf[1] - 1);
				}
				break;
			case 0b00000010: //Command for DIP switch ID = 10
				HAL_SPI_Receive(&hspi3, (uint8_t*) frame_buf_tmp, 6400, 1000);
				SPI_master2slave(&spi3_buf, &frame_buf_tmp, 1);
				if(BOARD_NUMBER=3)
				{
					erase_flash_sector(spi3_buf[1] - 1);
					write_flash_page(&frame_buf_tmp, spi3_buf[1] - 1);
				}
				break;
			case 0b00000011: //Command for DIP switch ID = 11
				HAL_SPI_Receive(&hspi3, (uint8_t*) frame_buf_tmp, 6400, 1000);
				SPI_master2slave(&spi3_buf, &frame_buf_tmp, 1);
				if(BOARD_NUMBER==4)
				{
					erase_flash_sector(spi3_buf[1] - 1);
					write_flash_page(&frame_buf_tmp, spi3_buf[1] - 1);
				}
				break;
			default: //Broadcast to every board
				HAL_SPI_Receive(&hspi3, (uint8_t*) frame_buf_tmp, 6400, 1000);
				SPI_master2slave(&spi3_buf, &frame_buf_tmp, 1);
				erase_flash_sector(spi3_buf[1] - 1);
				write_flash_page(&frame_buf_tmp, spi3_buf[1] - 1);
				break;
			}
			break;
		case 0b00001000: //Read data from SPI flash
			break;
		case 0b00100000: //Display Data by Slave SPI
			break;
		}
		break;
	}
	// USB command: ID
	switch (spi3_buf[0] & 0b00000111) {
	case 0b00000000:  //Command for DIP switch ID = 00
		break;
	case 0b00000001:  //Command for DIP switch ID = 01
		break;
	case 0b00000010:  //Command for DIP switch ID = 10
		break;
	case 0b00000011:  //Command for DIP switch ID = 11
		break;
	default:  //Broadcast to every board
		break;
	}

	HAL_SPI_Receive_IT(&hspi3, &spi3_buf, 2);
}

void FLASH_WriteByte(uint8_t data) {
    for (int i = 7; i >= 0; i--) {
        HAL_GPIO_WritePin(FLASH_MOSI_PORT, FLASH_MOSI_PIN, (data >> i) & 1);

        HAL_GPIO_WritePin(FLASH_CLK_PORT, FLASH_CLK_PIN, GPIO_PIN_SET);
        delay_100ns(1);
        HAL_GPIO_WritePin(FLASH_CLK_PORT, FLASH_CLK_PIN, GPIO_PIN_RESET);
        delay_100ns(1);
    }
}

uint8_t FLASH_ReadByte(void) {
    uint8_t data = 0;

    for (int i = 7; i >= 0; i--) {
        HAL_GPIO_WritePin(FLASH_CLK_PORT, FLASH_CLK_PIN, GPIO_PIN_SET);
        delay_100ns(1);
        data |= (HAL_GPIO_ReadPin(FLASH_MISO_PORT, FLASH_MISO_PIN) << i);
        HAL_GPIO_WritePin(FLASH_CLK_PORT,FLASH_CLK_PIN, GPIO_PIN_RESET);
        delay_100ns(1);
    }

    return data;
}

void write_flash_page(uint8_t *data, uint8_t image_id)
{
	int image_id_H = image_id/8;
	int image_id_L = image_id%8;
	int count=0;
	for(uint32_t i=image_id_L*0x20; i<image_id_L*0x20+25; i++)
	{
		// enable write
		HAL_GPIO_WritePin(FLASH_CS_PORT, FLASH_CS_PIN, GPIO_PIN_RESET);
		FLASH_WriteByte(0x06);
		HAL_GPIO_WritePin(FLASH_CS_PORT, FLASH_CS_PIN, GPIO_PIN_SET);

		// write data to flash page
		HAL_GPIO_WritePin(FLASH_CS_PORT, FLASH_CS_PIN, GPIO_PIN_RESET);
		FLASH_WriteByte(0x02);
		FLASH_WriteByte(image_id_H);
		FLASH_WriteByte(i);
		FLASH_WriteByte(0x00);
		for(uint32_t j=0; j<256; j++)
		{
			FLASH_WriteByte(data[count*256+j]);
		}
		HAL_GPIO_WritePin(FLASH_CS_PORT, FLASH_CS_PIN, GPIO_PIN_SET);

		// disable write
		HAL_GPIO_WritePin(FLASH_CS_PORT, FLASH_CS_PIN, GPIO_PIN_RESET);
		FLASH_WriteByte(0x04);
		HAL_GPIO_WritePin(FLASH_CS_PORT, FLASH_CS_PIN, GPIO_PIN_SET);
		delay_us(1000);
		count++;
	}
}

void read_flash_page(uint8_t *data, uint8_t image_id)
{
	int image_id_H = image_id/8;
	int image_id_L = image_id%8;
	int count=0;
	for(uint32_t i=image_id_L*0x20; i<image_id_L*0x20+25; i++)
	{
		HAL_GPIO_WritePin(FLASH_CS_PORT, FLASH_CS_PIN, GPIO_PIN_RESET);
		FLASH_WriteByte(0x03);
		FLASH_WriteByte(image_id_H);
		FLASH_WriteByte(i);
		FLASH_WriteByte(0x00);
		for(uint32_t j=0; j<256; j++)
		{
			data[count*256+j] = FLASH_ReadByte();
		}
		HAL_GPIO_WritePin(FLASH_CS_PORT, FLASH_CS_PIN, GPIO_PIN_SET);
		count++;
	}
}

void erase_flash_sector(uint8_t image_id)
{
	int image_id_H = image_id/8;
	int image_id_L = image_id%8;
	reset_flash_software();

	HAL_GPIO_WritePin(FLASH_CS_PORT, FLASH_CS_PIN, GPIO_PIN_RESET);
	FLASH_WriteByte(0x06);
	HAL_GPIO_WritePin(FLASH_CS_PORT, FLASH_CS_PIN, GPIO_PIN_SET);

	HAL_GPIO_WritePin(FLASH_CS_PORT, FLASH_CS_PIN, GPIO_PIN_RESET);
	FLASH_WriteByte(0x20);
	FLASH_WriteByte(image_id_H);
	FLASH_WriteByte(image_id_L*0x20);
	FLASH_WriteByte(0x00);
	HAL_GPIO_WritePin(FLASH_CS_PORT, FLASH_CS_PIN, GPIO_PIN_SET);

	HAL_GPIO_WritePin(FLASH_CS_PORT, FLASH_CS_PIN, GPIO_PIN_RESET);
	FLASH_WriteByte(0x04);
	HAL_GPIO_WritePin(FLASH_CS_PORT, FLASH_CS_PIN, GPIO_PIN_SET);

	delay_us(30000);
	////////////////////////////////////////////////////////////
	HAL_GPIO_WritePin(FLASH_CS_PORT, FLASH_CS_PIN, GPIO_PIN_RESET);
	FLASH_WriteByte(0x06);
	HAL_GPIO_WritePin(FLASH_CS_PORT, FLASH_CS_PIN, GPIO_PIN_SET);

	HAL_GPIO_WritePin(FLASH_CS_PORT, FLASH_CS_PIN, GPIO_PIN_RESET);
	FLASH_WriteByte(0x20);
	FLASH_WriteByte(image_id_H);
	FLASH_WriteByte(image_id_L*0x20+0x10);
	FLASH_WriteByte(0x00);
	HAL_GPIO_WritePin(FLASH_CS_PORT, FLASH_CS_PIN, GPIO_PIN_SET);

	HAL_GPIO_WritePin(FLASH_CS_PORT, FLASH_CS_PIN, GPIO_PIN_RESET);
	FLASH_WriteByte(0x04);
	HAL_GPIO_WritePin(FLASH_CS_PORT, FLASH_CS_PIN, GPIO_PIN_SET);

	delay_us(30000);
}

void reset_flash_software()
{
	HAL_GPIO_WritePin(FLASH_CS_PORT, FLASH_CS_PIN, GPIO_PIN_RESET);
	FLASH_WriteByte(0x66);
	HAL_GPIO_WritePin(FLASH_CS_PORT, FLASH_CS_PIN, GPIO_PIN_SET);
	delay_100ns(1);
	HAL_GPIO_WritePin(FLASH_CS_PORT, FLASH_CS_PIN, GPIO_PIN_RESET);
	FLASH_WriteByte(0x99);
	HAL_GPIO_WritePin(FLASH_CS_PORT, FLASH_CS_PIN, GPIO_PIN_SET);
	delay_100ns(1);

	delay_us(1000);
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
	while (1) {
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
