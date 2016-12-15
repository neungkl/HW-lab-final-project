/**
 ******************************************************************************
 * File Name          : main.c
 * Description        : Main program body
 ******************************************************************************
 *
 * COPYRIGHT(c) 2016 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s2;
I2S_HandleTypeDef hi2s3;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S2_Init(void);
static void MX_I2S3_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
#define PDM_BUFFER_SIZE 16
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

int Accelerometer_Get_Direction();

// ###################
// #                 #
// #   Game Logic    #
// #                 #
// ###################

uint8_t INIT = 0;
uint8_t WAITING = 1;
uint8_t PLAY = 2;
uint8_t DIE = 3;

uint8_t LEFT = 0;
uint8_t RIGHT = 1;
uint8_t UP = 2;
uint8_t DOWN = 3;

uint16_t i, j, k;
uint8_t gState = 0;

uint8_t gStartMsg[9][18] = {
	"+--------------+\n\r",
	"|              |\n\r",
	"|   Response   |\n\r",
	"|     Game     |\n\r",
	"|              |\n\r",
	"|==============|\n\r",
	"|    Snap To   |\n\r",
	"|     Start    |\n\r",
	"+--------------+\n\r"
};

uint8_t gIdleMsg[9][18] = {
	"+--------------+\n\r",
	"|              |\n\r",
	"|   Waiting..  |\n\r",
	"| .       .    |\n\r",
	"|      .       |\n\r",
	"|   .     .    |\n\r",
	"|              |\n\r",
	"| .   .     .  |\n\r",
	"+--------------+\n\r"
};

uint8_t gUpMsg[9][18] = {
	"+--------------+\n\r",
	"|              |\n\r",
	"|      /\\      |\n\r",
	"|     /##\\     |\n\r",
	"|    /####\\    |\n\r",
	"|     |##|     |\n\r",
	"|     |##|     |\n\r",
	"|              |\n\r",
	"+--------------+\n\r"
};

uint8_t gDownMsg[9][18] = {
	"+--------------+\n\r",
	"|              |\n\r",
	"|     |##|     |\n\r",
	"|     |##|     |\n\r",
	"|    \\####/    |\n\r",
	"|     \\##/     |\n\r",
	"|      \\/      |\n\r",
	"|              |\n\r",
	"+--------------+\n\r"
};

uint8_t gLeftMsg[9][18] = {
	"+--------------+\n\r",
	"|       \\      |\n\r",
	"|       ##\\    |\n\r",
	"|   ########\\  |\n\r",
	"|   ########/  |\n\r",
	"|       ##/    |\n\r",
	"|       /      |\n\r",
	"|              |\n\r",
	"+--------------+\n\r"
};

uint8_t gRightMsg[9][18] = {
	"+--------------+\n\r",
	"|      /       |\n\r",
	"|    /##       |\n\r",
	"|  /#########  |\n\r",
	"|  \\#########  |\n\r",
	"|    \\##       |\n\r",
	"|      \\       |\n\r",
	"|              |\n\r",
	"+--------------+\n\r"
};

uint8_t gLoseMsg[9][18] = {
	"+--------------+\n\r",
	"|              |\n\r",
	"|      You     |\n\r",
	"|     Lose     |\n\r",
	"|              |\n\r",
	"|==============|\n\r",
	"|    Snap To   |\n\r",
	"|    Restart   |\n\r",
	"+--------------+\n\r"
};

uint8_t gBuffer[7][14];
uint32_t gPrevTime;
uint32_t gCurTime;

uint32_t rPrevTime;
uint32_t rCurTime;

uint8_t gCurrentDirection;
uint8_t gDirectionTime;

uint32_t random() {
	return HAL_GetTick();
}

void Game_Init() {
	for(i=0; i<7; i++) {
		for(j=0; j<14; j++) gBuffer[i][j] = 0;
	}
}

void Game_Random_Direction() {
	gCurrentDirection = random() % 4;
	gDirectionTime = random() % 1000 + 200;
}

int Game_Die() {

	uint8_t dir = Accelerometer_Get_Direction();
	if(dir == 5) return 0;

	if(dir != gCurrentDirection) return 0;

	uint8_t pass = 0;
	for(i = 0; i<7; i++) {
		for(j=0; j<14; j++) {
			if(gBuffer[i][j] == ' ') pass = 1;
		}
	}

	return pass;
}

void Game_Random_Decrease_Life() {
	if(random() % 10 != 0) return ;

	uint8_t N = 7 * 14;
	uint8_t pos = random() % N;

	for(i=0; i<N; i++) {
		pos++;
		if(pos >= N) pos = 0;
		if(gBuffer[pos / 14][pos % 14] == ' ') {
			gBuffer[pos / 14][pos % 14] = '.';
			break;
		}
	}
}

void Game_Render() {
	if(gState == INIT) {
		for(i=0; i<7; i++) {
			HAL_UART_Transmit(&huart2, (uint8_t*) gStartMsg[i], 18, 100);
		}
	} else if(gState == WAITING) {
		for(i=0; i<7; i++) {
			HAL_UART_Transmit(&huart2, (uint8_t*) gIdleMsg[i], 18, 100);
		}
	} else if(gState == DIE) {
		for(i=0; i<7; i++) {
			HAL_UART_Transmit(&huart2, (uint8_t*) gLoseMsg[i], 18, 100);
		}
	} else if(gState == PLAY) {
		uint8_t **msg;

		if(gCurrentDirection == LEFT) msg = gLeftMsg;
		else if(gCurrentDirection == RIGHT) msg = gRightMsg;
		else if(gCurrentDirection == UP) msg = gUpMsg;
		else msg = gDownMsg;

		for(i=0; i<9; i++) {
			for(j=0; j<18; j++) {
				if(msg[i][j] == ' ') {
					HAL_UART_Transmit(&huart2, &msg[i][j], 1, 100);
				} else {
					HAL_UART_Transmit(&huart2, &gBuffer[i-1][j-1], 1, 100);
				}
			}
		}
	}
}

// ###################
// #                 #
// #     Speaker     #
// #                 #
// ###################

uint16_t sTemp[100];
uint8_t sSendData[PDM_BUFFER_SIZE];

uint16_t pdm_buffer[PDM_BUFFER_SIZE];

int Transmit_Audio_Data(uint8_t address, uint8_t data) {
	sSendData[0] = address;
	sSendData[1] = data;
	return HAL_I2C_Master_Transmit(&hi2c1, 0x94, sSendData, 2, 50);
}

void Play_Sound(uint8_t chord) {

	if (chord == 0x00)
		return;

	Transmit_Audio_Data(0x1E, 0x20);
	Transmit_Audio_Data(0x1C, chord);
	Transmit_Audio_Data(0x1E, 0xE0);

	int i;
	for (i = 0; i < 100; i++)
		HAL_I2S_Transmit(&hi2s3, sTemp, 100, 10);
}

void Audio_Init() {

	int i;
	for (i = 0; i < 100; i++)
		sTemp[i] = 0;

	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, 0);
	HAL_Delay(500);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, 1); //Reset is set Up (Power CS43L22)
	HAL_Delay(500);

	Transmit_Audio_Data(0x00, 0x99);
	Transmit_Audio_Data(0x47, 0x80);
	Transmit_Audio_Data(0x32, 0x80);
	Transmit_Audio_Data(0x32, 0x00);
	Transmit_Audio_Data(0x00, 0x00);
	Transmit_Audio_Data(0x1E, 0xC0);
	Transmit_Audio_Data(0x02, 0x9E);

	// Adjust Volume
	Transmit_Audio_Data(0x1D, 0x0F);

}

// ###################
// #                 #
// #  Accelerometer  #
// #                 #
// ###################

void Accelerometer_Init() {
	uint8_t aAddress = 0x20;
	uint8_t aData = 0x67;

	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &aAddress, 1, 50);
	HAL_SPI_Transmit(&hspi1, &aData, 1, 50);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
}

int Accelerometer_Get_Direction() {
	uint8_t aX, aY, aZ;
	uint8_t aAddress;

	int rotateThreshold = 20;

	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_RESET);

	aAddress = 0x29 | 0x80;
	HAL_SPI_Transmit(&hspi1, &aAddress, 1, 50);
	HAL_SPI_Receive(&hspi1, &aX, 1, 50);

	aAddress = 0x2B | 0x80;
	HAL_SPI_Transmit(&hspi1, &aAddress, 1, 50);
	HAL_SPI_Receive(&hspi1, &aY, 1, 50);

	aAddress = 0x2C | 0x80;
	HAL_SPI_Transmit(&hspi1, &aAddress, 1, 50);
	HAL_SPI_Receive(&hspi1, &aZ, 1, 50);

	if (aX < 255 - rotateThreshold && aX > 128) {
		// Left On
		return LEFT;
	} else if (aX <= 128 && aX > rotateThreshold) {
		// Right On
		return RIGHT;
	}

	if (aY > rotateThreshold && aY < 128) {
		// Up On
		return UP;
	} else if (aY >= 128 && aY < 255 - rotateThreshold) {
		// Down On
		return DOWN;
	}

	return 5;
}

// ###################
// #                 #
// #    Microphone   #
// #                 #
// ###################

int Is_Loud() {

	uint16_t bit_position;

	HAL_I2S_Receive(&hi2s2, pdm_buffer, PDM_BUFFER_SIZE, 100);

	uint16_t pdm_count = 0;

	int i, j;
	for (i = 0; i < PDM_BUFFER_SIZE; i++) {
		bit_position = (1 << 15);
		for (j = 0; j < 16; j++) {
			if (bit_position & pdm_buffer[i]) {
				pdm_count++;
			}
			bit_position >>= 1;
		}
	}

	if (pdm_count > 130) {
		return 1;
	} else {
		return 0;
	}
}
/* USER CODE END 0 */

int main(void) {

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_I2C1_Init();
	MX_I2S2_Init();
	MX_I2S3_Init();
	MX_SPI1_Init();
	MX_TIM1_Init();
	MX_USART2_UART_Init();

	/* USER CODE BEGIN 2 */
	Audio_Init();
	Accelerometer_Init();

	gPrevTime = HAL_GetTick();
	rPrevTime = gPrevTime;

	int rotateThreshold = 20;
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

//		if(gState == INIT) {
//			if(Is_Loud()) {
//				Game_Init();
//				gState == WAITING;
//			}
//		} else if(gState == WAITING) {
//			gCurTime = HAL_GetTick();
//			if(gCurTime - gPrevTime > 1500) {
//				gPrevTime = gCurTime;
//				gState = PLAY;
//				Game_Random_Direction();
//			}
//		} else if(gState == PLAY) {
//			gCurTime = HAL_GetTick();
//			if(gCurTime - gPrevTime > gDirectionTime) {
//				Game_Random_Direction();
//			} else {
//				if(Game_Die()) {
//					gState = DIE;
//				} else {
//					Game_Random_Decrease_Life();
//				}
//			}
//		} else if(gState == DIE) {
//			if(Is_Loud()) {
//				gState = INIT;
//			}
//		}

		rCurTime = HAL_GetTick();
		if(rCurTime - rPrevTime > 100) {
			Game_Render();
			rPrevTime = rCurTime;
		}

		uint8_t aX, aY, aZ;
		uint8_t aAddress;

		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_RESET);

		aAddress = 0x29 | 0x80;
		HAL_SPI_Transmit(&hspi1, &aAddress, 1, 50);
		HAL_SPI_Receive(&hspi1, &aX, 1, 50);

		aAddress = 0x2B | 0x80;
		HAL_SPI_Transmit(&hspi1, &aAddress, 1, 50);
		HAL_SPI_Receive(&hspi1, &aY, 1, 50);

		aAddress = 0x2C | 0x80;
		HAL_SPI_Transmit(&hspi1, &aAddress, 1, 50);
		HAL_SPI_Receive(&hspi1, &aZ, 1, 50);

		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_SET);

		if (aX < 255 - rotateThreshold && aX > 128) {
			// Left On
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
		} else if (aX <= 128 && aX > rotateThreshold) {
			// Right On
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
		} else {
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
		}

		if (aY > rotateThreshold && aY < 128) {
			// Up On
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
		} else if (aY >= 128 && aY < 255 - rotateThreshold) {
			// Down On
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
		} else {
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
		}

		Play_Sound(0x1A);
		HAL_Delay(100);
	}
	/* USER CODE END 3 */

}

/** System Clock Configuration
 */
void SystemClock_Config(void) {

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

	/**Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE()
	;

	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 168;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
		Error_Handler();
	}

	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
	PeriphClkInitStruct.PLLI2S.PLLI2SN = 88;
	PeriphClkInitStruct.PLLI2S.PLLI2SR = 4;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/**Configure the Systick interrupt time
	 */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	/**Configure the Systick
	 */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void) {

	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 50000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}

}

/* I2S2 init function */
static void MX_I2S2_Init(void) {

	hi2s2.Instance = SPI2;
	hi2s2.Init.Mode = I2S_MODE_MASTER_RX;
	hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
	hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B;
	hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
	hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_44K;
	hi2s2.Init.CPOL = I2S_CPOL_LOW;
	hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
	hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
	if (HAL_I2S_Init(&hi2s2) != HAL_OK) {
		Error_Handler();
	}

}

/* I2S3 init function */
static void MX_I2S3_Init(void) {

	hi2s3.Instance = SPI3;
	hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
	hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
	hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
	hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
	hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_44K;
	hi2s3.Init.CPOL = I2S_CPOL_LOW;
	hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
	hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
	if (HAL_I2S_Init(&hi2s3) != HAL_OK) {
		Error_Handler();
	}

}

/* SPI1 init function */
static void MX_SPI1_Init(void) {

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
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		Error_Handler();
	}

}

/* TIM1 init function */
static void MX_TIM1_Init(void) {

	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_MasterConfigTypeDef sMasterConfig;

	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 1680;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 99;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}

}

/* USART2 init function */
static void MX_USART2_UART_Init(void) {

	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}

}

/** Configure pins as 
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 PA9   ------> USB_OTG_FS_VBUS
 PA10   ------> USB_OTG_FS_ID
 PA11   ------> USB_OTG_FS_DM
 PA12   ------> USB_OTG_FS_DP
 */
static void MX_GPIO_Init(void) {

	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOE_CLK_ENABLE()
	;
	__HAL_RCC_GPIOC_CLK_ENABLE()
	;
	__HAL_RCC_GPIOH_CLK_ENABLE()
	;
	__HAL_RCC_GPIOA_CLK_ENABLE()
	;
	__HAL_RCC_GPIOB_CLK_ENABLE()
	;
	__HAL_RCC_GPIOD_CLK_ENABLE()
	;

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin,
			GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOD,
			LD4_Pin | LD3_Pin | LD5_Pin | LD6_Pin | Audio_RST_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin : CS_I2C_SPI_Pin */
	GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
	GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : BOOT1_Pin */
	GPIO_InitStruct.Pin = BOOT1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
	 Audio_RST_Pin */
	GPIO_InitStruct.Pin = LD4_Pin | LD3_Pin | LD5_Pin | LD6_Pin | Audio_RST_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pin : VBUS_FS_Pin */
	GPIO_InitStruct.Pin = VBUS_FS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(VBUS_FS_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : OTG_FS_ID_Pin OTG_FS_DM_Pin OTG_FS_DP_Pin */
	GPIO_InitStruct.Pin = OTG_FS_ID_Pin | OTG_FS_DM_Pin | OTG_FS_DP_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
	GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : MEMS_INT1_Pin */
	GPIO_InitStruct.Pin = MEMS_INT1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(MEMS_INT1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : MEMS_INT2_Pin */
	GPIO_InitStruct.Pin = MEMS_INT2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
	}
	/* USER CODE END Error_Handler */
}

#ifdef USE_FULL_ASSERT

/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */

}

#endif

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
