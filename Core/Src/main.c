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
#include "string.h"
#include "cmsis_os.h"
#include <stdio.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mongoose_glue.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MMA8451_ADDR 0x1D
#define F_STATUS 0x00
#define X_MSB 0x01
#define X_LSB 0x02
#define Y_MSB 0x03
#define Y_LSB 0x04
#define Z_MSB 0x05
#define Z_LSB 0x06
#define WHOAMI_REG 0x0D
#define CTRL1 0x2A
#define CTRL2 0x2B
#define G 9.8

asm(".global _printf_float");
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma location=0x2007c000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x2007c0a0
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x2007c000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x2007c0a0))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __GNUC__ ) /* GNU Compiler */

ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */
#endif

ETH_TxPacketConfig TxConfig;

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

ETH_HandleTypeDef heth;

I2C_HandleTypeDef hi2c1;

RNG_HandleTypeDef hrng;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart3;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
char car_status[64] = "Braking";
uint16_t PWM_Value2 = 0;
uint16_t PWM_Value3 = 0;
uint16_t PWM_Value4 = 0;
uint16_t PWM_Value5 = 0;
uint16_t aksx; 						/* Fra akselerometer */
uint16_t aksy; 						/* Fra akselerometer */
uint16_t fart_input = 0; 			/* Kontroller fart */
uint16_t turning_input = 0; 		/* Fra joystick */
float current_speed = 0; 			/* Kalkulert med akselerasjon */
bool front_lights = false;			/* Status for front lights on car */
bool brake_lights = false;			/* Status for brake lights on car */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_RNG_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C1_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
struct car_lights {
    bool front_light;
    bool brake_light;
};

struct car_status {
	char status[64];
	int speed;
};

void BrakeStatusTask(void *argument) {
    for (;;) {
        // Update the brake light variable based on car state
        brake_lights = strcmp(car_status, "Braking") == 0;

        // Update GPIO hardware
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, brake_lights ? GPIO_PIN_SET : GPIO_PIN_RESET);

        osDelay(200); // every 200ms
    }
}

void CarControlTask(void *argument) {
    for (;;) {
        // Update the brake light variable based on car state
    	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, PWM_Value2);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, PWM_Value3);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, PWM_Value4);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, PWM_Value5);

		  /*Når bilen har bremset og skal kjøre frem*/
		  if (fart_input > 2045 && current_speed >= 0 && turning_input < 2400)
			{
			  PWM_Value4 = (4095 / 2045)*(fart_input - 2045);
			  PWM_Value5 = 0;

			  PWM_Value2 = (4095 / 2045)*(fart_input - 2045);
			  PWM_Value3 = 0;
			  strcpy(car_status, "Driving");
			}

		  if (fart_input <= 2045 && current_speed <= 0)
			{
			  PWM_Value4 = 0;
			  PWM_Value5 = -(4095/2045) * fart_input + 4095;

			  PWM_Value2 = 0;
			  PWM_Value3 = -(4095/2045) * fart_input + 4095;
			  strcpy(car_status, "Driving");
			}

		  /*Når bilen skal bremse og den kjører fremover*/

		  if (current_speed > 0 && fart_input <= 2045)
			{
			  PWM_Value4 = 0;
			  PWM_Value5 = 0;

			  PWM_Value2 = 0;
			  PWM_Value3 = 0;
			  strcpy(car_status, "Braking");
			}

			/*Når bilen skal bremse og den kjører bakover*/

		  if (current_speed < 0 && fart_input >= 2045)
			{
			  PWM_Value2 = 0;
			  PWM_Value3 = 0;

			  PWM_Value4 = 0;
			  PWM_Value5 = 0;
			  strcpy(car_status, "Braking");
			}

			/*Svinging når bilen kjører frammover*/

		  if (current_speed >= 0 && fart_input >= 2045 && turning_input < 1800)
			{
			  PWM_Value4 = (4095 / 2045)*(fart_input - 2045);
			  PWM_Value5 = 0;

			  PWM_Value2 = (4095 / 2045)*(fart_input - 2045) - 500;
			  PWM_Value3 = 0;
			  strcpy(car_status, "Turning left");
			}

		  if (current_speed >= 0 && fart_input >= 2045 && turning_input > 2400)
			{
			  PWM_Value4 =   (4095 / 2045)*(fart_input - 2045) - 500;
			  PWM_Value5 = 0;

			  PWM_Value2 = (4095 / 2045)*(fart_input - 2045);
			  PWM_Value3 = 0;
			  strcpy(car_status, "Turning left");
			}

		  if (current_speed <= 2045 && fart_input <= 2045)
			{
			  PWM_Value4 = 0;
			  PWM_Value5 = -(4095/2045) * fart_input + 4095;

			  PWM_Value2 = 0;
			  PWM_Value3 = -(4095/2045) * fart_input + 4095 - 500;
			  strcpy(car_status, "Turning right");
			}

		  if (current_speed <= 2045 && fart_input <= 2045 && turning_input < 1800)
			{
			  PWM_Value4 = 0;
			  PWM_Value5 = -(4095/2045) * fart_input + 4095 - 500;

			  PWM_Value2 = 0;
			  PWM_Value3 = -(4095/2045) * fart_input + 4095;
			  strcpy(car_status, "Turning right");
			}



        osDelay(200); // every 200ms
    }
}

void ButtonTask(void *argument) {
    for (;;) {
        GPIO_PinState btn = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);

        if (btn == GPIO_PIN_SET) {
            strcpy(car_status, "Driving");
        }

        osDelay(100);
    }
}

void I2CTask(void *argument) {
	for (;;) {
		int8_t x_msb_value;
		    uint8_t x_lsb_value;
		    HAL_I2C_Mem_Read(&hi2c1, MMA8451_ADDR << 1, X_MSB, 1, &x_msb_value, 1, HAL_MAX_DELAY);
		    HAL_I2C_Mem_Read(&hi2c1, MMA8451_ADDR << 1, X_LSB, 1, &x_lsb_value, 1, HAL_MAX_DELAY);
		    int16_t x_akse = (x_msb_value << 8 | x_lsb_value); // 4;
		    x_akse /= 4;
		    printf("x_msb: %02x: x_lsb %02x,x-akse: %i\r\n", x_msb_value, x_lsb_value, x_akse);

		    int8_t y_msb_value;
		    uint8_t y_lsb_value;
		    HAL_I2C_Mem_Read(&hi2c1, MMA8451_ADDR << 1, Y_MSB, 1, &y_msb_value, 1, HAL_MAX_DELAY);
		    HAL_I2C_Mem_Read(&hi2c1, MMA8451_ADDR << 1, Y_LSB, 1, &y_lsb_value, 1, HAL_MAX_DELAY);
		    int16_t y_akse = (y_msb_value << 8 | y_lsb_value); // 4;
		    y_akse /= 4;
		    printf("y_msb: %02x: y_lsb %02x,y-akse: %i\r\n", y_msb_value, y_lsb_value, y_akse);

		    int8_t z_msb_value;
		    uint8_t z_lsb_value;
		    HAL_I2C_Mem_Read(&hi2c1, MMA8451_ADDR << 1, Z_MSB, 1, &z_msb_value, 1, HAL_MAX_DELAY);
		    HAL_I2C_Mem_Read(&hi2c1, MMA8451_ADDR << 1, Z_LSB, 1, &z_lsb_value, 1, HAL_MAX_DELAY);
		    int16_t z_akse = (z_msb_value << 8 | z_lsb_value); // 4;
		    z_akse /= 4;
		    printf("z_msb: %i: z_lsb %i,z-akse: %i\r\n", z_msb_value, z_lsb_value, z_akse);

		    float x_akslerasjon = (x_akse*2*G)/8092.0;
		    float y_akslerasjon = (y_akse*2*G)/8092.0;
		    float z_akslerasjon = (z_akse*2*G)/8092.0;

		    current_speed = current_speed + y_akslerasjon * 0.1; 					// Calculate the speed in y-direction

		    printf("x-akse: %4.2f m/s^2, y-akse: %4.2f m/s^2,z-akse: %4.2f m/s^2\r\n",
		        x_akslerasjon,y_akslerasjon,z_akslerasjon);

		        osDelay(100);
	}
}

void set_lights(struct lights *data) {
    front_lights = data -> front;
    brake_lights = data -> brake;

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, front_lights ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, brake_lights ? GPIO_PIN_SET : GPIO_PIN_RESET);

    printf("Lights updated: front=%d, brake=%d\r\n", data -> front, data -> brake);
}

void get_lights(struct lights *data) {
    data -> front = front_lights;
    data -> brake = brake_lights;

    printf("Lights read: front=%d, brake=%d\r\n", data -> front, data -> brake);
}

void get_status(struct status *data) {
    data -> speed = current_speed;
    strcpy(data -> user_input, car_status);
    printf("Status GET: speed=%d, user_input=%s\r\n", data -> speed, data -> user_input);
}

void set_status(struct status *data) {
    current_speed = data -> speed;
    strcpy(car_status, data -> user_input);
    printf("Status SET: speed=%.2f, user_input=%s\r\n", current_speed, car_status);
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
bool mg_random(void *buf, size_t len) {  // Use on-board RNG
  for (size_t n = 0; n < len; n += sizeof(uint32_t)) {
    uint32_t r;
    HAL_RNG_GenerateRandomNumber(&hrng, &r);
    memcpy((char *) buf + n, &r, n + sizeof(r) > len ? len - n : sizeof(r));
  }
  return true; // TODO(): ensure successful RNG init, then return on false above
}

int _write(int fd, unsigned char *buf, int len) {
  if (fd == 1 || fd == 2) {                     // stdout or stderr ?
    HAL_UART_Transmit(&huart3, buf, len, 999);  // Print to the UART
  }
  return len;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	uint8_t i2c_data;
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
  MX_ETH_Init();
  MX_RNG_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);

	uint8_t rx_data;

	// Software-reset av MMA8451
	uint8_t tx_data = 1<<6;
	HAL_I2C_Mem_Write(&hi2c1,MMA8451_ADDR<<1,CTRL2,1,&tx_data,1,HAL_MAX_DELAY);
	HAL_Delay(1000);
	uint8_t ctrl2_value;
	HAL_I2C_Mem_Read(&hi2c1, MMA8451_ADDR << 1, CTRL2, 1, &ctrl2_value, 1, HAL_MAX_DELAY);

	uint8_t ctrl1_value;
	HAL_I2C_Mem_Read(&hi2c1, MMA8451_ADDR << 1, CTRL1, 1, &ctrl1_value, 1, HAL_MAX_DELAY);

	uint8_t whoami_value;
	HAL_I2C_Mem_Read(&hi2c1, MMA8451_ADDR << 1, WHOAMI_REG, 1, &whoami_value, 1, HAL_MAX_DELAY);

	uint8_t fstatus_value;
	HAL_I2C_Mem_Read(&hi2c1, MMA8451_ADDR << 1, F_STATUS, 1, &fstatus_value, 1, HAL_MAX_DELAY);

	printf("Etter reset: ctrl2: %02X ctrl1: %02X whoami: %02X fstatus: %02X\n",
			ctrl2_value, ctrl1_value, whoami_value, fstatus_value);

	// Set config til ACTIVE
	tx_data = 1;
	HAL_I2C_Mem_Write(&hi2c1,MMA8451_ADDR<<1,CTRL1,1,&tx_data,1,HAL_MAX_DELAY);
	HAL_Delay(100);

	HAL_I2C_Mem_Read(&hi2c1, MMA8451_ADDR << 1, CTRL2, 1, &ctrl2_value, 1, HAL_MAX_DELAY);
	HAL_I2C_Mem_Read(&hi2c1, MMA8451_ADDR << 1, CTRL1, 1, &ctrl1_value, 1, HAL_MAX_DELAY);
	HAL_I2C_Mem_Read(&hi2c1, MMA8451_ADDR << 1, WHOAMI_REG, 1, &whoami_value, 1, HAL_MAX_DELAY);
	HAL_I2C_Mem_Read(&hi2c1, MMA8451_ADDR << 1, F_STATUS, 1, &fstatus_value, 1, HAL_MAX_DELAY);

	printf("Etter config: ctrl2: %02X ctrl1: %02X whoami: %02X fstatus: %02X\r\n",
			ctrl2_value, ctrl1_value, whoami_value, fstatus_value);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

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
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
  osThreadNew(BrakeStatusTask, NULL, NULL);
  osThreadNew(CarControlTask, NULL, NULL);
  osThreadNew(ButtonTask, NULL, NULL);
  osThreadNew(I2CTask, NULL, NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
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
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
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
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

   static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

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
  hi2c1.Init.Timing = 0x20404768;
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
  * @brief RNG Initialization Function
  * @param None
  * @retval None
  */
static void MX_RNG_Init(void)
{

  /* USER CODE BEGIN RNG_Init 0 */

  /* USER CODE END RNG_Init 0 */

  /* USER CODE BEGIN RNG_Init 1 */

  /* USER CODE END RNG_Init 1 */
  hrng.Instance = RNG;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RNG_Init 2 */

  /* USER CODE END RNG_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 4;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 4095;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 4;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 4095;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|GPIO_PIN_14|GPIO_PIN_7|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pins : LD1_Pin PB14 PB7 PB8 */
  GPIO_InitStruct.Pin = LD1_Pin|GPIO_PIN_14|GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  mongoose_init();
  mongoose_set_http_handlers("lights", get_lights, set_lights, NULL);					// Handler for sending/getting the lights status on the car
  mongoose_set_http_handlers("status", get_status, set_status, NULL);					// Handler for sending/getting the status to/from API endpoint
  for (;;) {
    mongoose_poll();
    glue_update_state();
  }
  /* Infinite loop */
  for(;;)
  {
	  osDelay(100);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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
#ifdef USE_FULL_ASSERT
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
