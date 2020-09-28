/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

#include "I2Cdev.h"
#include "HMC5883L.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//KOMPAS
#define PI	3.14159265358979f
		/*PID YAW*/
#define KP_YAW 20
#define KI_YAW 0
#define KD_YAW 55
		/*PID PITCH*/
#define KP_PITCH 100
#define KI_PITCH 0
#define KD_PITCH 110

void input_data_wahana();
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */
uint8_t buffer[200];
uint16_t ukuranstring;
//----------KOMPAS-------
int16_t x_heading, y_heading, z_heading;
float heading, headingDegrees, declinationAngle;
//---------potensio------
uint32_t data_potensio;
float analog_potensio;
//---------var Control PID-----
int Out_PIDPITCH, Out_PIDYAW;
int dir_pitch = 0;
int dir_yaw = 0;
float KI_pitch = 0;
float KI_yaw = 0;
float errorPitch, last_errorPitch, pitch_P, pitch_I, pitch_D;
float errorYaw, last_errorYaw, yaw_P, yaw_I, yaw_D;
float errorI_yaw, hasilErrorYaw, pidy;
float data_heading, data_Pitch1;
//---------VAR GPS ACSES-----
float lon_gps, lat_gps;
DMA_Event_t dma_uart4_rx = {0, 0, DMA_BUF_SIZE};
uint8_t dma_rx4_buf[DMA_BUF_SIZE];
uint8_t data_gps[DMA_BUF_SIZE] = {'\0'};
bool UART4DataFlag = false;
//----------VAR DATA INPUT----
DMA_Event_t dma_uart1_rx = {0, 0, DMA_BUF_SIZE};
uint8_t dma_rx1_buf[DMA_BUF_SIZE];
uint8_t data_input[DMA_BUF_SIZE] = {'\0'};
bool USART1DataFlag = false;
char setPoint_pitchAct[10], setPoint_yawAct[10];
float setPoint_pitch = 0, setPoint_yaw=1;
int cek_in, flag_in, a_in, bit_in, ke_in = 0, metu_in, mode_in = 0;
// --------drivermotor---
int arah;
uint16_t pwm;
// --------Data Wahana -----
DMA_Event_t dma_uart3_rx = {0, 0, DMA_BUF_SIZE};
uint8_t dma_rx3_buf[DMA_BUF_SIZE];
uint8_t data_wahana[DMA_BUF_SIZE] = {'\0'};
bool USART3DataFlag = false;
int cek_w, a_w, flag_w, bit_w, ke_w = 0, out_w;
char data_mode_w[10], data_yaw_w[10], data_pitch_w[10], data_roll_w[10];
char data_angin_w[10], data_alti_w[10], data_lat_w[10], data_long_w[10], data_bat_w[10];
int mode_w;
float yaw_w, pitch_w, roll_w, angin_w, alti_w, lat_w, long_w, bat_w;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_UART4_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float baca_potensio(){
	HAL_ADC_Start(&hadc1);
	while (__HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_EOC) != SET){;}
		  HAL_ADC_Stop(&hadc1);
		  data_potensio = HAL_ADC_GetValue(&hadc1);
		  analog_potensio = (float)data_potensio;
		  if (analog_potensio > 5200) analog_potensio = 5200;
		  return analog_potensio*(180.0f/5200.0f);
}
void GetPOTENSIO(){
	  data_Pitch1 = baca_potensio();
}
void HMC5883LInit(){
	HMC5883L_initialize();
	HAL_Delay(1000);
	while (!HMC5883L_testConnection()){
		ukuranstring = sprintf((char*)buffer, "Inisialisasi HMC5893L GAGAL \r\n");
		HAL_UART_Transmit(&huart1, buffer, ukuranstring, 10);
		HMC5883L_initialize();
		HAL_Delay(500);
	}
	ukuranstring = sprintf((char*)buffer, "Inisialisasi HMC5893L Telah Berhasil \r\n");
	HAL_UART_Transmit(&huart1, buffer, ukuranstring, 10);
}
float bacaheading(){
	HMC5883L_getHeading(&x_heading, &y_heading, &z_heading);
	heading = (atan2(x_heading, y_heading));
	declinationAngle = 0.22;				//data untuk mendahului gerakan yaw saat wahana cepat
	heading += declinationAngle;
	//normalise PI 0 sampai dengan 360
	if (heading < 0){
		heading += 2*PI;
	}
	if (heading > 2*PI){
		heading -= 2*PI;
	}
	headingDegrees = heading * 180 / M_PI;	//konversi dari data radian menjadi degree
	if (headingDegrees < 0) headingDegrees += 360;
	else if (headingDegrees > 360) headingDegrees -= 360;
	return headingDegrees;
}
void GetHMC5893L(){
	if(HMC5883L_getReadyStatus()){
		data_heading = bacaheading();
	}
}
void cek_heading(){
	if(data_heading == 0.0){
		ukuranstring = sprintf((char*)buffer, "data heading INVALID !!\r\n");
		HAL_UART_Transmit(&huart1, buffer, ukuranstring, 10);
		return;
	}
}
void SetDriverMotor_yaw(int arah, uint16_t pwm){
	switch (arah){
	case 0:
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
		break;
	case 1:
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
		break;
	}
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm);
}
void SetDriverMotor_pitch(int arah, uint16_t pwm ){
	switch (arah){
	case 0:
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);
		break;
	case 1:
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);
		break;
	}
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm);
}
//-----------parsing Data GPS---------------
void gps_parse(){
	if(UART4DataFlag == 1){
				  char *pointer;
				  char lat[20];
				  char lat_a;
				  char lon[20];
				  char lon_a;
				  int length = sizeof(data_gps);

				  memset(lat, '\0', 20);
				  memset(lon, '\0', 20) ;
				  pointer = strchr((char*)data_gps, '$');

				  do{
					  char *ptrstart;
					  char *ptrend;
					  if(strncmp(pointer, "$GNGGA" , 6) == 0){ //$GNGGA
						  ptrstart = (char*)memchr(pointer + 1, ',', length);
						  ptrstart = (char*)memchr(ptrstart + 1, ',', length);
						  ptrend = (char*)memchr(ptrstart + 1, ',', length);

					  } else if(strncmp(pointer, "$GNGLL", 6) == 0){ //$GNGLL
						  ptrstart = (char*)memchr(pointer + 1, ',', length);
						  ptrend = (char*)memchr(ptrstart + 1, ',', length);

					  } else if(strncmp(pointer, "$GNRMC", 6) == 0){
						  ptrstart = (char*)memchr(pointer + 1, ',', length);
						  ptrstart = (char*)memchr(ptrstart + 1, ',', length);
						  ptrstart = (char*)memchr(ptrstart + 1, ',', length);
						  ptrend = (char*)memchr(ptrstart + 1, ',', length);

					  } else {
						  pointer = strchr(pointer + 6, '$');
						  continue;
					  }

					  for(int i = 1; i < (ptrend - ptrstart); i++) lat[i - 1] = ptrstart[i];
					  lat_a = *(ptrend + 1);

					  ptrstart = (char*)memchr(ptrend + 1, ',', length);
					  ptrend = (char*)memchr(ptrstart + 1, ',', length);

					  for(int i = 1; i < (ptrend - ptrstart); i++) lon[i - 1] = ptrstart[i];
					  lon_a = *(ptrend + 1);
					  if(lon[0] != '\0' && lat[0] != '\0'){
						  //ukuranstring = sprintf((char*)buffer, "Lat: %s | %c\tLon: %s | %c\r\n", lat, lat_a, lon, lon_a);
						  //HAL_UART_Transmit(&huart1, buffer, ukuranstring, 100);
						  lat_gps = atof((char*)lat);
						  lon_gps = atof((char*)lon);
						  break;
					  }
					  pointer = strchr(pointer + 4, '$');
				  }
				  while(pointer != NULL);
				  UART4DataFlag = false;
			  }
}
void baca_input(){
	if(USART1DataFlag == true){
		uint8_t pitch[2];
		uint8_t yaw[3];
		   if(data_input[0] == '#'){ //#,30,090
			   pitch[0] = data_input[2];
			   pitch[1] = data_input[3];

			   yaw[0] = data_input[5];
			   yaw[1] = data_input[6];
			   yaw[2] = data_input[7];
			   ukuranstring = sprintf((char*)buffer, "Pitch: %s\tYaw: %s\r\n", pitch, yaw);
			   HAL_UART_Transmit(&huart1, buffer, ukuranstring, 100);

			   setPoint_pitch = atof ((char*)pitch);
			   setPoint_yaw = atof ((char*)yaw);
			   if (setPoint_pitch > 90) setPoint_pitch = 90;
			   if (setPoint_pitch <  0) setPoint_pitch = 0;
			   if (setPoint_yaw > 359) setPoint_yaw= 359;
			   if (setPoint_yaw <  1 ) setPoint_yaw = 1;
			   ukuranstring = sprintf((char*)buffer, "data INPUT = %s | %f | %f\r\n", data_input, setPoint_pitch, setPoint_yaw);
			   HAL_UART_Transmit(&huart1, buffer, ukuranstring, 10);
		   } else{
			   ukuranstring = sprintf((char*)buffer, "Data salah\r\n");
			   HAL_UART_Transmit(&huart1, buffer, ukuranstring, 100);
		   }
		   USART1DataFlag = false;
	}
}
//------------------------PID CONTROL-----------------------
void PID_PITCH(){
	errorPitch = setPoint_pitch - data_Pitch1;
	int maxPitchSpeed = 900;
	ukuranstring = sprintf((char*)buffer, "erorPitch = %f", errorPitch);
	HAL_UART_Transmit(&huart1, buffer, ukuranstring, 10);
	if (errorPitch < 0){
		dir_pitch = 0;   //TURUN
		maxPitchSpeed = 250;
	} else {
		dir_pitch = 1;  //NAIK
	}

	pitch_P  = KP_PITCH * errorPitch;
	pitch_I += KI_PITCH * errorPitch;
	pitch_D  = KD_PITCH * (errorPitch - last_errorPitch);

	if (-5.0f < errorPitch && errorPitch < 5.0f){
		if (pitch_I > 90)
			pitch_I = 90;
		else if (pitch_I < (-90)){
			pitch_I = -90;
			KI_pitch = 0.5;
		}
	} else {
		KI_pitch = 0.9;
	}
	Out_PIDPITCH = pitch_P + pitch_I + pitch_D;
	if (Out_PIDPITCH < 0)Out_PIDPITCH *= -1;
	if (Out_PIDPITCH > maxPitchSpeed) Out_PIDPITCH = maxPitchSpeed;
	if (errorPitch == 0) Out_PIDPITCH = 0;
		SetDriverMotor_pitch(dir_pitch, Out_PIDPITCH);
		last_errorPitch = errorPitch;
}
void PID_YAW(){
	errorYaw = setPoint_yaw - data_heading;
	if (data_heading >= 265 && setPoint_yaw <= 95){
		errorYaw += 359;
	} else if (data_heading <= 95 && setPoint_yaw >= 265){
		errorYaw += 359;
		errorYaw *= -1;
	}
	if (errorYaw < 0){
		dir_yaw = 1;
	} else {
		dir_yaw = 0;
	}

	yaw_P  = KP_YAW * errorYaw;
	yaw_I += KI_YAW * errorYaw;
	yaw_D  = KD_YAW * (errorYaw - last_errorYaw);

	if (-5.0f < errorYaw && errorYaw < 5.0f){
		if (yaw_I > 120) yaw_I = -120;
		else if (-2.0f < errorYaw && errorYaw < 2.0f) KI_yaw = 0.2;
		else (KI_yaw = 0.4);
	} else {
		KI_yaw = 0.4;
	}
	Out_PIDYAW = yaw_P + yaw_I + yaw_D;

	if (Out_PIDYAW < 0) Out_PIDYAW *= -1;
	if (Out_PIDYAW > 500) Out_PIDYAW = 500;

	if (-1.5f < errorYaw && errorYaw < 1.5f) Out_PIDYAW = 0;
	SetDriverMotor_yaw(dir_yaw, Out_PIDYAW);
	last_errorYaw = errorYaw;
}
void input_data_wahana(){
	if(USART3DataFlag == true){
		ukuranstring = sprintf((char*)buffer, "Wahana input = %s\r\n", data_wahana);
		HAL_UART_Transmit(&huart3, buffer, ukuranstring, 10);
		for (cek_w = 0; cek_w < sizeof(data_wahana); cek_w++){
			if(('t' == data_wahana[cek_w]) && (flag_w == 0)){
				if(flag_w == 0){
					a_w = cek_w + 1;
					flag_w = 9;
				}
				break;
			}
		}
		if (flag_w == 9){
			for(cek_w = a_w; cek_w < sizeof(data_wahana); cek_w++){
				if (data_wahana[cek_w] == ','){
					bit_w ++;
					ke_w = 0;
					continue;
				} else {
					if(bit_w == 1){
						data_mode_w[ke_w] = data_wahana[cek_w];
					} else if (bit_w == 2){
						data_yaw_w[ke_w] = data_wahana[cek_w];
					} else if (bit_w == 3){
						data_pitch_w[ke_w] = data_wahana[cek_w];
					} else if (bit_w == 4){
						data_roll_w[ke_w] = data_wahana[cek_w];
					} else if (bit_w == 5){
						data_angin_w[ke_w] = data_wahana[cek_w];
					} else if (bit_w == 6){
						data_alti_w[ke_w] = data_wahana[cek_w];
					} else if (bit_w == 7){
						data_lat_w[ke_w] = data_wahana[cek_w];
					} else if (bit_w == 8){
						data_long_w[ke_w] = data_wahana[cek_w];
					} else if (bit_w == 9){
						data_bat_w[ke_w] = data_wahana[cek_w];
					} else if (bit_w > 9){
						bit_w = 0;
						flag_w = 0;
						out_w = 1;
						break;
					} ke_w++;
				}
			}
		}
		mode_w  = atoi(data_mode_w);
		yaw_w   = atof(data_yaw_w);
		pitch_w = atof(data_pitch_w);
		roll_w  = atof(data_roll_w);
		angin_w = atof(data_angin_w);
		alti_w  = atof(data_alti_w);
		lat_w	= atof(data_lat_w);
		long_w	= atof(data_long_w);
		bat_w	= atof(data_bat_w);
		USART3DataFlag = false;
	}
}
void kirim_GCS(){
	ukuranstring = sprintf((char*)buffer, "%d,%2.f,%2.f,%2.f,%2.f,%2.f,%2.f,%f,%2.f,%f,%f,%f,%f\r\n", mode_w, yaw_w,
			pitch_w, roll_w, angin_w, alti_w, lat_w, long_w, bat_w, data_Pitch1, data_heading, lat_gps, lon_gps);
	//ukuranstring = sprintf((char*)buffer, "POT:%f | HMC:%f | P:%d | Y:%d\r\n", data_Pitch1, data_heading, Out_PIDPITCH, Out_PIDYAW);
	HAL_UART_Transmit(&huart1, buffer, ukuranstring, 30);
}
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
  MX_DMA_Init();
  MX_I2C3_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_UART4_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  I2Cdev_init(&hi2c3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HMC5883LInit();
  bacaheading();
  HAL_ADC_Start(&hadc1);

  //interupt usart4
  __HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);
  if(HAL_UART_Receive_DMA(&huart4, dma_rx4_buf, DMA_BUF_SIZE) != HAL_OK){
	  Error_Handler();
  }

  __HAL_DMA_DISABLE_IT(huart4.hdmarx, DMA_IT_HT);


  //interupt uart1
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
  if(HAL_UART_Receive_DMA(&huart1, dma_rx1_buf, DMA_BUF_SIZE) != HAL_OK){
	  Error_Handler();
  }

  __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);


  //interupt usart2
  __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
  if(HAL_UART_Receive_DMA(&huart3, dma_rx3_buf, DMA_BUF_SIZE) != HAL_OK){
	  Error_Handler();
  }

  __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  ukuranstring = sprintf((char*)buffer, "=========SISTEM TRACKER DICOBA======\r\n");
  HAL_UART_Transmit(&huart1, buffer, ukuranstring, 5);
  while (1)
  {
	  input_data_wahana();
	  gps_parse();
	  GetHMC5893L();
	  GetPOTENSIO();
	  cek_heading();
	  baca_input();     //send : "#,{arahVertikal},{arahHorizontal}\n
	  PID_PITCH();
	  PID_YAW();
	  kirim_GCS();
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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  hadc1.Init.ContinuousConvMode = ENABLE;
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
  sConfig.Channel = ADC_CHANNEL_14;
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
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 168-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000 - 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE9 PE10 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == UART4){
	    uint16_t i, pos, start, length;
	    uint16_t currCNDTR = __HAL_DMA_GET_COUNTER(huart->hdmarx);

	    if(dma_uart4_rx.flag && currCNDTR == DMA_BUF_SIZE)
	    {
	        dma_uart4_rx.flag = 0;
	        return;
	    }
	    start = (dma_uart4_rx.prevCNDTR < DMA_BUF_SIZE) ? (DMA_BUF_SIZE - dma_uart4_rx.prevCNDTR) : 0;
	    if(dma_uart4_rx.flag)    /* Timeout event */
	    {
	        length = (dma_uart4_rx.prevCNDTR < DMA_BUF_SIZE) ? (dma_uart4_rx.prevCNDTR - currCNDTR) : (DMA_BUF_SIZE - currCNDTR);
	        dma_uart4_rx.prevCNDTR = currCNDTR;
	        dma_uart4_rx.flag = 0;
	    }
	    else                /* DMA Rx Complete event */
	    {
	        length = DMA_BUF_SIZE - start;
	        dma_uart4_rx.prevCNDTR = DMA_BUF_SIZE;
	    }
	    for(i=0,pos=start; i<length; ++i,++pos)
	    {
	        data_gps[i] = dma_rx4_buf[pos];
	    }
	    UART4DataFlag = true;
	}
	else if(huart->Instance == USART1){
		uint16_t a, pos, mulai, kanan;
		uint16_t currCNDTR = __HAL_DMA_GET_COUNTER(huart->hdmarx);
		if (dma_uart1_rx.flag && currCNDTR == DMA_BUF_SIZE){
			dma_uart1_rx.flag = 0;
			return;
		}
		mulai = (dma_uart1_rx.prevCNDTR < DMA_BUF_SIZE) ? (DMA_BUF_SIZE - dma_uart1_rx.prevCNDTR) : 0;

		if (dma_uart1_rx.flag) { // time out
			kanan = (dma_uart1_rx.prevCNDTR < DMA_BUF_SIZE) ? (dma_uart1_rx.prevCNDTR - currCNDTR) : (DMA_BUF_SIZE - currCNDTR);
			dma_uart1_rx.prevCNDTR = currCNDTR;
			dma_uart1_rx.flag = 0;
		} else {				//complete event
			kanan = DMA_BUF_SIZE - mulai;
			dma_uart1_rx.prevCNDTR = DMA_BUF_SIZE;
		}
		for (a = 0,pos = mulai; a < kanan; ++a,++pos){
			data_input[a] = dma_rx1_buf[pos];
		}
		USART1DataFlag = true;
	}
	else if (huart->Instance == USART3){
		uint16_t c, pos, mulai, kiri;
		uint16_t currCNDTR = __HAL_DMA_GET_COUNTER(huart->hdmarx);
		if(dma_uart3_rx.flag && currCNDTR == DMA_BUF_SIZE){
			dma_uart3_rx.flag = 0;
			return;
		}
		mulai = (dma_uart3_rx.prevCNDTR < DMA_BUF_SIZE) ? (DMA_BUF_SIZE - dma_uart3_rx.prevCNDTR) : 0;

		if (dma_uart3_rx.flag){
			kiri = (dma_uart3_rx.prevCNDTR < DMA_BUF_SIZE) ? (dma_uart3_rx.prevCNDTR - currCNDTR) : (DMA_BUF_SIZE - currCNDTR);
			dma_uart3_rx.prevCNDTR = currCNDTR;
			dma_uart3_rx.flag = 0;
		} else {
			kiri = DMA_BUF_SIZE - mulai;
			dma_uart3_rx.prevCNDTR = DMA_BUF_SIZE;
		}
		for(c = 0,pos = mulai; c < kiri; ++c, ++pos){
			data_wahana[c] = dma_rx3_buf[pos];
		}
		USART3DataFlag = true;
	}
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
