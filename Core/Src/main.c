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
#define KP_YAW 1.2
#define KI_YAW 0
#define KD_YAW 50
		/*PID PITCH*/
#define KP_PITCH 5.0
#define KI_PITCH 0
#define KD_PITCH 15
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
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
//---------VAR GPS ACSES------
char gps[200];
int cek2,a2,b2,flag2,nomor_parsing2=0,ke2=0;
float data_lat, data_longi;
int data_time;
char lat[15],lat_char[15],longi[15],longi_char[15],valid[15],time[15];
float latitude,longitude,latitude_zero,longitude_zero,selisih_gps_lat,selisih_gps_long;
float currentLat, currentLong;
double currentLat1, currentLong1;
DMA_Event_t dma_uart_rx = {0, 0, DMA_BUF_SIZE};
uint8_t dma_rx_buf[DMA_BUF_SIZE];
uint8_t data[DMA_BUF_SIZE] = {'\0'};
bool USART3DataFlag = false;
//----------VAR DATA INPUT----
DMA_Event_t dma_uart1_rx = {0, 0, DMA_BUF_SIZE};
uint8_t dma_rx1_buf[DMA_BUF_SIZE];
uint8_t data_input[DMA_BUF_SIZE] = {'\0'};
bool USART1DataFlag = false;
char setPoint_pitchAct[10], setPoint_yawAct[10];
float setPoint_pitch = 0, setPoint_yaw=1;
int cek_in, flag_in, a_in, nomor_parsing_in, ke_in = 0, metu_in, mode_in = 0;
// --------drivermotor---
int arah;
uint16_t pwm;
//union Floating data_heading, dataPitch1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float baca_potensio(){
	HAL_ADC_Start(&hadc1);
	while (__HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_EOC) != SET);
		  HAL_ADC_Stop(&hadc1);
		  data_potensio = HAL_ADC_GetValue(&hadc1);
		  analog_potensio = (float)data_potensio;
		  if (analog_potensio > 4000) analog_potensio = 4000;
		  return analog_potensio*(180.0f/4000.0f);
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
	data_heading = bacaheading();
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
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);
		break;
	case 1:
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);
		break;
	}
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm);
}
//-----------parsing Data GPS---------------
void ParsedataGPS_GNGLL(void){
	if (flag2 == 2){
		for (cek2 = a2; cek2 < sizeof(data); cek2++){
			if(data[cek2] == ','){
				nomor_parsing2++;
				ke2 = 0;
				continue;
			}
			else {
				if (nomor_parsing2 == 1){
					lat[ke2] = data[cek2];
				}
				else if(nomor_parsing2 == 2){
					lat_char[ke2] = data[cek2];
				}
				else if (nomor_parsing2 == 3){
					longi[ke2] = data[cek2];
				}
				else if (nomor_parsing2 == 4){
					longi_char[ke2] = data[cek2];
				}
				else if (nomor_parsing2 == 5){
					time[ke2] = data[cek2];
				}
				else if (nomor_parsing2 == 6){
					valid[ke2] = data[cek2];
				}
				else if (nomor_parsing2 > 6){
					nomor_parsing2 = 0;
					flag2 = 0;
					break;
				}
				ke2++;
			}
		}
	}
	data_time = atoi(time);
	data_lat  = atof(lat);
	data_longi = atof (longi);
}
void ParsedataGPS_GNGGA(void){
	if (flag2 == 2){
		for(cek2 = a2; cek2 < sizeof(data); cek2++){
			if(data[cek2] == ','){
				nomor_parsing2++;
				ke2 = 0;
				continue;
			}
			else{
				if (nomor_parsing2 == 1){
					time[ke2] = data[ke2];
				}
				else if(nomor_parsing2 == 2){
					lat[ke2] = data[ke2];
				}
				else if(nomor_parsing2 == 3){
					lat_char[ke2] = data[ke2];
				}
				else if(nomor_parsing2 == 4){
					longi[ke2] = data[ke2];
				}
				else if(nomor_parsing2 == 5){
					longi_char[ke2] = data[ke2];
				}
				else if(nomor_parsing2 == 6){
					valid[ke2] = data[ke2];
				}
				else if(nomor_parsing2 > 6){
					nomor_parsing2 = 0;
					flag2 = 0;
					break;
				}
				ke2++;
			}
		}
	}
	data_time = atoi(time);
	data_lat = atof(lat);
	data_longi = atof(longi);
}
void ParsedataGPS_GNRMC(void){
	if (flag2 == 2){
		for (cek2 = a2; cek2 < sizeof(data); cek2++){
			if (data[cek2] == ','){
				nomor_parsing2++;
				ke2 = 0;
				continue;
			}
			else {
				if (nomor_parsing2 == 1){
					time[ke2] = data[cek2];
				}
				else if (nomor_parsing2 == 2){
					valid[ke2] = data[ke2];
				}
				else if (nomor_parsing2 == 3){
					lat[ke2] = data[ke2];
				}
				else if (nomor_parsing2 == 4){
					lat_char[ke2] = data[ke2];
				}
				else if (nomor_parsing2 == 5){
					longi[ke2] = data[ke2];
				}
				else if (nomor_parsing2 == 6){
					longi_char[ke2] = data[ke2];
				}
				else if (nomor_parsing2 > 6){
					nomor_parsing2 = 0;
					flag2 = 0;
					break;
				}
				ke2++;
			}
		}
	}
	data_time = atoi(time);
	data_lat = atoi(lat);
	data_longi = atof(longi);
}
void GPS_Validasi(void){
	for (cek2 = 0; cek2 < sizeof(gps); cek2++){
		if(('$' == gps[cek2]) && ('G' == gps[cek2+1]) && ('N' == gps[cek2+2]) && ('G' == gps[cek2+3]) && ('L' == gps[cek2+4]) && ('L' == gps[cek2+5]) && (flag2 == 0))
		{
			if(flag2 == 0) {
				a2 = cek2+6;
				flag2 = 2;
			}
			ParsedataGPS_GNGLL();
		}
		else if (('$' == gps[cek2]) && ('G' == gps[cek2+1]) && ('N' == gps[cek2+2]) && ('G' == gps[cek2+3]) && ('G' == gps[cek2+4]) && ('A' == gps[cek2+5]) && (flag2 == 0))
		{
			if(flag2 == 0) {
				a2 = cek2+6;
				flag2 = 2;
			}
			ParsedataGPS_GNGGA();
		}
		else if(('$' == gps[cek2]) && ('G' == gps[cek2+1]) && ('N' == gps[cek2+2]) && ('R' == gps[cek2+3]) && ('M' == gps[cek2+4]) && ('C' == gps[cek2+5]) && (flag2 == 0))
		{
				if(flag2 == 0) {
				a2 = cek2+6;
				flag2 = 2;
		}
				ParsedataGPS_GNRMC();
	}
  }
}
void pindah_data_gps(){
	  currentLat = dms_dd(data_lat, lat_char[0]);
	  currentLong = dms_dd(data_longi, longi_char[0]);
	  currentLat1 = currentLat;
	  currentLong1 = currentLong;
}
void dms_dd(float in_coords, char angin){
	float f = in_coords;
	char arah = angin;
	int firstdig = ((int)f) / 100;
	float nextdig = f-(float)(firstdig*100);
	if ('W' == arah){
		float final = (float)((firstdig + (nextdig / 60.0))* -1.0);
		return final;
	}
	if ('N' == arah){
		float final = (float)((firstdig + (nextdig / 60.0))* -1.0);
		return final;
	}
	if ('E' == arah){
		float final = (float)((firstdig + (nextdig / 60.0))* -1.0);
		return final;
	}
	if ('S' == arah){
		float final = (float)((firstdig + (nextdig / 60.0))* -1.0);
		return final;
	}
}
void baca_input(){
	if(USART1DataFlag == true){
/*	   if(data_input[0] == '#'){
		   if (data_input[2] == '0'){
			   arah = 0;
		   }else if(data_input[2] == '1'){
			   arah = 1;
		   }
		   char pwmstr[4];
		   memcpy(pwmstr, &data_input[3], 4);
		   pwm = atoi(pwmstr);
		   if (data_input[1] == 'y'){
				  SetDriverMotor_yaw(arah, pwm);
		   }else if(data_input[1] == 'p'){
				  SetDriverMotor_pitch(arah, pwm);
		   }
	   }*/
		for (cek_in = 0; cek_in<sizeof(data_input); cek_in++){
			if ('#' == data_input[cek_in] && flag_in == 0){
				if(flag_in == 0){
					a_in = cek_in + 1;
					flag_in = 2;
				}
				break;
			}
		}
		if (flag_in == 2){
			for (cek_in = a_in; cek_in < sizeof(data_input); cek_in++){
				if (data_input[cek_in] == ','){
					nomor_parsing_in++;
					ke_in = 0;
					continue;
				}
				else {
					if (nomor_parsing_in == 1){
						setPoint_pitchAct[ke_in] = data_input[cek_in];
					}
					if (nomor_parsing_in == 2){
						setPoint_yawAct[ke_in] = data_input[cek_in];
					}
					else if (nomor_parsing_in > 2){
						nomor_parsing_in =0;
						flag_in = 0;
						metu_in = 1;
						break;
					}
					ke_in++;
				}
			}
		}
		setPoint_pitch = atof (setPoint_pitchAct);
		setPoint_yaw = atof (setPoint_yawAct);
		if (setPoint_pitch > 90) setPoint_pitch = 90;
		if (setPoint_pitch <  0) setPoint_pitch = 0;
		if (setPoint_yaw > 359) setPoint_yaw= 359;
		if (setPoint_yaw <  1 ) setPoint_yaw = 1;
		ukuranstring = sprintf((char*)buffer, "data INPUT = %s | %f | %f\r\n", data_input, setPoint_pitch, setPoint_yaw);
		HAL_UART_Transmit(&huart1, buffer, ukuranstring, 10);
	   USART1DataFlag = false;
	}
}
//------------------------PID CONTROL-----------------------
void PID_PITCH(){
	errorPitch = setPoint_pitch - data_Pitch1;
	int maxPitchSpeed = 200;
	if (errorPitch < 0){
		dir_pitch = 0;   //TURUN
		maxPitchSpeed = 90;
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
	if (Out_PIDYAW > 250) Out_PIDYAW = 250;

	if (-1.5f < errorYaw && errorYaw < 1.5f) Out_PIDYAW = 0;
	SetDriverMotor_yaw(dir_yaw, Out_PIDYAW);
	last_errorYaw = errorYaw;
}
void kirim_GCS(){

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
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  I2Cdev_init(&hi2c3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HMC5883LInit();
  bacaheading();
  HAL_ADC_Start(&hadc1);
  __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);

  if(HAL_UART_Receive_DMA(&huart3, dma_rx_buf, DMA_BUF_SIZE) != HAL_OK){
	  Error_Handler();
  }

  __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);

  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);

  if(HAL_UART_Receive_DMA(&huart1, dma_rx1_buf, DMA_BUF_SIZE) != HAL_OK){
	  Error_Handler();
  }

  __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  ukuranstring = sprintf((char*)buffer, "=========SISTEM DICOBA======\r\n");
  HAL_UART_Transmit(&huart1, buffer, ukuranstring, 5);
  while (1)
  {
	  /*send : "#,{arahVertikal},{arahHorizontal}\n	   */
	  /*======DATA GPS ======*/
/*	  if(USART3DataFlag == true){
		  strcpy(gps, data);
		  GPS_Validasi();
		  pindah_data_gps();
			  ukuranstring = sprintf((char*)buffer, "data GPS = %s", data);
			  HAL_UART_Transmit(&huart1, buffer, ukuranstring, 50);
			  ukuranstring = sprintf ((char*)buffer, "GPS = %f  | %f == %f | %f\r\n", currentLat1, currentLong1, data_lat, data_longi);
		  	  HAL_UART_Transmit(&huart1, buffer, ukuranstring, 10);
			  USART3DataFlag = false;
	  }*/
	  /*=======DATA HMC5893L & POTENSIO====*/
	  if(HMC5883L_getReadyStatus()){
		  GetHMC5893L();
		  ukuranstring = sprintf((char*)buffer, "X : %d   Y : %d   Z : %d || X = %2.f degree= %2.f\r\n", x_heading, y_heading, z_heading, heading, data_heading);
		  //HAL_UART_Transmit(&huart1, buffer, ukuranstring, 30);
	  }
	  data_Pitch1 = baca_potensio();
	  ukuranstring = sprintf((char*)buffer, "POTENSIO = %f | %f\r\n", data_Pitch1, analog_potensio);
	  HAL_UART_Transmit(&huart1, buffer, ukuranstring, 10);
	  cek_heading();
	  baca_input();
	  HAL_Delay(100);
	  PID_YAW();
	  PID_PITCH();
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
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
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
	if(huart->Instance == USART3){
	    uint16_t i, pos, start, length;
	    uint16_t currCNDTR = __HAL_DMA_GET_COUNTER(huart->hdmarx);

	    /* Ignore IDLE Timeout when the received characters exactly filled up the DMA buffer and DMA Rx Complete IT is generated, but there is no new character during timeout */
	    if(dma_uart_rx.flag && currCNDTR == DMA_BUF_SIZE)
	    {
	        dma_uart_rx.flag = 0;
	        return;
	    }

	    /* Determine start position in DMA buffer based on previous CNDTR value */
	    start = (dma_uart_rx.prevCNDTR < DMA_BUF_SIZE) ? (DMA_BUF_SIZE - dma_uart_rx.prevCNDTR) : 0;

	    if(dma_uart_rx.flag)    /* Timeout event */
	    {
	        /* Determine new data length based on previous DMA_CNDTR value:
	         *  If previous CNDTR is less than DMA buffer size: there is old data in DMA buffer (from previous timeout) that has to be ignored.
	         *  If CNDTR == DMA buffer size: entire buffer content is new and has to be processed.
	        */
	        length = (dma_uart_rx.prevCNDTR < DMA_BUF_SIZE) ? (dma_uart_rx.prevCNDTR - currCNDTR) : (DMA_BUF_SIZE - currCNDTR);
	        dma_uart_rx.prevCNDTR = currCNDTR;
	        dma_uart_rx.flag = 0;
	    }
	    else                /* DMA Rx Complete event */
	    {
	        length = DMA_BUF_SIZE - start;
	        dma_uart_rx.prevCNDTR = DMA_BUF_SIZE;
	    }

	    /* Copy and Process new data */
	    for(i=0,pos=start; i<length; ++i,++pos)
	    {
	        data[i] = dma_rx_buf[pos];
	    }
	    USART3DataFlag = true;
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
