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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lwgps/lwgps.h"
#include "LSM6DSLTR.h"
#include "./BME280/bme280.h"
#include "W25Qxx.h"
#include <math.h>
#include "FIR_FILTER.h"
#include "stdio.h"
#include "string.h"
#include "kalman.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LORA_TX_BUFFER_SIZE 70
#define EGU_RX_BUFFER_SIZE 34
#define RX_BUFFER_SIZE 128
#define DEVICE_ID 2


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
uint8_t EGU_RX_BUFFER[EGU_RX_BUFFER_SIZE];
uint8_t rx_data_EGU=0;
uint8_t rx_index_EGU=0;
const uint8_t EGU_durum_sorgusu[5]={0x54,0x52,0x35,0x0D,0x0A};
const uint8_t EGU_motor_atesleme[5]={0x54,0x52,0x32,0x0D,0x0A};

float adc_pil_val=0;
float adc =0;
uint8_t counter=0;
uint8_t adc_flag=0;

uint8_t rx_buffer[RX_BUFFER_SIZE];
uint8_t rx_index=0;
uint8_t rx_data=0;

uint8_t kademe_flag=0;
uint8_t timer_start_flag =0;
uint8_t altitude_rampa_control =0;

uint32_t page =0;
uint8_t offset =1;
uint8_t flash_flag =0;
uint8_t flash_timer =0;
uint8_t timer_200ms_flag =0;
int buzzer_timer=0;


float zaman=0 , zaman2=0, altitude_max=0;

uint8_t v4_battery=0;
uint8_t v4_mod=0;
uint8_t magnetic_switch=0;
uint8_t BUTTON_STATE=0;


uint8_t writeData[50] = {0,1,1,1};
uint8_t readData[50] = {0};
uint8_t i=0;
uint8_t i_flag=0;
uint8_t flash_altitude[256]={0};
uint8_t flash_accX[256]={0};
uint8_t flash_accZ[256]={0};
uint8_t flash_accY[256]={0};
uint8_t flash_gyroX[256]={0};
uint8_t flash_gyroY[256]={0};
uint8_t flash_gyroZ[256]={0};

uint8_t loratx[LORA_TX_BUFFER_SIZE];
uint8_t test[256];
uint8_t lora_flag=0;
uint8_t sensor_flag=0;
uint8_t egu_durum_flag=0;
uint8_t egu_ok=0;

float temperature=0;
float humidity=0;
float altitude=0;
float offset_altitude=0;
float pressure=0;
float alt=0;
float P0 = 1013.25;
float prev_alt=0;
float speed=0, speed_max=0;;
float altitude_kalman;

float real_pitch, real_roll , toplam_pitch,toplam_roll , x_max;
uint8_t  sensor_counter =0;

typedef union{
  float fVal;
  unsigned char array[4];
}float2unit8;

KalmanFilter kf;

enum ZORLU2024
{
	RAMPA,UCUS_BASLADI,KADEMEAYRILDIMI,AYRILDI,APOGEE,SUSTAINER_ANA,FINISH
};
enum ZORLU2024 SUSTAINER;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_UART4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */
int16_t I2C_Testsensor(void);
int8_t user_i2c_read(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len);
int8_t user_i2c_write(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len);
void user_delay_ms(uint32_t period);
float BME280_Get_Altitude(void);
void E220_CONFIG(uint8_t ADDH, uint8_t ADDL, uint8_t CHN, uint8_t MODE);
void union_converter(void);
void EGU_Buff_Load(void);
void Buzzer(int how_many , uint32_t how_long);
void Altitude_Offset();
void Flash_toplu_temizleme(int page);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
lwgps_t gps;
LSM6DSLTR Lsm_Sensor;

FIRFilter accy;
FIRFilter accz;
FIRFilter accx;
FIRFilter IMU_GYROX;
FIRFilter IMU_GYROY;
FIRFilter IMU_GYROZ;

struct bme280_dev dev;
struct bme280_data comp_data;
int8_t rslt=0;
float2unit8 conv;


float map( long A , long B , long C , long D, long E)
{
	return ( A - B) * (E - D ) / (C - B )+ D ;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart==&huart2){
	if(rx_data != '\n' && rx_index < RX_BUFFER_SIZE) {
		rx_buffer[rx_index++] = rx_data;
	} else {
		lwgps_process(&gps, rx_buffer, rx_index+1);
		rx_index = 0;
		rx_data = 0;
	}
	HAL_UART_Receive_IT(&huart2, &rx_data, 1);
	}


	if(huart == &huart6){
		if(rx_data_EGU != '\n' && rx_index_EGU <34){//sol taraf silinebilir
			EGU_RX_BUFFER[rx_index_EGU++]=rx_data_EGU;

		}
		else
		{
			rx_data_EGU=0;
			rx_index_EGU=0;

		}
	HAL_UART_Receive_IT(&huart6, &rx_data_EGU, 1);
		}
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	if(htim==&htim11){ // 1 sn
   lora_flag=1;

	}
	if(htim==&htim10){ //30ms
	sensor_flag=1;

	}
	if(htim==&htim6){ // 200 ms timer
	timer_200ms_flag = 1;
	egu_durum_flag=1;

	counter++;
		if(counter == 15)
		{
			adc_flag=1;
			counter =0;
			HAL_ADC_Start_IT(&hadc1);
		}


	}


}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(hadc->Instance == ADC1 )
	{
		adc= HAL_ADC_GetValue(&hadc1);


		adc_flag = 0;
	}
}
/**********************************************/

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
  MX_UART4_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_FATFS_Init();
  MX_TIM11_Init();
  MX_TIM10_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);

  ///KURTARMA PORTLARI KAPALI EMIN OL
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);//A
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);//B
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);//C
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);//D

  HAL_UART_Receive_IT(&huart2,&rx_data,1);
  HAL_UART_Receive_IT(&huart6, &rx_data_EGU, 1);
  HAL_TIM_Base_Start_IT(&htim11);
  HAL_TIM_Base_Start_IT(&htim10);
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Stop_IT(&htim7);

  MAFilter_Init(&accx);
  FIRFilter_Init(&IMU_GYROY);
  FIRFilter_Init(&IMU_GYROX);
  FIRFilter_Init(&IMU_GYROZ);


  lwgps_init(&gps);
  LSM6DSLTR_Init();
  E220_CONFIG(0x6,0x4A,0X10,1);
  HAL_ADC_Start_IT(&hadc1);

  dev.dev_id = BME280_I2C_ADDR_PRIM;
  dev.intf = BME280_I2C_INTF;
  dev.read = user_i2c_read;
  dev.write = user_i2c_write;
  dev.delay_ms = user_delay_ms;

  rslt = bme280_init(&dev);

  dev.settings.osr_h = BME280_OVERSAMPLING_1X;
  dev.settings.osr_p = BME280_OVERSAMPLING_4X;
  dev.settings.osr_t = BME280_OVERSAMPLING_2X;
  dev.settings.filter = BME280_FILTER_COEFF_16;
  rslt = bme280_set_sensor_settings(BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL, &dev);

  ////ALTITUDE OFFSET
   Altitude_Offset();
   KalmanFilter_Init(&kf, 0.005, 0.1, 0.0); // Adjust Q=0.01 idi and R based on your system characteristics



 //  W25Q_Read(1, 0, sizeof(flash_accX), flash_accX);
   Buzzer(10, 100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

/********************* Sensor Ölçüm **************************************************/
	if(sensor_flag==1)
	{
		 sensor_flag=0;
		 prev_alt=altitude_kalman;
		 rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, &dev);
		/* �?��?�터 취�? */
		rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);

		if(rslt == BME280_OK)
		{
		  temperature = comp_data.temperature/100.00;
		  humidity = comp_data.humidity;
		  pressure = comp_data.pressure;
		  altitude=BME280_Get_Altitude()-offset_altitude;
		  altitude_kalman= KalmanFilter_Update(&kf, altitude);
		  speed=(altitude_kalman-prev_alt)*20;
    	}

		 LSM6DSLTR_Read_Accel_Data(&Lsm_Sensor);
		 calculate_roll_pitch(&Lsm_Sensor);
		 LSM6DSLTR_Read_Gyro_Data(&Lsm_Sensor);
		 update_angles(&Lsm_Sensor);

		 Lsm_Sensor.Accel_X=FIRFilter_Update(&accx,  Lsm_Sensor.Accel_X);
		 Lsm_Sensor.Gyro_X=FIRFilter_Update(&IMU_GYROX,  Lsm_Sensor.Gyro_X);
		 Lsm_Sensor.Gyro_Y=FIRFilter_Update(&IMU_GYROY, Lsm_Sensor.Gyro_Y);
		 Lsm_Sensor.Gyro_Z=FIRFilter_Update(&IMU_GYROZ, Lsm_Sensor.Gyro_Z);

		 toplam_pitch+= Lsm_Sensor.Pitch;
		 toplam_roll+= Lsm_Sensor.Roll;

		 sensor_counter++;
		 if(sensor_counter == 6)
		 {
			 real_pitch = toplam_pitch/6;
			 real_roll = toplam_roll/6;
			 toplam_roll=0;
			 toplam_pitch=0;
			 sensor_counter =0;
		 }

		 magnetic_switch=HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
		 BUTTON_STATE=HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9);


	}

/********************Lora Ayar ve Gönderme*****************************************************/
	if(lora_flag==1)
	{
	    lora_flag=0;

		loratx[0]=0x8;
		loratx[1]=0x2A;
		loratx[2]=0x10;
		loratx[3]=DEVICE_ID;
		loratx[4]=gps.sats_in_view;

		union_converter();

		loratx[49]=(uint8_t)adc_pil_val;
		loratx[50]=v4_mod;
		loratx[51]=magnetic_switch;

		////////EGU PART
		EGU_Buff_Load();

		HAL_UART_Transmit_IT(&huart3,loratx,sizeof(loratx));

	}


/************ EGÜ durum sorgusu gönder **************************************************/
	if(egu_durum_flag==1)
	{
		egu_durum_flag=0;
		HAL_UART_Transmit(&huart6, EGU_durum_sorgusu, 5, 1000);
	}


/**************************Algoritma****************************************************/
		  switch(SUSTAINER){

		  case RAMPA:
				v4_mod=1;
			  //RAMPA MODU ROKET RAMPADA EGÜ SWİTCHLERİ VE ALT KADEME HABERLE�?ME KONTROL ET

				if(Lsm_Sensor.Accel_X > 5 )
				  {
					SUSTAINER=UCUS_BASLADI;
//					Buzzer(6, 300);
				  }

			  break;

		  case UCUS_BASLADI:
				v4_mod=2;
				// FLASH MEMORYE KAYDETMEYE BASLA

				SUSTAINER=KADEMEAYRILDIMI;

			 break;

		  case KADEMEAYRILDIMI:
				v4_mod=3;
					  //ALT KADEMEDEN GELEN SİNYALE VE EGÜ SWİTCHLERİNE BAK
					  //BELLİ BİR SÜRE VE İRTİFA BOYUNCA AYRILMA OLMAZSA APOGEE YA GEÇ
				if(timer_start_flag == 0){

					__HAL_TIM_SET_COUNTER(&htim7 , 0);
					HAL_TIM_Base_Start(&htim7);
					timer_start_flag =1;
				}

				if((magnetic_switch==0 && EGU_RX_BUFFER[30]==0) && TIM7->CNT >= 45000 && altitude_rampa_control == 1)
				{
				  SUSTAINER=AYRILDI;

//				  Buzzer(10, 100);
				}

			 break;

		  case AYRILDI:
				v4_mod=4;
					  //MOTOR ATE�?LEME TALEBİ GÖNDER MEGU YE MESAJ AT

				HAL_UART_Transmit(&huart6, EGU_motor_atesleme, 5, 1000);
				SUSTAINER=APOGEE;

			 break;

		  case APOGEE:
				v4_mod=5;
					  //AYRILMA GERÇEKLE�?MESE BİLE APOGEE İLE ROKETİ KURTAR *ucus basladı kısmına timer kuracam ona göre ayrıldımıdan APOGEEya geçecek


				if((real_pitch <= 32) && speed <= 2 && altitude < altitude_max )
				{
					// GPIO

					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, SET);



					SUSTAINER=SUSTAINER_ANA;
					altitude_rampa_control =0;
				}


			 break;

		  case SUSTAINER_ANA:
				v4_mod=6;
					  //AYRILDI VEYA APOGEEDAN GELEN APOGEE BİLGİSİNDEN SONRA İSTENEN İRTİFADA ANA PARA�?ÜT AÇ

				if(altitude <= 500 && speed < 0  && altitude_rampa_control == 0 )
				{
					// GPIO
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, RESET);

					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, SET);

					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, RESET);

					SUSTAINER=FINISH;
					flash_flag=1;
				}

			  break;

		  case FINISH:
				v4_mod=7;
					  //KURTARMA GERÇEKLE�?Tİ VERİ KAYDETMEYİ BIRAK VE BUZZERI AÇ


			  break;
		  }


/**************************************************************************************/
		  if(altitude >30 && SUSTAINER <3)
		  {
			  altitude_rampa_control =1;
		  }
/*************************************************************************************/
		  if(altitude>altitude_max) altitude_max = altitude;

		  if(speed>speed_max) speed_max = speed;

		  if( Lsm_Sensor.Accel_X> x_max) x_max =  Lsm_Sensor.Accel_X;
/**************************************************************************************/
		  if(adc_flag ==1)
		  {
			  if(adc > 2476) adc = 2234;
			  if(adc < 1755) adc = 1755;
			  // 6V = 1755 adc val 1,41V
			  // 8.4V = 2476 adc val 1,99V 0,58V
			  adc_pil_val=(float)( ( ( (adc/4095)*3.3)-1.41) / (1.99-1.41) ) *100 ; // pil conv
		  }
/**********************Flash Kayıt*********************************************************/
//	if(flash_flag == 1 && SUSTAINER >=6 ) //
//	{
//
//		W25Q_Write_Page(page, 0, sizeof(flash_accX), flash_accX);
//		page++;
//		W25Q_Write_Page(page, 0, sizeof(flash_accX), flash_accY);
//		page++;
//		W25Q_Write_Page(page, 0, sizeof(flash_accX), flash_accZ);
//		page++;
//		W25Q_Write_Page(page, 0, sizeof(flash_accX), flash_gyroX);
//		page++;
//		W25Q_Write_Page(page, 0, sizeof(flash_accX), flash_gyroY);
//		page++;
//		W25Q_Write_Page(page, 0, sizeof(flash_accX), flash_gyroZ);
//
//		flash_flag=0;
//	}
//
	if(timer_200ms_flag == 1 && i_flag ==0 && SUSTAINER >=1)
	{
		if(i >= 252) {
			i_flag=1;
		}

		conv.fVal=Lsm_Sensor.Accel_X;

		flash_accX[i] = conv.array[0];
		flash_accX[i+1] = conv.array[1];
		flash_accX[i+2] = conv.array[2];
		flash_accX[i+3] = conv.array[3];

		conv.fVal=Lsm_Sensor.Accel_Y;
		flash_accY[i] = conv.array[0];
		flash_accY[i+1] = conv.array[1];
		flash_accY[i+2] = conv.array[2];
		flash_accY[i+3] = conv.array[3];

		conv.fVal=Lsm_Sensor.Accel_Z;
		flash_accZ[i] = conv.array[0];
		flash_accZ[i+1] = conv.array[1];
		flash_accZ[i+2] = conv.array[2];
		flash_accZ[i+3] = conv.array[3];


		timer_200ms_flag =0;

		i=i+4;

	}
/***********************************END*************************************************/
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
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLRCLK;
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
  sConfig.Channel = ADC_CHANNEL_12;
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
  hi2c1.Init.ClockSpeed = 400000;
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
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
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 840;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 19999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 8400-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 59999;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 1680;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 3000-1;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 16800;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 10000-1;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  huart3.Init.BaudRate = 9600;
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
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 19200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14|CS_Pin|BUZZER_Pin|GATE_D_Pin
                          |GATE_C_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, M0_Pin|M1_Pin|FN_Pin|LED2_Pin
                          |LED1_Pin|GATE_B_Pin|GATE_A_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC14 CS_Pin BUZZER_Pin GATE_D_Pin
                           GATE_C_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_14|CS_Pin|BUZZER_Pin|GATE_D_Pin
                          |GATE_C_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : M0_Pin M1_Pin FN_Pin LED2_Pin
                           LED1_Pin GATE_B_Pin GATE_A_Pin */
  GPIO_InitStruct.Pin = M0_Pin|M1_Pin|FN_Pin|LED2_Pin
                          |LED1_Pin|GATE_B_Pin|GATE_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SECINP_Pin */
  GPIO_InitStruct.Pin = SECINP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SECINP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_Pin */
  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB8 INT2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_8|INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int16_t I2C_Testsensor(void){

	HAL_StatusTypeDef status;

	status=HAL_I2C_IsDeviceReady(&hi2c1,0x6A <<1, 4, 100);


	if(HAL_OK==status){

		return 1;
		}
	else {
		return 0;
		}


}
void E220_CONFIG(uint8_t ADDH, uint8_t ADDL, uint8_t CHN, uint8_t MODE)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, SET);
    HAL_Delay(50);

    char cfg_buff[8] = {0}; // E220 için 8 elemanlı bir dizi kullanıyoruz
    enum {Transparent, Fixed} mode;
    mode = MODE;

    cfg_buff[0] = ADDH;
    cfg_buff[1] = ADDL;
    cfg_buff[2] = 0x62;
    cfg_buff[3] = 0x00;
    cfg_buff[4] = CHN;

    switch(mode){
        case Transparent:
            cfg_buff[5] = 0x00;  // opsiyon
            break;
        case Fixed:
            cfg_buff[5] = 0x11;
            break;
        default:
            cfg_buff[5] = 0x11;
     }

     cfg_buff[6] = 0x00;
     cfg_buff[7] = 0x00;


    HAL_UART_Transmit(&huart3, (uint8_t*) cfg_buff, 8, 1000);

    HAL_Delay(25);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, RESET);
    HAL_Delay(25);
}

float BME280_Get_Altitude(void)
{
	float press = comp_data.pressure / 10000.0;
	float temp = comp_data.temperature / 100.0;
	alt = 44330 * (1 - pow((press / 1013.25),(1/5.255)));
	//alt = ((pow((P0/press), (1/5.257))-1) * (temp + 273.15)) / 0.0065;

	return (alt);
}
int8_t user_i2c_read(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
  if(HAL_I2C_Master_Transmit(&hi2c1, (id << 1), &reg_addr, 1, 10) != HAL_OK) return -1;
  if(HAL_I2C_Master_Receive(&hi2c1, (id << 1) | 0x01, data, len, 10) != HAL_OK) return -1;

  return 0;
}

void user_delay_ms(uint32_t period)
{
  HAL_Delay(period);
}
int8_t user_i2c_write(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
  int8_t *buf;
  buf = malloc(len +1);
  buf[0] = reg_addr;
  memcpy(buf +1, data, len);

  if(HAL_I2C_Master_Transmit(&hi2c1, (id << 1), (uint8_t*)buf, len + 1, HAL_MAX_DELAY) != HAL_OK) return -1;

  free(buf);
  return 0;
}

void union_converter(void)
{
	 float2unit8 f2u8_gpsalt;
    f2u8_gpsalt.fVal=gps.altitude;
		 for(uint8_t i=0;i<4;i++)
		 {
			loratx[i+5]=f2u8_gpsalt.array[i];
		 }

	 float2unit8 f2u8_latitude;
	 f2u8_latitude.fVal=gps.latitude;
		 for(uint8_t i=0;i<4;i++)
		 {
			loratx[i+9]=f2u8_latitude.array[i];
		 }

	 float2unit8 f2u8_longitude;
	 f2u8_longitude.fVal=gps.longitude;
		 for(uint8_t i=0;i<4;i++)
		 {
			loratx[i+13]=f2u8_longitude.array[i];
		 }

	 float2unit8 f2u8_altitude;
	 f2u8_altitude.fVal=altitude;
		 for(uint8_t i=0;i<4;i++)
		 {
			loratx[i+17]=f2u8_altitude.array[i];
		 }
	 float2unit8 f2u8_speed;
	 f2u8_speed.fVal=speed;
		 for(uint8_t i=0;i<4;i++)
		 {
			loratx[i+21]=f2u8_speed.array[i];
		 }

	 float2unit8 f2u8_temp;
	 f2u8_temp.fVal=temperature;
		 for(uint8_t i=0;i<4;i++)
		 {
			loratx[i+25]=f2u8_temp.array[i];
		 }

	 float2unit8 f2u8_accx;
	 f2u8_accx.fVal=Lsm_Sensor.Accel_X;
		 for(uint8_t i=0;i<4;i++)
		 {
			loratx[i+29]=f2u8_accx.array[i];
		 }

	 float2unit8 f2u8_accy;
	 f2u8_accy.fVal=Lsm_Sensor.Accel_Y;
	 	 for(uint8_t i=0;i<4;i++)
		 {
			loratx[i+33]=f2u8_accy.array[i];
		 }

	 float2unit8 f2u8_accz;
	 f2u8_accz.fVal=Lsm_Sensor.Accel_Z;
	 	 for(uint8_t i=0;i<4;i++)
		 {
		    loratx[i+37]=f2u8_accz.array[i];
		 }

	 float2unit8 f2u8_roll;
	 f2u8_roll.fVal=real_roll;
		 for(uint8_t i=0;i<4;i++)
		 {
			loratx[i+41]=f2u8_roll.array[i];
		 }
	 float2unit8 f2u8_pitch;
	 f2u8_pitch.fVal=real_pitch;
		 for(uint8_t i=0;i<4;i++)
		 {
			loratx[i+45]=f2u8_pitch.array[i];
		 }
}

void EGU_Buff_Load(void)
{
	loratx[52]=EGU_RX_BUFFER[29];//EGU HATA
	loratx[53]=EGU_RX_BUFFER[30];//EGU AYRILMA TESPIT
	loratx[54]=EGU_RX_BUFFER[6];//EGU BATARYA-F
	loratx[55]=EGU_RX_BUFFER[7];
	loratx[56]=EGU_RX_BUFFER[8];
	loratx[57]=EGU_RX_BUFFER[9];
	loratx[58]=EGU_RX_BUFFER[22];//EGU ANGLE-F
	loratx[59]=EGU_RX_BUFFER[23];
	loratx[60]=EGU_RX_BUFFER[24];
	loratx[61]=EGU_RX_BUFFER[25];
	loratx[62]=EGU_RX_BUFFER[10];//EGU IRTIFA-F
	loratx[63]=EGU_RX_BUFFER[11];
	loratx[64]=EGU_RX_BUFFER[12];
	loratx[65]=EGU_RX_BUFFER[13];
	loratx[66]=EGU_RX_BUFFER[26];//EGU UCUS BASLADIMI?
	loratx[67]=EGU_RX_BUFFER[28];//STAGE AYRILDIMI?
	loratx[68]=EGU_RX_BUFFER[27];//MOTOR ATESLEME TALEBİ GELDİ Mİ?
	loratx[69]='\n';

}



void Buzzer(int how_many , uint32_t how_long)
{
	if((how_many %2) == 1) how_many++;

	  for(uint8_t i=0;i<how_many; i++)
	  {
	  	    HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_4);
	  	    HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_14);
	  	    HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_13);
	  		HAL_Delay(how_long);


	  }
}

void Altitude_Offset()
{
	for(uint8_t i=0;i<5;i++)
	{
		HAL_Delay(40);
	  rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, &dev);
	  rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);
	  if(rslt == BME280_OK)
	  { pressure = comp_data.pressure;
	    offset_altitude=BME280_Get_Altitude();
	  }
	}
}
void Flash_toplu_temizleme(int page)
{
	for(uint8_t i=0;i<page;i++)
		{
		W25Q_Erase_Sector(i);
		HAL_Delay(500);
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
