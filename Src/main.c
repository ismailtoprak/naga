#include "main.h"
#include "stm32f4xx_hal.h"
#include "MY_NRF24.h"
#include "defines.h"
#include "tm_stm32_mpu6050.h"
#include <math.h>
#include "stdint.h"

uint64_t RxpipeAddrs = 0x11223344AA;
uint16_t adc_data[32], gaz, motor1, motor2, motor3, motor4, start = 0;

TM_MPU6050_t MPU6050_Sensor;
uint16_t adress = 0x1A;
uint8_t data = 0x06;
uint8_t *dataPtr = &data;

float baslangic, son ;

//Gyro Variables
int gyro_x, gyro_y, gyro_z;
long acc_x, acc_y, acc_z, acc_total_vector;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
long loop_timer;
int lcd_loop_counter, set_gyro_angles;
float angle_pitch, angle_roll, angle_yaw;
int angle_pitch_buffer, angle_roll_buffer;
float angle_roll_acc, angle_pitch_acc;
float angle_pitch_output, angle_roll_output, angle_yaw_output;
float aci_x, aci_y, aci_z;

//PID settings

float roll, roll_PID, roll_error, roll_desired_value, roll_previous_error;
float roll_pid_p=0;
float roll_pid_i=0;
float roll_pid_d=0;

double roll_kp=0;
double roll_ki=0;
double roll_kd=5;
float roll_desired_angle = 0;

float pitch, pitch_PID, pitch_error, pitch_desired_value, pitch_previous_error;
float pitch_pid_p=0;
float pitch_pid_i=0;
float pitch_pid_d=0;

double pitch_kp=0;
double pitch_ki=0;
double pitch_kd=5;
float pitch_desired_angle = 0;   

float yaw, yaw_PID, yaw_error, yaw_desired_value, yaw_previous_error;
float yaw_pid_p=0;
float yaw_pid_i=0;
float yaw_pid_d=0;

double yaw_kp=1;
double yaw_ki=0.01;
double yaw_kd=0;
float yaw_desired_angle = 0;


I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;


void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	
	if(htim->Instance == TIM2){
	
			roll_desired_angle = (roll )/13 - 10;
			pitch_desired_angle = (pitch )/12.5f - 10;
				
			roll_error  =  aci_y 	- roll_desired_angle;
			pitch_error =  aci_x  - pitch_desired_angle;
			yaw_error   = -aci_z  + yaw_desired_angle;
				
			roll_pid_p  = roll_kp*roll_error;
			pitch_pid_p = pitch_kp*pitch_error;
			yaw_pid_p   = yaw_kp*yaw_error;
			
			if((roll_pid_i)  < -30 || (roll_pid_i)  > 30){roll_pid_i = roll_pid_i+(roll_ki*roll_error); }
			
			if((pitch_pid_i) < -30 || (pitch_pid_i) > 30){pitch_pid_i = pitch_pid_i+(pitch_ki*pitch_error); }
			
			if((yaw_pid_i)   < -30 || (yaw_pid_i)   > 30){yaw_pid_i = yaw_pid_i+(yaw_ki*yaw_error); }
			

			roll_pid_d  = roll_kd*((roll_error - roll_previous_error));														//dt ye bölme. Joob brokking öyle yapmis.
			pitch_pid_d = pitch_kd*((pitch_error - pitch_previous_error));
			yaw_pid_d   = yaw_kd*((yaw_error - yaw_previous_error));
			
			roll_PID  = roll_pid_p + roll_pid_i+ roll_pid_d;
			pitch_PID = pitch_pid_p + pitch_pid_i+ pitch_pid_d;
			yaw_PID   = yaw_pid_p + yaw_pid_i + yaw_pid_d;
				
			if(pitch_PID > 70){pitch_PID = 70;}
			if(pitch_PID < -70){pitch_PID = -70;}
			if(roll_PID > 70){roll_PID = 70;}
			if(roll_PID < -70){roll_PID = -70;}
			if(yaw_PID > 70){yaw_PID = 70;}
			if(yaw_PID < -70){yaw_PID = -70;}
		
			roll_previous_error  = roll_error;
			pitch_previous_error = pitch_error;
			yaw_previous_error   = yaw_error;
	}
	
		if(htim->Instance == TIM3){   //2ms
			
			TM_MPU6050_ReadAll(&MPU6050_Sensor);	
			
			gyro_x = MPU6050_Sensor.Gyroscope_X;
			gyro_y = MPU6050_Sensor.Gyroscope_Y;
			gyro_z = MPU6050_Sensor.Gyroscope_Z;
			
			acc_x = MPU6050_Sensor.Accelerometer_X;
			acc_y = MPU6050_Sensor.Accelerometer_Y;
			acc_z = MPU6050_Sensor.Accelerometer_Z;
			
			gyro_x -= gyro_x_cal;
			gyro_y -= gyro_y_cal;
			gyro_z -= gyro_z_cal;
			
			angle_pitch += gyro_x * 0.00015267;
			angle_roll 	+= gyro_y * 0.00015267;
			
			if(gyro_z < 100 && gyro_z > -100){gyro_z 	 = 0;}
			angle_yaw   += gyro_z * 0.00015267;
			
			
			angle_pitch += angle_roll  * sin(gyro_z * 0.000002665);
		  angle_roll 	-= angle_pitch * sin(gyro_z * 0.000002665);
			
			acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));
			
			angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;
			angle_roll_acc 	= asin((float)acc_x/acc_total_vector)* -57.296;
			
			angle_pitch_acc -= 0.0;
			angle_roll_acc -= 0.0;
			
			if(set_gyro_angles){                                                 
				angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;     
				angle_roll  = angle_roll  * 0.9996 + angle_roll_acc  * 0.0004;       
				}
			else{                                                                //At first start
				angle_pitch = angle_pitch_acc;                                      
				angle_roll = angle_roll_acc;                                       
				set_gyro_angles = 1;                                            	//Set the IMU started flag
				}
			
			angle_pitch_output = angle_pitch_output * 0.96 + angle_pitch * 0.04;
			angle_roll_output  = angle_roll_output  * 0.96 + angle_roll  * 0.04;
			angle_yaw_output = angle_yaw;
	}
}

int main(void)
{
  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
	
	//HAL_I2C_Master_Transmit(&hi2c1, adress, dataPtr, 1, 1000);
	while(TM_MPU6050_Init(&MPU6050_Sensor, TM_MPU6050_Device_0, TM_MPU6050_Accelerometer_8G, TM_MPU6050_Gyroscope_500s) != TM_MPU6050_Result_Ok){}
		
	for (int cal_int = 0; cal_int < 5000 ; cal_int ++){
		
	TM_MPU6050_ReadAll(&MPU6050_Sensor);
	gyro_x = MPU6050_Sensor.Gyroscope_X;
	gyro_y = MPU6050_Sensor.Gyroscope_Y;
	gyro_z = MPU6050_Sensor.Gyroscope_Z;
	
	acc_x = MPU6050_Sensor.Accelerometer_X;
	acc_y = MPU6050_Sensor.Accelerometer_Y;
	acc_z = MPU6050_Sensor.Accelerometer_Z;
	
	gyro_x_cal += gyro_x;
	gyro_y_cal += gyro_y;
	gyro_z_cal += gyro_z; 
		
}
		
	gyro_x_cal /= 5000;
	gyro_y_cal /= 5000;
	gyro_z_cal /= 5000;

	HAL_Delay(8000);
	
	HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim3);
	
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
 
	NRF24_begin(GPIOC, GPIO_PIN_4, GPIO_PIN_5, hspi1);
	NRF24_setAutoAck(true);
	NRF24_setChannel(52);
	NRF24_setPayloadSize(32);
	NRF24_openReadingPipe(1, RxpipeAddrs);
	NRF24_enableAckPayload();
	NRF24_startListening();
	
  while (1){

		loop_timer = HAL_GetTick();
		
		if(NRF24_available()){
			
			NRF24_read(adc_data, 32);
			
			gaz		= (230 - adc_data[0]);
			pitch	= adc_data[1];
			roll	= adc_data[2];
			aci_x = -(angle_pitch_output - 2.0f );
			aci_y = -(angle_roll_output + 2.2f);
			aci_z = angle_yaw_output;
			
			if((start == 0) && (roll == 0) && (pitch == 0)){ start = 1; }                   ////----Dronun aktif olmasi için özel sart bekleniyor-----////
			
			if((start == 1) && (gaz > 25) && (gaz < 250)){
	
				motor3 = gaz - roll_PID - pitch_PID + yaw_PID;
				motor4 = gaz + roll_PID - pitch_PID - yaw_PID;
				motor1 = gaz + roll_PID + pitch_PID + yaw_PID;
				motor2 = gaz - roll_PID + pitch_PID - yaw_PID;
				
				if(motor1 > 100){ motor1 = 100;}
				if(motor2 > 100){ motor2 = 100;}
				if(motor3 > 100){ motor3 = 100;}
				if(motor4 > 100){ motor4 = 100;}
			
				__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, motor1);
				__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, motor2);
				__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, motor3);
				__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, motor4);
	
			}
			else{
				
				motor1		= 0;
				motor2 	 	= 0;
				motor3	 	= 0;
				motor4 		= 0;
				__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 0);
				__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 0);
				__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 0);
				__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 0);
			
			}
		}
	}
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_16_9;
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

}

static void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }

}

static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 83;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 249;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
	
  HAL_TIM_MspPostInit(&htim1);

}

static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
	
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOC, CSNpin_Pin|CEpin_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = CSNpin_Pin|CEpin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

void Error_Handler(void)
{
	
}
