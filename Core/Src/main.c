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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "string.h"
#include "usbd_cdc_if.h"
#include <math.h>
#include "accelerometer.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define UART_TRANSMIT_TIMEOUT 1000  // 1 second in milliseconds
#define UART_RECEIVE_TIMEOUT  1000  // 1 second in milliseconds

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c3;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim13;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
#define TRIG_PIN_1 GPIO_PIN_0
#define TRIG_PORT_1 GPIOC
#define TRIG_PIN_2 GPIO_PIN_1
#define TRIG_PORT_2 GPIOC
#define TRIG_PIN_3 GPIO_PIN_2
#define TRIG_PORT_3 GPIOC
#define TRIG_PIN_4 GPIO_PIN_3
#define TRIG_PORT_4 GPIOC

uint32_t IC_Val1_Left = 0;
uint32_t IC_Val2_Left = 0;
uint32_t IC_Val1_Right = 0;
uint32_t IC_Val2_Right = 0;
uint32_t IC_Val1_Front = 0;
uint32_t IC_Val2_Front = 0;
uint32_t IC_Val1_Back = 0;
uint32_t IC_Val2_Back = 0;
uint32_t Difference = 0;
uint8_t Is_First_Captured_1 = 0;  // is the first value captured
uint8_t Is_First_Captured_2 = 0;  // is the first value captured
uint8_t Is_First_Captured_3 = 0;  // is the first value captured
uint8_t Is_First_Captured_4 = 0;  // is the first value captured
float Distance_Left ;
float Distance_Right ;
float Distance_Back ;
float Distance_Front;
float avgDistLeft;
float avgDistRight;
float avgDistBack;
float avgDistFront;
uint8_t dataBuffer;

float temperature;
float yAcceleration;
float xAcceleration;
float zAcceleration;
float temp;
float roll;
float pitch;
float yaw;
float avgRoll;
float avgPitch;
float avgYaw;
float avgXAcceleration;
float avgYAcceleration;
float avgZAcceleration;
FIRFilter lpfAccZ;
FIRFilter lpfAccY;
FIRFilter lpfAccX;
FIRFilter lpfRoll;
FIRFilter lpfPitch;
FIRFilter lpfYaw;

gpio_Pin en_motor1 = { .gpioGroup = GPIOB, .gpioPin = GPIO_PIN_12 };
gpio_Pin en_motor2 = { .gpioGroup = GPIOB, .gpioPin = GPIO_PIN_13};
gpio_Pin en_motor3 = { .gpioGroup = GPIOB, .gpioPin = GPIO_PIN_14};
gpio_Pin en_motor4 = { .gpioGroup = GPIOB, .gpioPin = GPIO_PIN_15};
#define BUFFER_SIZE 128  // Adjust based on your needs


char data[BUFFER_SIZE];
uint8_t usbBuffer[128];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C3_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM13_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void stopMotors();
void left();
void right();
void forward();
void reverse();
uint8_t calculate_checksum(uint8_t *packet, int length);
void HCSR04_Read_Right();
void HCSR04_Read_Left();
void HCSR04_Read_Front();
void HCSR04_Read_Back();
void setServoPosition(uint8_t address, uint16_t position) ;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim == &htim7){
		accTask();
	}
	else if(htim == &htim10){
//		readGyro();
		readTemp();
	}
	else if(htim == &htim13){
//		HCSR04_Read_Back();
//		HCSR04_Read_Right();
//		HCSR04_Read_Left();
//		HCSR04_Read_Front();

	}
}



/*************************************************************
                      MOTOR CODE
**************************************************************/
//void motorInit(){
//	//motor timers start
//	HAL_TIM_Base_Start(&htim3);
//	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
//	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
//	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
//	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
//
//	htim3.Instance->CCR1 = 0; // M2 FWD
//	htim3.Instance->CCR2 = 0; // M2 REV
//	htim3.Instance->CCR3 = 0; // M1 FWD
//	htim3.Instance->CCR4 = 0; // M1 FWD
//
//	HAL_TIM_Base_Start(&htim4);
//	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
//	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
//	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
//	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
//
//	htim4.Instance->CCR1 = 0; // M3 FWD
//	htim4.Instance->CCR2 = 0; // M3 FWD
//	htim4.Instance->CCR3 = 0; //
//	htim4.Instance->CCR4 = 0; // M4 FWD
//}
//
//
//void forward(){
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);  // en1
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);  // en2
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);  // en3
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);  // en4
//
//	htim3.Instance->CCR1 = 150; // M2 FWD
//	htim3.Instance->CCR2 = 0; // M2 REV
//	htim3.Instance->CCR3 = 0; // M1 FWD
//	htim3.Instance->CCR4 = 0; // M1 REV
//
//	htim4.Instance->CCR1 = 150; // M3 FWD
//	htim4.Instance->CCR2 = 0; // M3 REV
//	htim4.Instance->CCR3 = 150; // M4 FWD
//	htim4.Instance->CCR4 = 0; // M4 REV
//}
//
//void reverse(){
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);  // en1
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);  // en2
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);  // en3
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);  // en4
//
//	htim3.Instance->CCR1 = 0; // M2 FWD
//	htim3.Instance->CCR2 = 150; // M2 REV
//	htim3.Instance->CCR3 = 0; // M1 FWD
//	htim3.Instance->CCR4 = 0; // M1 REV
//
//	htim4.Instance->CCR1 = 0; // M3 FWD
//	htim4.Instance->CCR2 = 150; // M3 REV
//	htim4.Instance->CCR3 = 0; // M4 FWD
//	htim4.Instance->CCR4 = 150; // M4 REV
//}
//
//void left(){
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);  // en1
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);  // en2
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);  // en3
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);  // en4
//
//	htim3.Instance->CCR1 = 0; // M2 FWD
//	htim3.Instance->CCR2 = 150; // M2 REV
//	htim3.Instance->CCR3 = 0; // M1 FWD
//	htim3.Instance->CCR4 = 0; // M1 REV
//
//	htim4.Instance->CCR1 = 150; // M3 FWD
//	htim4.Instance->CCR2 = 0; // M3 REV
//	htim4.Instance->CCR3 = 150; // M4 FWD
//	htim4.Instance->CCR4 = 0; // M4 REV
//}
//
//void right(){
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);  // en1
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);  // en2
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);  // en3
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);  // en4
//
//	htim3.Instance->CCR1 = 150; // M2 FWD
//	htim3.Instance->CCR2 = 0; // M2 REV
//	htim3.Instance->CCR3 = 150; // M1 FWD
//	htim3.Instance->CCR4 = 0; // M1 REV
//
//	htim4.Instance->CCR1 = 0; // M3 FWD
//	htim4.Instance->CCR2 = 150; // M3 REV
//	htim4.Instance->CCR3 = 0; // M4 FWD
//	htim4.Instance->CCR4 = 150; // M4 REV
//}
//
//void stopMotors(){
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);  // en1
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);  // en2
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);  // en3
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);  // en4
//
//	htim3.Instance->CCR1 = 0; // M2 FWD
//	htim3.Instance->CCR2 = 0; // M2 REV
//	htim3.Instance->CCR3 = 0; // M1 FWD
//	htim3.Instance->CCR4 = 0; // M1 REV
//
//	htim4.Instance->CCR1 = 0; // M3 FWD
//	htim4.Instance->CCR2 = 0; // M3 REV
//	htim4.Instance->CCR3 = 0; // M4 FWD
//	htim4.Instance->CCR4 = 0; // M4 REV
//}



/*************************************************************
                      SERVO CODE
**************************************************************/

/**
 *
 *
 * WRITING TO A REGISTER IS 0X03
 * ------------------------------------------------------------------------------------------------------
 *			HEADER			|	ID		|	LEN			|	INST	|	PARAM1	|	PARAM2	|	CHECKSUM
 * ------------------------------------------------------------------------------------------------------
 * 				|			|			|				|			|			|			|
 *		0XFF  	|	0XFF	|	0X??	|	PARAM + 2	|	 0X03	|	X19		|	0X01	|	0XDC
 *				|			|			|				|			|			|			|
 * ------------------------------------------------------------------------------------------------------
 * **/
uint8_t calculate_checksum(uint8_t *packet, int length) {
    unsigned int sum = 0;
    for(int i = 2; i < length - 1; i++) {  // Start from ID (skip 0xFF 0xFF)
        sum += packet[i];
    }
    return (uint8_t)(255 - (sum % 256));
}

void servoCommand(){
    uint8_t motors[] = {0x02, 0x06, 0x08};  // Array of motor IDs
    uint16_t motor2Angle = (usbBuffer[2]<<8) | usbBuffer[1];
    uint16_t motor6Angle = (usbBuffer[4]<<8) | usbBuffer[3];
    uint16_t motor8Angle = (usbBuffer[6]<<8) | usbBuffer[5];

    // Set positions for each motor
    setServoPosition(motors[0], motor2Angle);
    HAL_Delay(1);  // Small delay between commands
    setServoPosition(motors[1], motor6Angle);
    HAL_Delay(1);
    setServoPosition(motors[2], motor8Angle);
    return;
}

void setServoPosition(uint8_t address, uint16_t position) {
    uint8_t sendBuffer[9] = {0xFF, 0xFF, address, 0x05, 0x03, 0x1E,
                            position & 0xFF,        // Low byte
                            (position >> 8) & 0xFF, // High byte
                            0x00};
    sendBuffer[8] = calculate_checksum(sendBuffer, sizeof(sendBuffer));

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, sendBuffer, sizeof(sendBuffer), 0xFFFF);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
}

void turnServoLed(uint8_t address) {
    uint8_t sendBuffer[8]= {0xFF, 0xFF, address, 0x04, 0x03, 0x19, 0x01, 0x00};
    sendBuffer[7] = calculate_checksum(sendBuffer, sizeof(sendBuffer));

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, sendBuffer, sizeof(sendBuffer), 0xFFFF);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
}

void setCWAngleLimit(uint8_t address, uint16_t angle) {
    uint8_t sendBuffer[9] = {0xFF, 0xFF, address, 0x05, 0x03, 0x06,
                            angle & 0xFF,        // Low byte
                            (angle >> 8) & 0xFF, // High byte
                            0x00};
    sendBuffer[8] = calculate_checksum(sendBuffer, sizeof(sendBuffer));

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, sendBuffer, sizeof(sendBuffer), 0xFFFF);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
}

void setCCWAngleLimit(uint8_t address, uint16_t angle) {
    uint8_t sendBuffer[9] = {0xFF, 0xFF, address, 0x05, 0x03, 0x08,
                            angle & 0xFF,        // Low byte
                            (angle >> 8) & 0xFF, // High byte
                            0x00};
    sendBuffer[8] = calculate_checksum(sendBuffer, sizeof(sendBuffer));

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, sendBuffer, sizeof(sendBuffer), 0xFFFF);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
}

void setMovingSpeed(uint8_t address, uint16_t speed) {
    uint8_t sendBuffer[9] = {0xFF, 0xFF, address, 0x05, 0x03, 0x20,
                            speed & 0xFF,        // Low byte
                            (speed >> 8) & 0xFF, // High byte
                            0x00};
    sendBuffer[8] = calculate_checksum(sendBuffer, sizeof(sendBuffer));

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, sendBuffer, sizeof(sendBuffer), 0xFFFF);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
}

void setTorqueLimit(uint8_t address, uint16_t torque) {
    uint8_t sendBuffer[9] = {0xFF, 0xFF, address, 0x05, 0x03, 0x22,
                            torque & 0xFF,        // Low byte
                            (torque >> 8) & 0xFF, // High byte
                            0x00};
    sendBuffer[8] = calculate_checksum(sendBuffer, sizeof(sendBuffer));

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, sendBuffer, sizeof(sendBuffer), 0xFFFF);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
}

void setMaxTorque(uint8_t address, uint16_t maxTorque) {
    uint8_t sendBuffer[9] = {0xFF, 0xFF, address, 0x05, 0x03, 0x0E,
                            maxTorque & 0xFF,        // Low byte
                            (maxTorque >> 8) & 0xFF, // High byte
                            0x00};
    sendBuffer[8] = calculate_checksum(sendBuffer, sizeof(sendBuffer));

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, sendBuffer, sizeof(sendBuffer), 0xFFFF);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
}

void setTorqueEnable(uint8_t address, uint8_t enable) {
    uint8_t sendBuffer[8] = {0xFF, 0xFF, address, 0x04, 0x03, 0x18, enable, 0x00};
    sendBuffer[7] = calculate_checksum(sendBuffer, sizeof(sendBuffer));

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, sendBuffer, sizeof(sendBuffer), 0xFFFF);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
}

/*************************************************************
                      STEREOSENSOR CODE
**************************************************************/


static uint8_t Is_First_Captured_Front = 0;
static uint8_t Is_First_Captured_Right = 0;
static uint8_t Is_First_Captured_Left = 0;
static uint8_t Is_First_Captured_Back = 0;


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef* htim)
{
    uint32_t Difference = 0;

    // Front Sensor - Channel 1
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
        if (Is_First_Captured_Front == 0) // First rising edge
        {
            IC_Val1_Front = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            Is_First_Captured_Front = 1;
            // Change polarity to falling edge
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
        }
        else if (Is_First_Captured_Front == 1) // Falling edge
        {
            IC_Val2_Front = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            __HAL_TIM_SET_COUNTER(htim, 0);

            if (IC_Val2_Front > IC_Val1_Front)
                Difference = IC_Val2_Front - IC_Val1_Front;
            else if (IC_Val1_Front > IC_Val2_Front)
                Difference = (0xffff - IC_Val1_Front) + IC_Val2_Front;

            Distance_Front = (float)Difference * .034/2;
            Is_First_Captured_Front = 0;

            // Reset for next measurement
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
            HAL_TIM_IC_Stop_IT(htim, TIM_CHANNEL_1);
        }
    }

    // Right Sensor - Channel 2
    else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
    {
        if (Is_First_Captured_Right == 0)
        {
            IC_Val1_Right = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
            Is_First_Captured_Right = 1;
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_FALLING);
        }
        else if (Is_First_Captured_Right == 1)
        {
            IC_Val2_Right = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
            __HAL_TIM_SET_COUNTER(htim, 0);

            if (IC_Val2_Right > IC_Val1_Right)
                Difference = IC_Val2_Right - IC_Val1_Right;
            else if (IC_Val1_Right > IC_Val2_Right)
                Difference = (0xffff - IC_Val1_Right) + IC_Val2_Right;

            Distance_Right = (float)Difference * .034/2;
            Is_First_Captured_Right = 0;

            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);
            HAL_TIM_IC_Stop_IT(htim, TIM_CHANNEL_2);
        }
    }

    // Left Sensor - Channel 3
    else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
    {
        if (Is_First_Captured_Left == 0)
        {
            IC_Val1_Left = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
            Is_First_Captured_Left = 1;
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_FALLING);
        }
        else if (Is_First_Captured_Left == 1)
        {
            IC_Val2_Left = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
            __HAL_TIM_SET_COUNTER(htim, 0);

            if (IC_Val2_Left > IC_Val1_Left)
                Difference = IC_Val2_Left - IC_Val1_Left;
            else if (IC_Val1_Left > IC_Val2_Left)
                Difference = (0xffff - IC_Val1_Left) + IC_Val2_Left;

            Distance_Left = (float)Difference * .034/2;
            Is_First_Captured_Left = 0;

            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);
            HAL_TIM_IC_Stop_IT(htim, TIM_CHANNEL_3);
        }
    }

    // Back Sensor - Channel 4
    else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
    {
        if (Is_First_Captured_Back == 0)
        {
            IC_Val1_Back = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
            Is_First_Captured_Back = 1;
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_FALLING);
        }
        else if (Is_First_Captured_Back == 1)
        {
            IC_Val2_Back = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
            __HAL_TIM_SET_COUNTER(htim, 0);

            if (IC_Val2_Back > IC_Val1_Back)
                Difference = IC_Val2_Back - IC_Val1_Back;
            else if (IC_Val1_Back > IC_Val2_Back)
                Difference = (0xffff - IC_Val1_Back) + IC_Val2_Back;

            Distance_Back = (float)Difference * .034/2;
            Is_First_Captured_Back = 0;

            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_RISING);
            HAL_TIM_IC_Stop_IT(htim, TIM_CHANNEL_4);
        }
    }
}

float approxRollingAverage(float avg, float new, float weight){
  return (((weight - 1.0f) / weight) * avg) + ((1.0f / weight) * new);
}

void delay_us (uint16_t us)
{
  __HAL_TIM_SET_COUNTER(&htim14, 0);  // set the counter value a 0
  while (__HAL_TIM_GET_COUNTER(&htim14) < us);  // wait for the counter to reach the us input in the parameter
}

void HCSR04_Read_Front (void)
{
  HAL_GPIO_WritePin(TRIG_PORT_1, TRIG_PIN_1, GPIO_PIN_SET);  // pull the TRIG pin HIGH
  delay_us(10);  // wait for 10 us
  HAL_GPIO_WritePin(TRIG_PORT_1, TRIG_PIN_1, GPIO_PIN_RESET);  // pull the TRIG pin low
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
}

void HCSR04_Read_Right (void)
{
  HAL_GPIO_WritePin(TRIG_PORT_2, TRIG_PIN_2, GPIO_PIN_SET);  // pull the TRIG pin HIGH
  delay_us(10);  // wait for 10 us
  HAL_GPIO_WritePin(TRIG_PORT_2, TRIG_PIN_2, GPIO_PIN_RESET);  // pull the TRIG pin low
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
}

void HCSR04_Read_Left (void)
{
  HAL_GPIO_WritePin(TRIG_PORT_3, TRIG_PIN_3, GPIO_PIN_SET);  // pull the TRIG pin HIGH
  delay_us(10);  // wait for 10 us
  HAL_GPIO_WritePin(TRIG_PORT_3, TRIG_PIN_3, GPIO_PIN_RESET);  // pull the TRIG pin low
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);
}

void HCSR04_Read_Back (void)
{
  HAL_GPIO_WritePin(TRIG_PORT_4, TRIG_PIN_4, GPIO_PIN_SET);  // pull the TRIG pin HIGH
  delay_us(10);  // wait for 10 us
  HAL_GPIO_WritePin(TRIG_PORT_4, TRIG_PIN_4, GPIO_PIN_RESET);  // pull the TRIG pin low
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4);
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
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_I2C3_Init();
  MX_TIM7_Init();
  MX_TIM14_Init();
  MX_TIM10_Init();
  MX_TIM13_Init();
  MX_USB_DEVICE_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  char rxBuffer[8];

  char txBuffer[8];

//  motorInit();
  accInit();
  firFilterInit(&lpfAccZ);
  firFilterInit(&lpfAccY);
  firFilterInit(&lpfAccX);

//  HAL_TIM_Base_Start(&htim14); // ultrasonic timing
  HAL_TIM_Base_Start_IT(&htim7); // accelerometer sampling timer
//  HAL_TIM_Base_Start_IT(&htim10); // gyro sampling timer
//  HAL_TIM_Base_Start_IT(&htim13); // ultrasonic sampling timer

  char * newData[64];
  int blink = 1;
  char * updated;
  uint8_t returnValue[6];


  uint8_t motors[] = {0x02, 0x06, 0x08};  // Array of motor IDs

  // Set limits for each motor
  for(int i = 0; i < 3; i++) {
      // Set CW (minimum) limit to 0
      setCWAngleLimit(motors[i], 0);
	  HAL_Delay (50);

      // Set CCW (maximum) limit to 1023 (300 degrees)
      setCCWAngleLimit(motors[i], 1023);
	  HAL_Delay (50);
      setTorqueLimit(motors[i], 1023);
	  HAL_Delay (50);
      setTorqueEnable(motors[i], 1);
	  HAL_Delay (50);
      setMaxTorque(motors[i], 1023);
	  HAL_Delay (50);
      setMovingSpeed(motors[i], 0x002F);
	  HAL_Delay (50);
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  for(int i = 0; i < 3; i++) {
//
//		  setTorqueEnable(motors[i], 1, returnValue);
//	  }
	  turnServoLed(0x06);
	  HAL_Delay (50);
	  turnServoLed(0x02);
	  HAL_Delay (50);
	  turnServoLed(0x08);
	  HAL_Delay (50);
	  //
	  int result;
	  memset(data, 0, BUFFER_SIZE);
	  result = snprintf(data, BUFFER_SIZE, "AvgX: %d.%02d, AvgY: %d.%02d, AvgZ: %d.%02d, F: %d.%02d, L: %d.%02d, R: %d.%02d, B: %d.%02d\n",
			  	  	  	 (int)avgXAcceleration,
			  	         (int)((avgXAcceleration - (int)avgXAcceleration) * 100),
			  	  	  	 (int)avgYAcceleration,
			  	         (int)((avgYAcceleration - (int)avgYAcceleration) * 100),
	                     (int)avgZAcceleration,
	                     (int)((avgZAcceleration - (int)avgZAcceleration) * 100),
	                     (int)Distance_Front,
	                     (int)((Distance_Front - (int)Distance_Front) * 100),
	                     (int)Distance_Left,
	                     (int)((Distance_Left - (int)Distance_Left) * 100),
	                     (int)Distance_Right,
	                     (int)((Distance_Right - (int)Distance_Right) * 100),
	                     (int)Distance_Back,
	                     (int)((Distance_Back - (int)Distance_Back) * 100));


	  CDC_Transmit_FS(data, sizeof(data));
	  HAL_Delay (1000);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 24;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 16;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
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
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  htim4.Init.Prescaler = 16;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 255;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  htim7.Init.Prescaler = 100;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 800;
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
  htim10.Init.Prescaler = 100;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 800;
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
  * @brief TIM13 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM13_Init(void)
{

  /* USER CODE BEGIN TIM13_Init 0 */

  /* USER CODE END TIM13_Init 0 */

  /* USER CODE BEGIN TIM13_Init 1 */

  /* USER CODE END TIM13_Init 1 */
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 200;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 800;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM13_Init 2 */

  /* USER CODE END TIM13_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 16;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 65535;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

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
  huart1.Init.BaudRate = 1000000;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_5|GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3
                           PC5 PC10 PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_5|GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15
                           PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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
