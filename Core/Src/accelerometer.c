/*
 * accelerometer.c
 *
 *  Created on: Nov 2, 2024
 *      Author: LDegr
 */

#include "accelerometer.h"

#include "stm32f4xx_hal_spi.h"

float gravity = 9.807f;

/*************************************************************
                      ACCELEROMETER CODE
**************************************************************/

gpio_Pin imuInt = { .gpioGroup = GPIOC, .gpioPin = GPIO_PIN_13 };          //IMU_INR
gpio_Pin imuCS = { .gpioGroup = GPIOC, .gpioPin = GPIO_PIN_5};            //CS

void accWrite(uint8_t add, uint8_t *data){
	HAL_GPIO_WritePin(imuCS.gpioGroup, imuCS.gpioPin, 0); // setting CS LOW
	HAL_SPI_Transmit(&hspi1, &add, 1, 100);
	HAL_SPI_Transmit(&hspi1, data, 1, 100);
	HAL_GPIO_WritePin(imuCS.gpioGroup, imuCS.gpioPin, 1);// setting CS HIGH
	return;
}


uint8_t accRead(uint8_t add){
	uint8_t buff;
  uint8_t data = 0x00;
	buff = add | 0x80;

	HAL_GPIO_WritePin(imuCS.gpioGroup, imuCS.gpioPin, 0); // setting CS LOW
	HAL_SPI_Transmit(&hspi1, &buff, sizeof buff, 100);
	HAL_SPI_Receive(&hspi1, &data, sizeof data, 100);
	HAL_GPIO_WritePin(imuCS.gpioGroup, imuCS.gpioPin, 1);// setting CS HIGH
	return data;
}

void accInit(void){
  uint8_t changeState;
  uint8_t ret;

  SEGGER_RTT_printf(0, "\nAccelerometer Sensor Check... \n");
  SEGGER_RTT_printf(0, "______________________ \n");
  //accelerometer Control Reg

  ret = accRead(0x0F);
  SEGGER_RTT_printf(0, "CHIP ID: 0x%02X\n", ret);
  // we are expecting 0x6C here

  ret = accRead(0x10);
  SEGGER_RTT_printf(0, "ACCELEROMETER CONTROL REG: 0x%02X\n", ret);
  changeState = ret | 0b10100000;
  accWrite(0x10, &changeState);
  ret = accRead(0x10);
  SEGGER_RTT_printf(0, "ACCELEROMETER CONTROL REG UPDATED: 0x%02X\n", ret);

  //GyroControl Reg
  ret = accRead(0x11);
  SEGGER_RTT_printf(0, "GYRO CONTROL REG: 0x%02X\n", ret);
  changeState = ret | 0b01010000;
  accWrite(0x11, &changeState);
  ret = accRead(0x11);
  SEGGER_RTT_printf(0, "GYRO CONTROL REG UPDATED: 0x%02X\n", ret);

  //
  ret = accRead(0x13);
  changeState = ret | 0b01000000;
  SEGGER_RTT_printf(0, "CONTROL REG 4: 0x%02X\n", ret);
  accWrite(0x13, &changeState);
  ret = accRead(0x13);
  SEGGER_RTT_printf(0, "CONTROL REG 4: 0x%02X\n", ret);

  // Control Register 7
  ret = accRead(0x16);
  changeState = ret | 0b00000000;
  SEGGER_RTT_printf(0, "CONTROL REG 4: 0x%02X\n", ret);
  accWrite(0x16, &changeState);
  ret = accRead(0x16);
  SEGGER_RTT_printf(0, "CONTROL REG 4: 0x%02X\n", ret);


  // Control Register 7
  accWrite(0x12, 0x01);

}

void accTask(void){
  uint8_t MSB;
  uint8_t LSB;
  uint16_t combinedZ;
  uint16_t combinedY;
  uint16_t combinedX;

  MSB = accRead(0x2D);
  LSB = accRead(0x2C);
  combinedZ = (MSB << 8) | LSB;

  MSB = accRead(0x2B);
  LSB = accRead(0x2A);
  combinedY = (MSB << 8) | LSB;

  MSB = accRead(0x29);
  LSB = accRead(0x28);
  combinedX = (MSB << 8) | LSB;

  // +- 4g

  zAcceleration = 4.0f * gravity * ((float)combinedZ / 65536.0f);
  yAcceleration = 4.0f * gravity * ((float)combinedY / 65536.0f);
  xAcceleration = 4.0f * gravity * ((float)combinedX / 65536.0f);

  firFilterUpdate(&lpfAccZ, zAcceleration, 'A');
  firFilterUpdate(&lpfAccY, yAcceleration, 'A');
  firFilterUpdate(&lpfAccX, xAcceleration, 'A');

  avgZAcceleration = lpfAccZ.out;
  avgYAcceleration = lpfAccY.out;
  avgXAcceleration = lpfAccX.out;

  return;
}


void readTemp(void){
	uint8_t MSB;
	uint8_t LSB;
	uint16_t combinedTemperature;
	MSB = accRead(0x21);
	LSB = accRead(0x20);

	combinedTemperature = (MSB << 8) | LSB;
	temperature = 4.0f * gravity * ((float)combinedTemperature / 65536.0f);
}

void readGyro(void){
	  uint8_t MSB;
	  uint8_t LSB;
	  uint16_t combinedRoll;
	  uint16_t combinedYaw;
	  uint16_t combinedPitch;

	  MSB = accRead(0x23);
	  LSB = accRead(0x22);
	  combinedRoll = (MSB << 8) | LSB;

	  MSB = accRead(0x27);
	  LSB = accRead(0x26);
	  combinedYaw = (MSB << 8) | LSB;

	  MSB = accRead(0x25);
	  LSB = accRead(0x24);
	  combinedPitch = (MSB << 8) | LSB;

	  firFilterUpdate(&lpfRoll, roll , 'G');
	  firFilterUpdate(&lpfPitch, pitch, 'G');
	  firFilterUpdate(&lpfYaw, yaw, 'G');

	  roll = 4.0f * gravity * ((float)combinedRoll / 65536.0f);
	  pitch = 4.0f * gravity * ((float)combinedPitch / 65536.0f);
	  yaw = 4.0f * gravity * ((float)combinedYaw / 65536.0f);

	  avgRoll = lpfRoll.out;
	  avgPitch = lpfPitch.out;
	  avgYaw = lpfYaw.out;

}

/*************************************************************
                      POSITIONING CODE
**************************************************************/

// Structure to hold position state
typedef struct {
    float pos_x;
    float pos_y;
    float pos_z;
    float vel_x;
    float vel_y;
    float vel_z;
    float last_ax;
    float last_ay;
    float last_az;
    uint32_t last_time_ms;
} PositionState;

// Initialize position tracking state
void init_position_state(PositionState* state) {
    state->pos_x = 0.0f;
    state->pos_y = 0.0f;
    state->pos_z = 0.0f;
    state->vel_x = 0.0f;
    state->vel_y = 0.0f;
    state->vel_z = 0.0f;
    state->last_ax = 0.0f;
    state->last_ay = 0.0f;
    state->last_az = 0.0f;
    state->last_time_ms = 0;
}


void update_position(PositionState* state,
                    float accel_x, float accel_y, float accel_z,
                    uint32_t current_time_ms) {
    // Calculate time delta in seconds
    float dt = (current_time_ms - state->last_time_ms) / 1000.0f;
    if (dt <= 0 || state->last_time_ms == 0) {
        state->last_time_ms = current_time_ms;
        state->last_ax = accel_x;
        state->last_ay = accel_y;
        state->last_az = accel_z - GRAVITY_COMPENSATION;
        return;
    }

    // Apply low pass filter to smooth accelerometer data
    float filtered_ax = (accel_x * LOW_PASS_ALPHA) + (state->last_ax * (1.0f - LOW_PASS_ALPHA));
    float filtered_ay = (accel_y * LOW_PASS_ALPHA) + (state->last_ay * (1.0f - LOW_PASS_ALPHA));
    float filtered_az = ((accel_z - GRAVITY_COMPENSATION) * LOW_PASS_ALPHA) +
                       (state->last_az * (1.0f - LOW_PASS_ALPHA));

    // Apply threshold to reduce drift
    if (fabs(filtered_ax) < ACCEL_THRESHOLD) filtered_ax = 0;
    if (fabs(filtered_ay) < ACCEL_THRESHOLD) filtered_ay = 0;
    if (fabs(filtered_az) < ACCEL_THRESHOLD) filtered_az = 0;

    // First integration: acceleration to velocity
    state->vel_x += filtered_ax * dt;
    state->vel_y += filtered_ay * dt;
    state->vel_z += filtered_az * dt;

    // Second integration: velocity to position
    state->pos_x += state->vel_x * dt;
    state->pos_y += state->vel_y * dt;
    state->pos_z += state->vel_z * dt;

    // Update last values
    state->last_ax = filtered_ax;
    state->last_ay = filtered_ay;
    state->last_az = filtered_az;
    state->last_time_ms = current_time_ms;
}

/*************************************************************
                      FILTERING CODE
**************************************************************/

// Function to calculate filter coefficients for higher order Butterworth filter
void butterworth_lowpass_coefficients(int order, float cutoff_frequency, float sampling_rate, float* a, float* b) {
    int n;
    float wc = tanf(cutoff_frequency * 3.1415926f / sampling_rate);

    for (n = 0; n < order; ++n) {
        float theta = (float)(2 * n + 1) * 3.1415926f / (2.0f * order);
        float sn = sinf(theta);
        float cs = cosf(theta);
        float beta = 0.5f * ((1.0f - sn * wc) / (1.0f + sn * wc));
        float gamma = (0.5f + beta) * cs;
        float alpha = 0.5f * (0.5f + beta - gamma);

        b[3 * n + 0] = alpha;
        b[3 * n + 1] = 2.0f * alpha;
        b[3 * n + 2] = alpha;

        a[3 * n + 0] = 1.0f;
        a[3 * n + 1] = -2.0f * gamma;
        a[3 * n + 2] = 2.0f * beta;
    }
}


// Function to apply the Butterworth filter
void butterworth_lowpass_filter(float* data, float* output, int length, int order, float* a, float* b) {
    float x1[order], x2[order];
    float y1[order], y2[order];

    for (int i = 0; i < order; ++i) {
        x1[i] = x2[i] = y1[i] = y2[i] = 0.0f;
    }

    for (int i = 0; i < length; ++i) {
        float x0 = data[i];
        float y0 = 0.0f;

        for (int j = 0; j < order; ++j) {
            y0 += (b[3 * j + 0] * x0 + b[3 * j + 1] * x1[j] + b[3 * j + 2] * x2[j] - a[3 * j + 1] * y1[j] - a[3 * j + 2] * y2[j]) / a[3 * j + 0];

            x2[j] = x1[j];
            x1[j] = x0;
            y2[j] = y1[j];
            y1[j] = y0;
        }

        output[i] = y0;
    }
}


void firFilterInit(FIRFilter * fir){
  for(int n = 0; n < FIR_FILTER_LENGTH; n++){
    fir->buf[n] = 0.0f;
  }
  fir->bufIndex = 0;
  fir->out      = 0.0f;
}

float firFilterUpdate(FIRFilter * fir, float inp, char sensor){
  fir->buf[fir->bufIndex] = inp;
  fir->bufIndex++;
  if(fir->bufIndex == FIR_FILTER_LENGTH){
    fir->bufIndex = 0;
  }
  fir->out = 0.0f;

  int sumIndex = fir->bufIndex;

  for(int n = 0; n < FIR_FILTER_LENGTH; n++){
    if(sumIndex > 0){
      sumIndex--;
    }
    else{
      sumIndex = FIR_FILTER_LENGTH - 1;
    }
    if(sensor == 'A'){
        fir->out += FIR_IMPULSE_RESPONSE[n] * fir->buf[sumIndex];
    }
    else if (sensor == 'G'){
        fir->out += FIR_HIGH_PASS[n] * fir->buf[sumIndex];
    }
  }
  return fir->out;
}
