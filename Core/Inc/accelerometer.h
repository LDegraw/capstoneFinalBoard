/*
 * accelerometer.h
 *
 *  Created on: Nov 2, 2024
 *      Author: LDegr
 */
#include "main.h"
#include "stm32f4xx_hal_spi.h"
#include "SEGGER_RTT.h"
#include "math.h"

#ifndef INC_ACCELEROMETER_H_
#define INC_ACCELEROMETER_H_
#define FIR_FILTER_LENGTH 39

extern SPI_HandleTypeDef hspi1;
extern float yAcceleration;
extern float xAcceleration;
extern float zAcceleration;
extern float roll;
extern float pitch;
extern float yaw;
extern float avgRoll;
extern float avgPitch;
extern float avgYaw;
extern float avgXAcceleration;
extern float avgYAcceleration;
extern float avgZAcceleration;

extern float temperature;

typedef struct{
  float buf[FIR_FILTER_LENGTH];
  int bufIndex;
  float out;
}FIRFilter;

extern FIRFilter lpfAccZ;
extern FIRFilter lpfAccY;
extern FIRFilter lpfAccX;

extern FIRFilter lpfRoll;
extern FIRFilter lpfPitch;
extern FIRFilter lpfYaw;

// Constants for filtering and compensation
#define GRAVITY_COMPENSATION 9.81f
#define ACCEL_THRESHOLD 0.1f    // Threshold to consider acceleration as movement
#define LOW_PASS_ALPHA 0.1f     // Low pass filter coefficient

//void init_position_state(PositionState* state);
//uint8_t accRead(uint8_t add);
//void accWrite(uint8_t add, uint8_t *data);
//
//
//void accInit(void);
//void accTask(void);
//
//void update_position(PositionState* state,
//                    float accel_x, float accel_y, float accel_z,
//                    uint32_t current_time_ms);



// accelerometer Impulse response: for very strong 10Hz lowpass filter
static float FIR_IMPULSE_RESPONSE[FIR_FILTER_LENGTH] =
{-0.000412874162859016
,-0.000893972307076119
,-0.001584465556485542
,-0.002554694045519977
,-0.003744290984319871
,-0.004924581340138285
,-0.005695269737174212
,-0.005521205769576145
,-0.003807101790746756
,0.000000000000000002
,0.006297330758057457
,0.015222635107387141
,0.026580595014201715
,0.039816007172827121
,0.054042926874405935
,0.068129510621657022
,0.080828392608713828
,0.090933955365679067
,0.097442419779686526
,0.099689364782559742
,0.097442419779686526
,0.090933955365679067
,0.080828392608713842
,0.068129510621657050
,0.054042926874405935
,0.039816007172827121
,0.026580595014201715
,0.015222635107387144
,0.006297330758057457
,0.000000000000000002
,-0.003807101790746757
,-0.005521205769576151
,-0.005695269737174216
,-0.004924581340138284
,-0.003744290984319868
,-0.002554694045519980
,-0.001584465556485542
,-0.000893972307076118
,-0.000412874162859016};


static float FIR_HIGH_PASS[FIR_FILTER_LENGTH] =
{-0.000645722349929725,
0.001522765377451210,
-0.000723278213801605,
-0.001844681320159923,
0.003572410513387025,
-0.000651050497596164,
-0.005962841257508440,
0.007956312679697819,
0.001549044200801827,
-0.015198688696710868,
0.013994678320044215,
0.009564260082970609,
-0.032895361803331934,
0.020233073397634069,
0.031867132638490676,
-0.070591578325853804,
0.024925843507861596,
0.119583963920804823,
-0.286231030289831101,
0.359949496231159349,
-0.286231030289831101,
0.119583963920804823,
0.024925843507861600,
-0.070591578325853832,
0.031867132638490676,
0.020233073397634069,
-0.032895361803331927,
0.009564260082970613,
0.013994678320044217,
-0.015198688696710868,
0.001549044200801828,
0.007956312679697828,
-0.005962841257508443,
-0.000651050497596164,
0.003572410513387022,
-0.001844681320159925,
-0.000723278213801605,
0.001522765377451209,
-0.000645722349929725};

void butterworth_lowpass_filter(float* data, float* output, int length, int order, float* a, float* b);
float firFilterUpdate(FIRFilter * fir, float inp, char sensor);
void butterworth_lowpass_coefficients(int order, float cutoff_frequency, float sampling_rate, float* a, float* b);

#endif /* INC_ACCELEROMETER_H_ */

