/************************************************************************
* FlexSensor.h
* Header file for the Flex Sensor
************************************************************************/
//
// 		FlexSensor 
//		Index Finger = left click
//		Middle+Fourth Finger = right click
//
// 		Should make sure the sample time (around 504Hz) is correct and ok ???
//
//  	Use RB12/AN12 as anlalog input for the index finger PIN49 J10
//  	Use RB13/AN13 as anlalog input for the middle finger PIN50 J10
//
//		Timer 3 used for ADC
//		T_PB should be 8 MHz
//
//		Should make sure AVDD is 3.3V, AVSS is 0V, checked!
//		Should make sure VR- (the negative input) is AVss (=0V) checked!
//
#ifndef __FlexSensor_h
#define __FlexSensor_h
#include <p32xxxx.h>
#endif 

/* Function prototypes */
void ADC_init (void);
void findDoubleVol(int IntVoltage);
int lrClick(double inVol);
void flexSensor(void);
// convert int to char
char in2char(int n);
/***************************end of FlexSensor.h*************************/
