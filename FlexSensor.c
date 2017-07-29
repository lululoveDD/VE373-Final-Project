//
// 		FlexSensor 
//		Index Finger = left click
//		Middle+Fourth Finger = right click
//
// 		Should make sure the sample time (around 504Hz) is correct and ok ????
//  	Use RB12/AN12 as anlalog input for the index finger PIN49 J10
//  	Use RB13/AN13 as anlalog input for the middle finger PIN50 J10
//		Timer 3 used for ADC
//		T_PB should be 8 MHz
//
//		Should make sure AVDD is 3.3V, AVSS is 0V, checked!
//		Should make sure VR- (the negative input) is AVss (=0V) checked!
// 
#include <plib.h>
#include "p32xxxx.h"

// global varialbe declaration
int portID = 0;					// 0 stands for AN12(index finger) just being sampled
								// 1 stands for AN13(middle finger) just being sampled
int ADCValue = 0;				// for temporary storage
int totalIndex = 0;				// total accumulated index finger voltage
int totalMiddle = 0;			// total accumulated middle&fourth finger voltage
int iIndex = 0;					// indicator for index finger voltage
int iMiddle = 0;				// indicator for middle&fourth finger voltage
double vol = 0;					// the current voltage in double
double threHoVolFlexSen = 1.4; 	// set the threshold voltage for flex sensor here

// function declaration
void IntGlobal_init (void);
void ADC_init (void);
void findDoubleVol(int IntVoltage);
bool lrClick(double inVol);
void flexSensor(void);

// enable interrupt
void IntGlobal_init() {
	INTCONbits.MVEC = 1;	// Enable multiple vector interrupt
	asm("ei"); 				// Enable all interrupts
}

// initialize the ADC module
void ADC_init (void) {
	// congfigure ADC port
	TRISB = 0xFFFF;				// set PORTB as input
	AD1PCFG = 0xDFFF;			// set RB12/AN12 & RB13/AN13 as anolog input::other digital

	// congfigure ADC controls
	AD1CON1 = 0x2040;			/* Configure: No operation in IDLE mode, 
									integer 16-bit format,
									timer3 period match, normal operation,
									SSRC bit = 010 implies TMR3 period match --
									ends sampling and starts converting. */

	AD1CON2 = 0x0400;			/* Configure ADC voltage reference
								   and buffer fill modes.
								   VREF from AVDD (3.3V) and AVSS (0V),
								   Inputs scanned for MUX-A, Interrupt every sample 
								   one 16-word buffer, always use MUX-A */

	AD1CON3 = 0x1FFF;			// Sample time = 31 TAD = 31 * 512 * TPB (~504Hz)

	AD1CHS = 0x000C0000;		/* Connect RB12/AN12 as CH0 positive input,
									should be ignored as scanning is enabled */
								// negative input is VR- = AVss (0V)

	AD1CSSL = 0x3000;			// select AN13&AN12 for scan. Skip otheres.
								// Start from AN12 and alternate
	// configure timer 3
	TMR3= 0x0000;
	PR3= 0xC350;				// 50 ms (50000 cycles)
	T3CON = 0x0030;				// 1 MHz (prescale set to 1:8)
	IPC3SET = 0x0000001C;		// Interrupt priority level 7, Subpriority level 0
	IFS0CLR = 0x00001000;		// Clear timer interrupt flag
	IEC0SET = 0x00001000;		// Enable Timer3 interrupt

	// configure ADC interrupt
								// AD1IP<2:0> bits IPC6<28:26> :: AD1IS<1:0> bits IPC6<25:24>
	IPC6SET = 0x18000000;		// Interrupt priority level 6, Subpriority level 0
	IFS1CLR = 0x0002;			// Clear ADC conversion interrupt
	IEC1SET = 0x0002;			// Enable ADC interrupts

	T3CONSET = 0x8000;			// turn on the TMR3
	AD1CON1SET = 0x8000;		// turn ON the ADC
	AD1CON1SET = 0x0004;		// start auto sampling every 50 mSecs
								// repeat continuously
}

// convert the voltage measured from int to double.
void findDoubleVol(int IntVoltage) {
	int a=0, b=0, c=0;

	a = (indexVoltage*33)/10240;
	b = (indexVoltage*33)/1024 -10 * a; 
	c = (indexVoltage*330)/1024 - 100 * a - 10 * b;

	vol = a + b * 0.1 + c * 0.01;
}

// click or not?
bool lrClick(double inVol) {
	if ( vol > threHoVolFlexSen )
		return 1;		// CLICK !!
	else
		return 0;		// NO CLICK !!
}

// collect and analyze flex sensors data
void flexSensor(void) {
	while (!IFS1 & 0x0002) {};			// conversion done?
	ADCValue = ADC1BUF0;				// yes then get first ADC value
	IFS1CLR = 0x0002;					// clear AD1IF

	if ( portID == 0 ) { // index finger just being sampled
		totalIndex = totalIndex + ADCValue;
		iIndex++;
		portID++;
	}
	else { // index finger just being sampled
		totalMiddle = totalMiddle + ADCValue;
		iMiddle++;
		portID--;
	}

	if ( iIndex == 3 ) { 				// convert for 3 times to average
		findDoubleVol(totalIndex/3);
		if ( lrClick(vol) ) {
			// click enabled
			// TODO::send to RN41
		}
		vol = -1; 						// for debug
		iIndex = 0;
		totalIndex = 0;
	}
	
	if ( iMiddle == 3 ) {				// convert for 3 times to average
		findDoubleVol(totalMiddle/3);
		if ( lrClick(vol) ) {
			// click enabled
			// TODO::send to RN41
		}
		vol = -1; 						// for debug
		iMiddle = 0;
		totalMiddle = 0;
	}
}

// main function
void main() {
	DDPCONbits.JTAGEN = 0;
	OSCSetPBDIV (OSC_PB_DIV_1);			// configure PBDIV so PBCLK = SYSCLK 8 MHz
	IntGlobal_init();					// initialize interrupt globally
	ADC_init();							// initialize the ADC module
	
	// TODO :: configure..collect and analysis gyro
	// TODO :: configure RN41 Bluetooth module
	
	// infinite loop
	while(1) {
		flexSensor();
		//
		//
	}

}




