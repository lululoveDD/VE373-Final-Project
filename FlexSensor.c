/************************************************************************
* FlexSensor.c
* implement file for the Flex Sensor
************************************************************************/
#include "FlexSensor.h"
#include <plib.h>
#include <p32xxxx.h>

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

//
char in2char(int n) {
	char c; 
	c = n+48;
 	return c;
}

// convert the voltage measured from int to double.
void findDoubleVol(int IntVoltage) {
	int a=0, b=0, c=0;
	unsigned char first, second, third;

	a = (IntVoltage*33)/10240;
	b = (IntVoltage*33)/1024 -10 * a; 
	c = (IntVoltage*330)/1024 - 100 * a - 10 * b;

	vol = a + b * 0.1 + c * 0.01;

	first=in2char(a); 
	second=in2char(b); 
	third=in2char(c);

	U1TXREG = first;
	U1TXREG = second;
	U1TXREG = third;
	U1TXREG = '\n';
}

// click or not?
int lrClick(double inVol) {
	if ( vol > threHoVolFlexSen )
		return 1;		// CLICK !!
	else
		return 0;		// NO CLICK !!
}

// collect and analyze flex sensors data
void flexSensor(void) {
	while (!IFS1 & 0x0002) {};			// conversion done?
	IFS1CLR = 0x0002;					// clear AD1IF
	ADCValue = ADC1BUF0;				// yes then get first ADC value

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
		if ( lrClick(vol) == 1 ) {
			// click enabled
			//PORTDbits.RD0 = 1;
			//int i = 0;
			//while ( i != 4000000) {i++;}
			//PORTDbits.RD0 = 0;
			// TODO::send to RN41
		}
		vol = 0; 						// for debug
		iIndex = 0;
		totalIndex = 0;
	}

	if ( iMiddle == 3 ) {				// convert for 3 times to average
		findDoubleVol(totalMiddle/3);
		if ( lrClick(vol) == 1 ) {
			// click enabled
			//PORTDbits.RD1 = 1;
			//int i = 0;
			//while ( i != 4000000) {i++;}
			//PORTDbits.RD1 = 0;
			// TODO::send to RN41
		}
		vol = 0; 						// for debug
		iMiddle = 0;
		totalMiddle = 0;
	}
}

