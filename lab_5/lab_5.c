//
// 		LAB 5
//
//
//		Should make sure AVDD is 3.3V, AVSS is 0V, checked!
//		Should make sure VR- (the negative input is 0V), is AVss, checked!
// 		Should make sure the sample time (around 504Hz) is correct and ok
//  	Should check output format
//
#include "LCD.h"
#include <plib.h>
#include "p32xxxx.h"

// function declaration
void IntGlobal_init (void);
void ADC_init (void);
char in2char(int n);
void LCDOut(int n);

// enable interrupt
void IntGlobal_init() {
	INTCONbits.MVEC = 1;	// Enable multiple vector interrupt
	asm("ei"); 				// Enable all interrupts
}

// initialize the ADC module
void ADC_init (void) {
	TRISB = 0xFFFF;				// set PORTB as input
	AD1PCFG = 0xEFFF;			// set RB12/AN12 as anolog input, other as digital
								// AD12, full scale input scan
	AD1CON1 = 0x2040;			/* Configure: No operation in IDLE mode, 
									integer 16-bit format,
									timer3 period match, normal operation,
									SSRC bit = 010 implies TMR3 period match --
									ends sampling and starts converting. */
	AD1CON2 = 0x0000;			/* Configure ADC voltage reference
								   and buffer fill modes.
								   VREF from AVDD (should be 3.3V) and AVSS (0V),
								   Inputs are not scanned, Interrupt every sample */
	AD1CON3 = 0x1FFF;			// Sample time = 31 TAD = 31 * 512 * TPB (~504Hz)

	AD1CHS = 0x000C0000;		// Connect RB12/AN12 as CH0 positive input
								// negative input is VR-=AVss (make sure it is zero)
	AD1CSSL = 0;				// no scan

	TMR3= 0x0000;
	PR3= 0xC350;				// 50 ms (50000 cycles)
	T3CON = 0x0030;				// 1 MHz, no prescale. start the timer

	//IPS6SET = 0x0014;			// Set Priority to 5
	//IPS6SET = 0x0003;			// Set Sub Priority to 3
	IFS1CLR = 0x0002;			/* Clear ADC conversion interrupt*/
	IEC1SET = 0x0002;			// Enable ADC interrupts
	
	T3CONSET = 0x8000;			// turn on the TMR3
	AD1CON1SET = 0x8000;		// turn ON the ADC
	AD1CON1SET = 0x0004;		// start auto sampling every 50 mSecs
								// repeat continuously
}

// convert int to char
char in2char(int n) {
	char c; 
	c = n+48;
 	return c;
}

// output to LCD
void LCDOut(int n) {
	int a=0, b=0, c=0;
	uchar first, second, third;
	
	a=(n*33)/10240 * 3;
	b=(n*33)/1024 * 3 -10 * a; 
	c=(n*330)/1024*3 - 100 * a - 10 * b;
	/*
	if ( c >= 10 ) {
		c = 0
		b += 1;
		if ( b >= 10 ) {
			b = 0;
			a += 1;
		}
	}
	else if ( b >= 10 ) {
		b = 0;
		a += 1;
	}
	*/

	first=in2char(a); 
	second=in2char(b); 
	third=in2char(c);
	
	uchar vovl[16] ="                "; 
	vovl[0] = first;
	vovl[1] = '.';
	vovl[2] = second;
	vovl[3] = third;
	vovl[5] ='V';

	LCD_goto(0x40);
	LCD_puts(vovl); 
	DelayUsec(40); 
}	

// main function
int main() {
	DDPCONbits.JTAGEN = 0;
	OSCSetPBDIV (OSC_PB_DIV_1);		// configure PBDIV so PBCLK = SYSCLK
	IntGlobal_init();				// initialize interrupt globally
	MCU_init(); 					// initialize the PIC32 MCU
	LCD_init(); 					// initialize the LCD module

	// check LCD module works fine or not
	uchar startStr1[] = "Voltage is:     ";	// initialize the string for display
	LCD_goto(0x0);							// LCD sendout display
	LCD_puts(startStr1);					// LCD display
	DelayUsec(40);
	
	ADC_init();								// initialize the ADC module
	
	int ADCValue = 0;
	int total = 0;	
	int i = 0;

	while(1) {
		while (!IFS1 & 0x0002) {};			// conversion done?
		ADCValue = ADC1BUF0;				// yes then get first ADC value
		IFS1CLR = 0x0002;					// clear AD1IF
		
		total = total + ADCValue;
		i++;
		if ( i == 3 ) {
			LCD_goto(0x0);
			LCD_puts(startStr1);
			DelayUsec(40);
			LCDOut(total/3);
			i = 0;
			total = 0;
		}
	}
}




