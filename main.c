#include <p32xxxx.h>
#include <plib.h>

#define BRATE_BT 103 // 9600 Bd (BREGH=1)
#define BRATE_AS 103 // 9600 Bd (BREGH=1)

/* Function prototypes */
void ADC_init (void);
void BT_send(char c);
void initUART(void);
void initIntGlobal();
void findDoubleVol(int IntVoltage, int indic__);
int lrClick(double inVol);
void flexSensor(void);

/* global varialbe declaration */
int ADCValueMiddle = 0;			// for temporary storage of the adc value for middle finger
int ADCValueIndex = 0;			// for temporary storage of the adc value for index finger
int totalIndex = 0;				// total accumulated index finger voltage
int totalMiddle = 0;			// total accumulated middle&fourth finger voltage
int i = 0;						// indicator for finger voltage
double volIndex = 0;			// the current voltage in double
double volMiddle = 0;			// the current voltage in double
double threHoVolFlexSen = 1.6; 	// set the threshold voltage for flex sensor here

#pragma interrupt UART_RXISR ipl6 vector 24
void UART_RXISR(void)
{
	/*
		Dealing with the data
	*/

	//IFS0bits.U2RXIF = 0;
}

/* main */
main() {
	OSCSetPBDIV (OSC_PB_DIV_1); 		// configure PBDIV so PBCLK = SYSCLK 8 MHz ??
	initIntGlobal();			
	initUART();
	ADC_init();							// initialize the ADC module
	/*
		initGyro();
	*/

	// infinite loop
	while(1) {
		flexSensor();
	}
}
/* end of main */ 

// send a char via bluetooth
void BT_send(char c) {
	while (!IFS0bits.U1TXIF) {};
	U1STAbits.UTXEN = 1;
	U1TXREG = c;
	IFS0bits.U1TXIF = 0;
}

/*----------------------------------------------
	UART config
	U1 for BlueTooth
		9600Bd, 8-bit data, no parity, 1 Stop bit
	U2 for Angle Sensor
		9600Bd, 8-bit data, no parity, 1 Stop bit
		Interrupt 
-----------------------------------------------*/
void initUART(void) {
	asm("di");

	U1BRG = BRATE_BT;
	U1MODEbits.BRGH = 1;
	U1STA = 0;
	U1STASET = 0x8000;

	IFS0bits.U1TXIF = 0;
	IEC0bits.U1TXIE = 1;
	U1STAbits.UTXEN = 1;
	/*
	U2BRG = BRATE_AS;
	U2MODEbits.BRGH = 1;
	U2STAbits.URXEN = 1;
	U2STA = 0;
	IFS0bits.U2RXIF = 0;
	IEC0bits.U2RXIE = 1;
	IPC6bits.U2IP = 6;
	IPC6bits.U2IS = 3;
	*/
	U1MODEbits.ON = 1;
	//U2MODEbits.ON = 1;

	asm("ei");
}

// intialize the global configurations
void initIntGlobal() {
	DDPCONbits.JTAGEN = 0;
  	TRISFbits.TRISF2 = 1;	// RF2/U1RX/J11-41/input
	TRISFbits.TRISF3 = 0;	// RF3/U1TX/J11-43/output
	TRISFbits.TRISF4 = 1;	// RF2/U2RX/J11-46/input
	TRISFbits.TRISF5 = 0;	// RF3/U2TX/J11-48/output
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

	AD1CON2 = 0x0404;			/* Configure ADC voltage reference
								   and buffer fill modes.
								   VREF from AVDD (3.3V) and AVSS (0V),
								   Inputs scanned for MUX-A, Interrupt every 2th sample/convert 
								   one 16-word buffer, always use MUX-A */

	AD1CON3 = 0x1FFF;			// Sample time = 31 TAD = 31 * 512 * TPB (~504Hz)

	AD1CHS = 0;					/* Connect RB12/AN12 as CH0 positive input,
									should be ignored as scanning is enabled */
								// negative input is VR- = AVss (0V)

	AD1CSSL = 0x3000;			// select AN13&AN12 for scan. Skip otheres.
								// Start from AN12 and alternate (start from lower)
	
	// configure timer 3
	TMR3= 0x0000;
	PR3= 0xC350;				// 50 ms (50000 cycles)
	T3CON = 0x0040;				// 1 MHz (prescale set to 1:8) [now 1:64 should recalculate]

	// configure ADC interrupt
								// AD1IP<2:0> bits IPC6<28:26> :: AD1IS<1:0> bits IPC6<25:24>
	IFS1CLR = 0x0002;			// Clear ADC conversion interrupt
	IEC1SET = 0x0002;			// Enable ADC interrupts

	T3CONSET = 0x8000;			// turn on the TMR3
	AD1CON1SET = 0x8000;		// turn ON the ADC
	AD1CON1SET = 0x0004;		// start auto sampling every 50 mSecs
								// repeat continuously
}

// convert the voltage measured from int to double.
void findDoubleVol(int IntVoltage, int indic__) {
	int a=0, b=0, c=0;

	a = (IntVoltage*33)/10240;
	b = (IntVoltage*33)/1024 -10 * a; 
	c = (IntVoltage*330)/1024 - 100 * a - 10 * b;

	if ( indic__ == 1 ) {
		volIndex = a + b * 0.1 + c * 0.01;
	}
	else {
		volMiddle = a + b * 0.1 + c * 0.01;
	}
}

// click or not?
int lrClick(double __inVol) {
	if ( __inVol > threHoVolFlexSen )
		return 1;		// CLICK !!
	else
		return 0;		// NO CLICK !!
}

// collect and analyze flex sensors data
void flexSensor(void) {
	while (!IFS1 & 0x0002) {};			// conversion done?
	AD1CON1bits.ASAM = 0;				// stop auto sampling
	ADCValueIndex = ADC1BUF0;			// yes then get the first ADC value::AN12
	ADCValueMiddle = ADC1BUF1;			// yes then get the second ADC value::AN13
	AD1CON1bits.ASAM = 1;				// start auto sampling
	IFS1CLR = 0x0002;					// clear AD1IF

	totalIndex = totalIndex + ADCValueIndex;
	totalMiddle = totalMiddle + ADCValueMiddle;
	i++;
	if ( i == 3 ) { 					// convert for 3 times to average
		findDoubleVol(totalIndex/3, 1);
		findDoubleVol(totalMiddle/3, 0);

		if ( lrClick(volIndex) == 1 )	// index finger click enabled
			BT_send('I');
		else
			BT_send('J');
		
		if ( lrClick(volMiddle) == 1 )	// middle finger click enabled
			BT_send('M');
		else
			BT_send('K');

		BT_send('\n');
	
		i = 0;
		volMiddle = 0;
		volIndex = 0;
		totalIndex = 0;
		totalMiddle = 0;
	}

}
