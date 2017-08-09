#include <p32xxxx.h>
#include <plib.h>

#define BRATE_BT 103 // 9600 Bd (BREGH=1)
#define BRATE_AS 103 // 9600 Bd (BREGH=1)
#define fabs(x) x>=0?x:-x
#define MAX_ANGLE 40
#define LEVEL_ANGLE (MAX_ANGLE/4)

/* Function prototypes */
void vibration_init(void);
void Timer4_init(void);
void vibration_go(void);
void BT_send(char c);
void initUART(void);
void initIntGlobal(void);
void initGyro(void);
void ADC_init (void);
void findDoubleVol(int IntVoltage, int indic__);
int lrClick(double inVol);
void flexSensor(void);
void gyroSensor(void);
void bitSet(unsigned char* Reg, int bitNo){ *Reg |= (1<<bitNo); }
void bitClr(unsigned char* Reg, int bitNo){ if (*Reg & (1<<bitNo)) *Reg ^= (1<<bitNo); }
/* End of Function Prototypes */

/* Global Varialbe Declaration */
int ADCValueMiddle = 0;			// for temporary storage of the adc value for middle finger
int ADCValueIndex = 0;			// for temporary storage of the adc value for index finger
int totalIndex = 0;				// total accumulated index finger voltage
int totalMiddle = 0;			// total accumulated middle&fourth finger voltage
int i = 0;						// indicator for finger voltage
double volIndex = 0;			// the current voltage in double
double volMiddle = 0;			// the current voltage in double
double threHoVolFlexSen = 1.8; 	// set the threshold voltage for flex sensor here
// the following are for gyro data process
unsigned char Re_buf[11],counter = 0;
unsigned char sign = 0;
float a[3], w[3], T;
signed int angle[3];
// global register for bluethooth communication
unsigned char StatusReg = 0;
/* End of Global Variable Declaration */

#pragma interrupt UART_RXISR ipl6 vector 32
void UART_RXISR(void)
{
	Re_buf[counter] = (unsigned char)U2RXREG;
	IFS1CLR = 0x0200;
    if( counter == 0 && Re_buf[0] != 0x55 ) return;	// check if it is the start of the packet            
    counter++;       
    if( counter == 11 ) {	// receive 11 data 
       counter = 0;               
       sign = 1;
    }  
}

#pragma interrupt PWM_ISR ipl3 vector 8
void PWM_ISR (void) {
	T2CONCLR = 0x8000; 		// STOP Timer 2
	OC1CONCLR = 0x8000;		// STOP OC1 module for PWM generation
	IFS0CLR = 0x0100;
	IEC0CLR = 0x0100;
}

/* main */
main() {
	OSCSetPBDIV (OSC_PB_DIV_1); 		// configure PBDIV so PBCLK = SYSCLK 8 MHz ??
	initIntGlobal();			
	initUART();
	ADC_init();							// initialize the ADC module
	//vibration_init();					// initialize the vibration unit
	//vibration_go();
	initGyro();
	// infinite loop
	while(1) {
		flexSensor();
		gyroSensor();
	}
}
/* end of main */ 

// initialize the vibration module
void vibration_init(void) {
	OC1CON = 0x0000; 			// stop OC1 module
	OC1RS = 60;					// initialize duty cycle register
	OC1R = 60;					// initialize OC1R register for the first time
	OC1CON = 0x0006; 			// OC1 16-bit, Timer 2, in PWM mode w/o FP
	PR2 = 0xFFFF;				// PWM signal period = 0x100*1/PBCLK = 32 us  //Thus, PWM Frequency = 32.25 kHz
	IFS0CLR = 0x0100;			// clear Timer 2 interrupt
	IEC0SET = 0x0100; 		// enable Timer 2 interrupt
	IPC2SET = 0x001C;			// Timer 2 interrupt priority 7, subpriority 0
}

void vibration_go(void) {
	T2CONSET = 0x8000;		// start Timer 2
	OC1CONSET = 0x8000;		// enable OC1 module for PWM generation
}

void vibration_stop(void) {
	OC1RS = 0;
}

// send a char via bluetooth
void BT_send(char c) {
	while (!IFS0bits.U1TXIF) {};
	U1STAbits.UTXEN = 1;
	U1TXREG = c;
	IFS0CLR = 0x10000000;
}

void initGyro() {
	PORTDbits.RD0 = 1;
	int j = 100000;
	while (j--);
	PORTDbits.RD0 = 0;

	//while (!IFS1bits.U2TXIF) {};
	U2STAbits.UTXEN = 1;
	U2TXREG = 0xFF;
	//IFS1CLR = 0x0400;
	//while (!IFS1bits.U2TXIF) {};
	U2STAbits.UTXEN = 1;
	U2TXREG = 0xAA;
	//IFS1CLR = 0x0400;
	//while (!IFS1bits.U2TXIF) {};
	U2STAbits.UTXEN = 1;
	U2TXREG = 0x52;
	//IFS1CLR = 0x0400;

	PORTDbits.RD0 = 1;
	j = 100000;
	while (j--);
	PORTDbits.RD0 = 0;
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

	IFS0CLR = 0x10000000;
	IEC0bits.U1TXIE = 1;
	U1STAbits.UTXEN = 1;
	
	U2BRG = BRATE_AS;
	U2MODEbits.BRGH = 1;
	U2STA = 0;
	//U2STASET = 0x8000;   //FOR GYRO INIT
	U2STAbits.UTXEN = 1; //FOR GYRO INIT
	U2STAbits.URXEN = 1;
	//IFS1CLR = 0x0400;		//FOR GYRO INIT
	//IEC1bits.U2TXIE = 1;	//FOR GYRO INIT
	IFS1CLR = 0x0200;
	IEC1bits.U2RXIE = 1;
	IPC8bits.U2IP = 6;
	IPC8bits.U2IS = 3;
	
	U1MODEbits.ON = 1;
	U2MODEbits.ON = 1;

	asm("ei");
}

// intialize the global configurations
void initIntGlobal() {	
	DDPCONbits.JTAGEN = 0;	
	TRISDSET = 0xffffffff;	// Set all ports to the input
	TRISDCLR = 0x00000001;	// Set RD0 as output J11/PIN 19 for pwm vibration.
  	TRISFbits.TRISF2 = 1;	// RF2/U1RX/J11-41/input
	TRISFbits.TRISF3 = 0;	// RF3/U1TX/J11-43/output
	TRISFbits.TRISF4 = 1;	// RF2/U2RX/J11-46/input
	TRISFbits.TRISF5 = 0;	// RF3/U2TX/J11-48/output
	PORTDbits.RD0 = 0;
	INTCONbits.MVEC = 1;	// Enable multiple vector interrupt
	asm("ei"); 				// Enable all interrupts
}

// initialize the ADC module
void ADC_init (void) {
	// congfigure ADC port
	TRISB = 0xFFFF;				// set PORTB as input
	AD1PCFG = 0xDFFF;			// set RB12/AN12 & RB13/AN13 as anolog input::other digital

	// congfigure ADC controls
	AD1CON1 = 0x20E0;			/* Configure: No operation in IDLE mode, 
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
	//TMR3= 0x0000;
	//PR3= 0xC350;				// 50 ms (50000 cycles)
	//T3CON = 0x0040;				// 1 MHz (prescale set to 1:8) [now 1:64 should recalculate]

	// configure ADC interrupt
								// AD1IP<2:0> bits IPC6<28:26> :: AD1IS<1:0> bits IPC6<25:24>
	IFS1CLR = 0x0002;			// Clear ADC conversion interrupt
	IEC1SET = 0x0002;			// Enable ADC interrupts

	//T3CONSET = 0x8000;			// turn on the TMR3
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
			bitSet(&StatusReg, 0);
		else
			bitClr(&StatusReg, 0);
		
		if ( lrClick(volMiddle) == 1 )	// middle finger click enabled
			bitSet(&StatusReg, 1);
		else
			bitClr(&StatusReg, 1);
		BT_send(StatusReg);
		i = 0;
		volMiddle = 0;
		volIndex = 0;
		totalIndex = 0;
		totalMiddle = 0;
	}

}

// dealing with the gyro data
void gyroSensor(void) {
	if ( sign == 1 ) {
    	sign = 0;
    	if ( Re_buf[0] == 0x55 ) {
    		switch ( Re_buf[1] ) {
    			case 0x51:
					a[0] = ( (short)(Re_buf[3]<<8|Re_buf[2]) ) / 32768.0 * 16;
					a[1] = ( (short)(Re_buf[5]<<8|Re_buf[4]) ) / 32768.0 * 16;
					a[2] = ( (short)(Re_buf[7]<<8|Re_buf[6]) ) / 32768.0 * 16;
					T = ( (short)(Re_buf[9]<<8|Re_buf[8]) ) / 340.0 + 36.25;
					break;
				case 0x52:
					w[0] = ( (short)(Re_buf[3]<<8|Re_buf[2]) ) / 32768.0 * 2000;
					w[1] = ( (short)(Re_buf[5]<<8|Re_buf[4]) ) / 32768.0 * 2000;
					w[2] = ( (short)(Re_buf[7]<<8|Re_buf[6]) ) / 32768.0 * 2000;
					T = ( (short)(Re_buf[9]<<8|Re_buf[8]) ) / 340.0 + 36.25;
					break;
				case 0x53:
			        angle[0] = ( (short)(Re_buf[3]<<8|Re_buf[2]) ) / 32768.0 * 180;
					angle[1] = ( (short)(Re_buf[5]<<8|Re_buf[4]) ) / 32768.0 * 180;
					angle[2] = ( (short)(Re_buf[7]<<8|Re_buf[6]) ) / 32768.0 * 180;
					T = ( (short)(Re_buf[9]<<8|Re_buf[8]) ) / 340.0 + 36.25;			
	                break;
	                /* 
	                BT_send(Re_buf[3]);
    				BT_send(Re_buf[2]);
    				BT_send('\n');
    				int first = int(angle[0])/100;
    				int second = int(angle[0]-first*100) / 10;
    				int third = int(angle[0])%10;
    				BT_send(10+first);
    				BT_send(10+second);
    				BT_send(10+third);
    				*/
    		}
    		if (angle[0] >= 0.0)
    			bitClr(&StatusReg, 2);
    		else
    			bitSet(&StatusReg, 2);
    		int tmp = angle[0]/LEVEL_ANGLE;
			if (tmp < 0) tmp = tmp * -1;
			if (tmp > 3) tmp = 3;
    		switch(tmp) {
    			case 0:
    				bitClr(&StatusReg, 3);
    				bitClr(&StatusReg, 4);
    				break;
    			case 1:
    				bitSet(&StatusReg, 3);
    				bitClr(&StatusReg, 4);
					break;
    			case 2:
    				bitClr(&StatusReg, 3);
    				bitSet(&StatusReg, 4);
					break;
    			case 3:
    				bitSet(&StatusReg, 3);
    				bitSet(&StatusReg, 4);
					break;
    		}
    		if (angle[2] >= 0.0)
    			bitClr(&StatusReg, 5);
    		else
    			bitSet(&StatusReg, 5);
    		tmp = angle[2]/LEVEL_ANGLE;
			if (tmp < 0) tmp = tmp * -1;
			if (tmp > 3) tmp = 3;
    		switch(tmp) {
    			case 0:
    				bitClr(&StatusReg, 6);
    				bitClr(&StatusReg, 7);
    				break;
    			case 1:
    				bitSet(&StatusReg, 6);
    				bitClr(&StatusReg, 7);
					break;
    			case 2:
    				bitClr(&StatusReg, 6);
    				bitSet(&StatusReg, 7);
					break;
    			case 3:
    				bitSet(&StatusReg, 6);
    				bitSet(&StatusReg, 7);
					break;
    		}
     	}
    }
}
