#include <p32xxxx.h>
#include <plib.h>

#define BRATE_BT 103 // 9600 Bd (BREGH=1)
#define BRATE_AS 103 // 9600 Bd (BREGH=1)

#pragma interrupt UART_RXISR ipl6 vector 24
void UART_RXISR(void)
{

	/*
		Dealing with the data
	*/

	IFS0bits.U1RXIF = 0;
}

/*----------------------------------------------
	UART config
	U1 for BlueTooth
		9600Bd, 8-bit data, no parity, 1 Stop bit
	U2 for Angle Sensor
		9600Bd, 8-bit data, no parity, 1 Stop bit
		Interrupt 
-----------------------------------------------*/
void initUART(void){
	asm("di");

	U1BRG = BRATE_BT;
	U1MODEbits.BRGH = 1;
	U1STA = 0;
	//U1STAbits.URXEN = 1;

	U2BRG = BRATE_AS;
	U2MODEbits.BRGH = 1;
	U2STAbits.URXEN = 1;
	U2STA = 0;
	IFS0bits.U2RXIF = 0;
	IEC0bits.U2RXIE = 1;
	IPC6bits.U2IP = 6;
	IPC6bits.U2IS = 3;

	U1MODEbits.ON = 1;
	U2MODEbits.ON = 1;

	asm("ei");
}

void initIntGlobal() {
	DDPCONbits.JTAGEN = 0;
  	TRISFbits.TRISF2 = 1;	// RF2/U1RX/J11-41/input
	TRISFbits.TRISF3 = 0;	// RF3/U1TX/J11-43/output
	TRISFbits.TRISF4 = 1;	// RF2/U2RX/J11-46/input
	TRISFbits.TRISF5 = 0;	// RF3/U2TX/J11-48/output
	INTCONbits.MVEC = 1;	// Enable multiple vector interrupt
	asm("ei"); 				// Enable all interrupts
}

main(){
	OSCSetPBDIV (OSC_PB_DIV_1); //configure PBDIV so PBCLK = SYSCLK
	initIntGlobal();
	initUART();
}