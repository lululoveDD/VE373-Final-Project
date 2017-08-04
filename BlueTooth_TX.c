#include <p32xxxx.h>
#include <plib.h>
#include "FlexSensor.c"

//#define BRATE 51 // 38400 Bd (BREGH=1)
#define BRATE 103 // 9600 Bd (BREGH=1)
static int sw = 0; //switch
int readRD6;
int cnt = 0;

#pragma interrupt CN_ISR ipl5 vector 26
void CN_ISR(void){
	IEC1CLR = 0x0001;
	if (readRD6 != PORTDbits.RD6)
		if (sw){ //falling edge
			sw = 0;
		} else { //rising edge
			U1TXREG = 'A';
			U1STAbits.UTXEN = 1;
		}
	readRD6 = PORTDbits.RD6;
	IFS1CLR = 0x0001;
	IEC1SET = 0x0001;
}

void initUART(void){
	asm("di");

	U1BRG = BRATE;
	U1MODEbits.BRGH = 1;
	U1STA = 0;
	U1STAbits.URXEN = 1;
	U1MODEbits.ON = 1;

	asm("ei");
}


void initCN(){
	asm("di");
	TRISDSET = 0x40;
	CNCON = 0x8000;
	CNEN = 0x8000;
	CNPUE = 0x8000;

	readRD6 = PORTDbits.RD6;

	IPC6SET = 0x00140000;
	IPC6SET = 0x00030000;
	IFS1CLR = 0x0001;
	IEC1SET = 0x0001;

	asm("ei");
}

main(){
	OSCSetPBDIV (OSC_PB_DIV_1);	//configure PBDIV so PBCLK = SYSCLK
	initIntGlobal();
	initUART();
	initCN();
	while (1){
		//U1TXREG = 'A';
		//U1STAbits.UTXEN = 1;
	}
}
