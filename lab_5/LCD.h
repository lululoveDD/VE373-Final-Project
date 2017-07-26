/************************************************************************
* LCD.h
* Header file for the LCD Driver
************************************************************************/
#include <p32xxxx.h>

/* define macros for LCD instructions */
#define LCD_IDLE 0x33
#define LCD_2_LINE_4_BITS 0x28
#define LCD_2_LINE_8_BITS 0x38
#define LCD_DSP_CSR 0x0c
#define LCD_CLR_DSP 0x01
#define LCD_CSR_INC 0x06
#define LCD_SFT_MOV 0x14

/* define macros for interfacing ports */
#define RS PORTDbits.RD0
#define E PORTDbits.RD1
#define Data PORTE

typedef unsigned char uchar;

struct bits {
	unsigned timer2_done : 1;
	//...other user defined flags may go here
} flags;

/* Function prototypes */
void MCU_init(void);
void LCD_init(void);
void LCD_putchar(uchar c);
void LCD_puts(const uchar *s);
void LCD_goto(uchar addr);
void DelayUsec(uchar num);
void DelayMsec(uchar num);
/*****************end of LCD.h**********************/

