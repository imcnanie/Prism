/*
 * prismFirmware.c
 *
 * Created: 6/30/2013 12:23:25 PM
 *  Author: Ian

BLACK: 8-LED PWM(TBS)
BROWN: 7-SCLK SCK
RED: 6-DN (MOSI) PB2
ORANGE: 5-D/C PB6
YELLOW: 4-RST PB4
GREEN: 3-SCE PB0
BLUE: 2-GND GND
PURPLE: 1-VCC +3.3v

0x21,   //switch to extended commands
0xE0,   //set value of Vop (controls contrast) = 0x80 | 0x60 (arbitrary)
0x04,   //set temperature coefficient
0x14,   //set bias mode to 1:48.
0x20,   //switch back to regular commands
0x0C,   //enable normal display (dark on light), horizontal addressing

0b00001101 // inverted display
 */ 


#include <avr/io.h>
#include <util/delay.h>
#include <math.h>
#include <avr/interrupt.h>
#include <string.h>

#define set_bit(address,bit) (address |= (1<<bit))
#define clear_bit(address,bit) (address &= ~(1<<bit))
#define toggle_bit(address,bit) (address ^= (1<<bit))
#define check_bit(address,bit) ((address & (1<<bit)) == (1<<bit))

#define LCD_X     84
#define LCD_Y     48

#define PIN_SCE   0 //Pin 3 on LCD
#define PIN_RESET 4 //Pin 4 on LCD
#define PIN_DC    6 //Pin 5 on LCD
#define PIN_SDIN  2 //Pin 6 on LCD
#define PIN_SCLK  1 //Pin 7 on LCD

#define LCD_COMMAND 0
#define LCD_DATA  1

typedef uint8_t byte;

int output = 0;

uint16_t ADC1;
uint16_t ADC2;

static const byte ASCII[][5] = {
{0x00, 0x00, 0x00, 0x00, 0x00} // 20
,{0x00, 0x00, 0x5f, 0x00, 0x00} // 21 !
  ,{0x00, 0x07, 0x00, 0x07, 0x00} // 22 "
  ,{0x14, 0x7f, 0x14, 0x7f, 0x14} // 23 #
  ,{0x24, 0x2a, 0x7f, 0x2a, 0x12} // 24 $
  ,{0x23, 0x13, 0x08, 0x64, 0x62} // 25 %
  ,{0x36, 0x49, 0x55, 0x22, 0x50} // 26 &
  ,{0x00, 0x05, 0x03, 0x00, 0x00} // 27 '
  ,{0x00, 0x1c, 0x22, 0x41, 0x00} // 28 (
  ,{0x00, 0x41, 0x22, 0x1c, 0x00} // 29 )
  ,{0x14, 0x08, 0x3e, 0x08, 0x14} // 2a *
  ,{0x08, 0x08, 0x3e, 0x08, 0x08} // 2b +
  ,{0x00, 0x50, 0x30, 0x00, 0x00} // 2c ,
  ,{0x08, 0x08, 0x08, 0x08, 0x08} // 2d -
  ,{0x00, 0x60, 0x60, 0x00, 0x00} // 2e .
  ,{0x20, 0x10, 0x08, 0x04, 0x02} // 2f /
  ,{0x3e, 0x51, 0x49, 0x45, 0x3e} // 30 0
  ,{0x00, 0x42, 0x7f, 0x40, 0x00} // 31 1
  ,{0x42, 0x61, 0x51, 0x49, 0x46} // 32 2
  ,{0x21, 0x41, 0x45, 0x4b, 0x31} // 33 3
  ,{0x18, 0x14, 0x12, 0x7f, 0x10} // 34 4
  ,{0x27, 0x45, 0x45, 0x45, 0x39} // 35 5
  ,{0x3c, 0x4a, 0x49, 0x49, 0x30} // 36 6
  ,{0x01, 0x71, 0x09, 0x05, 0x03} // 37 7
  ,{0x36, 0x49, 0x49, 0x49, 0x36} // 38 8
  ,{0x06, 0x49, 0x49, 0x29, 0x1e} // 39 9
  ,{0x00, 0x36, 0x36, 0x00, 0x00} // 3a :
  ,{0x00, 0x56, 0x36, 0x00, 0x00} // 3b ;
  ,{0x08, 0x14, 0x22, 0x41, 0x00} // 3c <
  ,{0x14, 0x14, 0x14, 0x14, 0x14} // 3d =
  ,{0x00, 0x41, 0x22, 0x14, 0x08} // 3e >
  ,{0x02, 0x01, 0x51, 0x09, 0x06} // 3f ?
  ,{0x32, 0x49, 0x79, 0x41, 0x3e} // 40 @
  ,{0x7e, 0x11, 0x11, 0x11, 0x7e} // 41 A
  ,{0x7f, 0x49, 0x49, 0x49, 0x36} // 42 B
  ,{0x3e, 0x41, 0x41, 0x41, 0x22} // 43 C
  ,{0x7f, 0x41, 0x41, 0x22, 0x1c} // 44 D
  ,{0x7f, 0x49, 0x49, 0x49, 0x41} // 45 E
  ,{0x7f, 0x09, 0x09, 0x09, 0x01} // 46 F
  ,{0x3e, 0x41, 0x49, 0x49, 0x7a} // 47 G
  ,{0x7f, 0x08, 0x08, 0x08, 0x7f} // 48 H
  ,{0x00, 0x41, 0x7f, 0x41, 0x00} // 49 I
  ,{0x20, 0x40, 0x41, 0x3f, 0x01} // 4a J
  ,{0x7f, 0x08, 0x14, 0x22, 0x41} // 4b K
  ,{0x7f, 0x40, 0x40, 0x40, 0x40} // 4c L
  ,{0x7f, 0x02, 0x0c, 0x02, 0x7f} // 4d M
  ,{0x7f, 0x04, 0x08, 0x10, 0x7f} // 4e N
  ,{0x3e, 0x41, 0x41, 0x41, 0x3e} // 4f O
  ,{0x7f, 0x09, 0x09, 0x09, 0x06} // 50 P
  ,{0x3e, 0x41, 0x51, 0x21, 0x5e} // 51 Q
  ,{0x7f, 0x09, 0x19, 0x29, 0x46} // 52 R
  ,{0x46, 0x49, 0x49, 0x49, 0x31} // 53 S
  ,{0x01, 0x01, 0x7f, 0x01, 0x01} // 54 T
  ,{0x3f, 0x40, 0x40, 0x40, 0x3f} // 55 U
  ,{0x1f, 0x20, 0x40, 0x20, 0x1f} // 56 V
  ,{0x3f, 0x40, 0x38, 0x40, 0x3f} // 57 W
  ,{0x63, 0x14, 0x08, 0x14, 0x63} // 58 X
  ,{0x07, 0x08, 0x70, 0x08, 0x07} // 59 Y
  ,{0x61, 0x51, 0x49, 0x45, 0x43} // 5a Z
  ,{0x00, 0x7f, 0x41, 0x41, 0x00} // 5b [
  ,{0x02, 0x04, 0x08, 0x10, 0x20} // 5c /* \ */
  ,{0x00, 0x41, 0x41, 0x7f, 0x00} // 5d ]
  ,{0x04, 0x02, 0x01, 0x02, 0x04} // 5e ^
  ,{0x40, 0x40, 0x40, 0x40, 0x40} // 5f _
  ,{0x00, 0x01, 0x02, 0x04, 0x00} // 60 `
  ,{0x20, 0x54, 0x54, 0x54, 0x78} // 61 a
  ,{0x7f, 0x48, 0x44, 0x44, 0x38} // 62 b
  ,{0x38, 0x44, 0x44, 0x44, 0x20} // 63 c
  ,{0x38, 0x44, 0x44, 0x48, 0x7f} // 64 d
  ,{0x38, 0x54, 0x54, 0x54, 0x18} // 65 e
  ,{0x08, 0x7e, 0x09, 0x01, 0x02} // 66 f
  ,{0x0c, 0x52, 0x52, 0x52, 0x3e} // 67 g
  ,{0x7f, 0x08, 0x04, 0x04, 0x78} // 68 h
  ,{0x00, 0x44, 0x7d, 0x40, 0x00} // 69 i
  ,{0x20, 0x40, 0x44, 0x3d, 0x00} // 6a j 
  ,{0x7f, 0x10, 0x28, 0x44, 0x00} // 6b k
  ,{0x00, 0x41, 0x7f, 0x40, 0x00} // 6c l
  ,{0x7c, 0x04, 0x18, 0x04, 0x78} // 6d m
  ,{0x7c, 0x08, 0x04, 0x04, 0x78} // 6e n
  ,{0x38, 0x44, 0x44, 0x44, 0x38} // 6f o
  ,{0x7c, 0x14, 0x14, 0x14, 0x08} // 70 p
  ,{0x08, 0x14, 0x14, 0x18, 0x7c} // 71 q
  ,{0x7c, 0x08, 0x04, 0x04, 0x08} // 72 r
  ,{0x48, 0x54, 0x54, 0x54, 0x20} // 73 s
  ,{0x04, 0x3f, 0x44, 0x40, 0x20} // 74 t
  ,{0x3c, 0x40, 0x40, 0x20, 0x7c} // 75 u
  ,{0x1c, 0x20, 0x40, 0x20, 0x1c} // 76 v
  ,{0x3c, 0x40, 0x30, 0x40, 0x3c} // 77 w
  ,{0x44, 0x28, 0x10, 0x28, 0x44} // 78 x
  ,{0x0c, 0x50, 0x50, 0x50, 0x3c} // 79 y
  ,{0x44, 0x64, 0x54, 0x4c, 0x44} // 7a z
  ,{0x00, 0x08, 0x36, 0x41, 0x00} // 7b {
  ,{0x00, 0x00, 0x7f, 0x00, 0x00} // 7c |
  ,{0x00, 0x41, 0x36, 0x08, 0x00} // 7d }
  ,{0x10, 0x08, 0x08, 0x10, 0x08} // 7e ~
  ,{0x78, 0x46, 0x41, 0x46, 0x78} // 7f DEL
};

//Display Buffer
unsigned char FrameBuffer[504];

/*
unsigned char FrameBuffer[504];
void ClearBuffer();


void WriteOutBuffer();
void SetPixel(int x, int y);
*/

int main(void)
{
	int timer = 0;
	int x;
	int y;

	char str1[16];
	char str2[16];
	
	clear_bit(DDRD,5);
	clear_bit(DDRD,4);
	
	set_bit(DDRB,7);
	
	set_bit(PORTD,5);
	set_bit(PORTD,4);
	set_bit(PORTB,7);

	LCDInit();
	ADCInit();

	sei();

	LCDClear();

    while(1)
    {

		timer ++;

		if (timer >= 1000) {

			timer = 0;

			if(!check_bit(PIND,5))
			{

			}

			if(!check_bit(PIND,4))
			{

			}			

		}

	}

}

//LCD Buffered Functions
void ClearBuffer(void) {
	for (int index = 0 ; index < (LCD_X * LCD_Y / 8) ; index++)
	FrameBuffer[index] = 0x00;
}

void WriteOutBuffer(void) {
	for (int index = 0 ; index < (LCD_X * LCD_Y / 8) ; index++)
	FrameBuffer[index] = 0x00;
}

//LCD Unbuffered Functions
drawToB(int x0, int y0, int x1, int y1) {
	uint8_t steep = abs(y1 - y0) > abs(x1 - x0);
	uint8_t dx, dy;
	int8_t err;
	int8_t ystep;

	if (steep) {
		swap(x0, y0);
		swap(x1, y1);
	}

	if (x0 > x1) {
		swap(x0, x1);
		swap(y0, y1);
	}

	dx = x1 - x0;
	dy = abs(y1 - y0);

	err = dx / 2;

	if (y0 < y1) {
		ystep = 1;
	} else {
		ystep = -1;
	}

	for (; x0<=x1; x0++) {
		if (steep) {
			writePixel(y0, x0);
		} else {
			writePixel(x0, y0);
		}
		err -= dy;
		if (err < 0) {
			y0 += ystep;
			err += dx;
		}
	}
}

drawTo(int x1, int y1, int x2, int y2) {

	int dx;
	int dy;

	int yax = 0;
	int xax = 0;

	dx = x2 - x1;
	dy = y2 - y1;

	for (xax = x1; xax >= x1 && xax <= x2; xax++) {
		xax++;
		yax = y1 + (dy) * (xax - x1)/(dx);

		writePixel(xax,yax);

		if (xax > x2) {
			break;
		}
	}
}


swap(int x, int y) {
	int a;
	int b;

	a = x;
	b = y;

	x = b;
	y = a;
}

writePixel(int x, int y) {

	int q;

	if (y >= 0 && y <= 7) {
		gotoXY(x,0);
		q = y - 0;
	}

	if (y > 7 && y <= 15) {
		gotoXY(x,1);
		q = y - 8;
	}

	if (y > 15 && y <= 23) {
		gotoXY(x,2);
		q = y - 16;
	}

	if (y > 23 && y <= 31) {
		gotoXY(x,3);
		q = y - 24;
	}

	if (y > 31 && y <= 39) {
		gotoXY(x,4);
		q = y - 32;
	}

	if (y > 39 && y <= 47) {
		gotoXY(x,5);
		q = y - 40;
	}

	LCDData(1 << q);
}

//normal write function breaks when drawing y lines (bug)
writeY(int x, int y) {

	int q;
	int a = 0;

	if (y >= 0 && y <= 7) {
		gotoXY(x,0);
		q = y - 0;
	}

	if (y > 7 && y <= 15) {
		gotoXY(x,1);
		q = y - 8;
	}

	if (y > 15 && y <= 23) {
		gotoXY(x,2);
		q = y - 16;
	}

	if (y > 23 && y <= 31) {
		gotoXY(x,3);
		q = y - 24;
	}

	if (y > 31 && y <= 39) {
		gotoXY(x,4);
		q = y - 32;
	}

	if (y > 39 && y <= 47) {
		gotoXY(x,5);
		q = y - 40;
	}


	if (q == 0)
	LCDData(0xFF);

	if (q == 1)
	LCDData(0xFF);

	if (q == 2)
	LCDData(0xFF);

	if (q == 3)
	LCDData(0xFF);

	if (q == 4)
	LCDData(0xFF);

	if (q == 5)
	LCDData(0xFF);

	if (q == 6)
	LCDData(0xFF);

	if (q == 7)
	LCDData(0xFF);

}

void LCDCharacter(char character) {
	LCDData(0x00);

	//for (int index = 0 ; index < 5 ; index++)
	//LCDData(ASCII[character - 0x20][index]);

	for (int index = 0 ; index < 5 ; index++)
	//Reverse Bits on ASCII table. Mirrors characters, LCDDataChar sends LSB.
	LCDDataChar(reverse(ASCII[character - 0x20][4-index]));

	LCDData(0x00);
}

void LCDString(char *characters) {
	strrev(characters);
	
	while (*characters) {
		
		LCDCharacter(*characters++);
	}	
}

void LCDBitmap(char my_array[]){
	for (int index = 0 ; index < (LCD_X * LCD_Y / 8) ; index++)
	LCDData(my_array[index]);
}

void LCDClear(void) {
	for (int index = 0 ; index < (LCD_X * LCD_Y / 8) ; index++)
	LCDData(0x00);

	gotoXY(0, 0);
}

void gotoXY(int x, int y) {
	LCDCommand(0x80 | x);
	LCDCommand(0x40 | y); 
}


//LCD Init and Data Functions
void LCDInit(void) {

	set_bit(DDRB,PIN_SCE);
	set_bit(DDRB,PIN_RESET);
	set_bit(DDRB,PIN_DC);
	set_bit(DDRB,PIN_SDIN);
	set_bit(DDRB,PIN_SCLK);

	set_bit(DDRF,5);

	//PRRO write to 0
	PRR0 = (0<<PRSPI);

	// Enable SPI, Master, set clock rate fck/16
	SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0);

	//Reset the LCD to a known state
	clear_bit(PORTB, PIN_RESET);
	set_bit(PORTB, PIN_RESET);

	LCDCommand(0x21); //Tell LCD that extended commands follow
	LCDCommand(0xB1); //Set LCD Vop (Contrast): Try 0xB1(good @ 3.3V) or 0xBF if your display is too dark
	LCDCommand(0x04); //Set Temp coefficent
	LCDCommand(0x14); //LCD bias mode 1:48: Try 0x13 or 0x14

	LCDCommand(0x20); //We must send 0x20 before modifying the display control mode
	LCDCommand(0x0C); //Set display control, normal mode. 0x0D for inverse
}

void LCDCommand(byte data) {
	clear_bit(PORTB,6);
	clear_bit(PORTB, PIN_SCE);

	/* Start transmission */
	SPDR = data;
	/* Wait for transmission complete */
	while(!(SPSR & (1<<SPIF)))
	;

	set_bit(PORTB, PIN_SCE);
}

void LCDData(byte data) {
	set_bit(PORTB,6);
	clear_bit(PORTB, PIN_SCE);

	/* Start transmission */
	SPDR = data;
	/* Wait for transmission complete */
	while(!(SPSR & (1<<SPIF)))
	;

	set_bit(PORTB, PIN_SCE);
}

void LCDDataChar(byte data) {
	set_bit(PORTB,6);
	clear_bit(PORTB, PIN_SCE);
	set_bit(SPCR, DORD); //LSB First

	/* Start transmission */
	SPDR = data;
	/* Wait for transmission complete */
	while(!(SPSR & (1<<SPIF)))
	;

	clear_bit(SPCR, DORD);
	set_bit(PORTB, PIN_SCE);
}

void reverse(uint8_t n) {
	n = ((n >> 1) & 0x55) | ((n << 1) & 0xaa);
	n = ((n >> 2) & 0x33) | ((n << 2) & 0xcc);
	n = ((n >> 4) & 0x0f) | ((n << 4) & 0xf0);
}

//ADC Init and Functions
mapData(int x, int in_min, int in_max, int out_min, int out_max) {
	output = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

ADCInit(void) {

	ADCSRA |= 1<<ADPS2;

	ADMUX |= 1<<REFS0 | 1<<REFS1;

	ADCSRA |= 1<<ADIE;
	ADCSRA |= 1<<ADEN;
	ADCSRA |= 1<<ADSC;

	ADMUX = 0xC4;

}

ISR(ADC_vect)
{
	uint8_t theLow = ADCL;
	uint16_t theTenBitResult = ADCH<<8 | theLow;

	switch (ADMUX)
	{
		case 0xC5:

			ADC1 = theTenBitResult;
			ADMUX = 0xC4;
			ADCSRA |= 1<<ADSC;
		break;

		case 0xC4:

			ADC2 = theTenBitResult;
			ADMUX = 0xC5;
			ADCSRA |= 1<<ADSC;
		break;

		default:

		break;
	}
}