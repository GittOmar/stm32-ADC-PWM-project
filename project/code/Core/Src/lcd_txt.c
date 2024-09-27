#include "lcd_txt.h"
#include  <stdio.h>
#include <math.h>


/*--------------- Initialize LCD ------------------*/
void lcd_init(void)
{

	HAL_Delay(30);

	PIN_LOW(D4_PORT,D4_PIN);
	PIN_HIGH(D5_PORT,D5_PIN);
	PIN_LOW(D6_PORT,D6_PIN);
	PIN_LOW(D7_PORT,D7_PIN);
	PIN_LOW(RS_PORT,RS_PIN);

	PIN_HIGH(EN_PORT,EN_PIN);
	PIN_LOW(EN_PORT,EN_PIN);

	lcd_write(0,0x28);
	lcd_write(0,0x0c);
	lcd_write(0,0x06);
	lcd_write(0,0x01);
}

/*--------------- Write To LCD ---------------*/
void lcd_write(uint8_t type,uint8_t data)
{
	HAL_Delay(2);
	if(type)
	{
		PIN_HIGH(RS_PORT,RS_PIN);
	}else
	{
		PIN_LOW(RS_PORT,RS_PIN);
	}

	//Send High Nibble
	if(data&0x80)
	{
		PIN_HIGH(D7_PORT,D7_PIN);
	}else
	{
		PIN_LOW(D7_PORT,D7_PIN);
	}

	if(data&0x40)
	{
		PIN_HIGH(D6_PORT,D6_PIN);
	}else
	{
		PIN_LOW(D6_PORT,D6_PIN);
	}

	if(data&0x20)
	{
		PIN_HIGH(D5_PORT,D5_PIN);
	}else
	{
		PIN_LOW(D5_PORT,D5_PIN);
	}

	if(data&0x10)
	{
		PIN_HIGH(D4_PORT,D4_PIN);
	}else
	{
		PIN_LOW(D4_PORT,D4_PIN);
	}
	PIN_HIGH(EN_PORT,EN_PIN);
	PIN_LOW(EN_PORT,EN_PIN);


	//Send Low Nibble
	if(data&0x08)
	{
		PIN_HIGH(D7_PORT,D7_PIN);
	}else
	{
		PIN_LOW(D7_PORT,D7_PIN);
	}

	if(data&0x04)
	{
		PIN_HIGH(D6_PORT,D6_PIN);
	}else
	{
		PIN_LOW(D6_PORT,D6_PIN);
	}

	if(data&0x02)
	{
		PIN_HIGH(D5_PORT,D5_PIN);
	}else
	{
		PIN_LOW(D5_PORT,D5_PIN);
	}

	if(data&0x01)
	{
		PIN_HIGH(D4_PORT,D4_PIN);
	}else
	{
		PIN_LOW(D4_PORT,D4_PIN);
	}
	PIN_HIGH(EN_PORT,EN_PIN);
	PIN_LOW(EN_PORT,EN_PIN);
}

void lcd_puts_s(uint8_t x, uint8_t y, int8_t *string)
{

	//Set Cursor Position
	#ifdef LCD16xN	//For LCD16x2 or LCD16x4
	switch(y)
	{
	        case 0: //Row 0
				lcd_write(0,0x80+0x00+x);
				break;
			case 1: //Row 1
				lcd_write(0,0x80+0x40 +x);
				break;
			case 2: //Row 2
				lcd_write(0,0x80+0x14 +x);
				break;
			case 3: //Row 3
				lcd_write(0,0x80+0x54 +x);
				break;
	}
	#endif


	while(*string)
	{
		lcd_write(1,*string);
		string++;
	}
}
void lcd_puts_d(uint8_t x, uint8_t y, uint8_t precision, float num)
{

	 int  first  = num;
	 int  digits = ( log10(first)+1  >  1  ?  log10(first)+1  :  1  ) ;
	 int  second = ( num - first ) * pow( 10 , precision ) ;
     char buffer[16];

	 sprintf(buffer, "%u",  first);
	 lcd_puts_s(x,y,(int8_t*) buffer);

	 if(precision)
	 {
		     lcd_puts_s( x+1 , y ,(int8_t*) ".");

			 sprintf(buffer, "%u",  second);
			 lcd_puts_s(x+1+digits,y,(int8_t*)buffer);
	 }

}
void lcd_clear(void)
{
	lcd_write(0,0x01);
}



