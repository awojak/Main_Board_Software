/**
 *	Modified 06.11.2019
 *
 *	Functions that do amazing things :)
 *	Author: Adrian Wojak
 *
 */

#include "tools.h"

/**
 * Function convert integer number to text
 * int number - number to convert
 * char *text - pointer for array where text will be save,
 *  			should have enough space to save whole number max 12 bytes with sign and \0.
 * return - number of digit with sign but without \0
 */
int IntToChar(int number, char *text)
{
	int size = 0, dig = 0, tmp = 0;

	if(number == 0)
	{
		*text++ = '0';
		*text = '\0';
		return 1;
	}

	if(number < 0)
	{
		//add minus sign
		*text++ = '-';
		size++;
		//change sign, need for algorithm
		number *= -1;
	}

    //looking for digit count
    tmp = number;
    while(tmp>0)
    {
        tmp/=10;
        dig++;
    }
	*(text+dig) = '\0';
    dig--;
    //Change number to char
	while(number>0)
	{
		*(text+dig--)=0x30+(number%10);
		number/=10;
		size++;
	}

	return size;
}

/**
 * Need check and update
 */
uint8_t BCDToNumber(uint8_t n) {
	return ((((uint16_t)(n&0xF0)*10)>>4)+(n&0x0F));
}
