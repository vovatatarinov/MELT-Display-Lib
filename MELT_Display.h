/*
*
* MELT_Display.h file
* Author: Vladimir Tatarinov
* March, 2022
* Display library for MELT LCD display for serial(!) 4-bit mode
* VO and R/W pins are connected to GND
*
* Notice: Please define your F_CPU in your compiler for correct delay times
*/

void sendData2LCD(uint8_t dataByte, uint8_t IsSymbol);

void setup_MELT_Display(volatile uint8_t* CS_port, uint8_t CS_pin);

void sendText(char* str);

 
