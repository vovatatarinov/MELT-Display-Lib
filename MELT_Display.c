/*
*
* MELT_Display.c file
* Author: Vladimir Tatarinov
* March, 2022
* Display library for MELT LCD display for serial(!) 4-bit mode through shift register (74HC595)
* VO and R/W pins are connected to GND
*
* Notice: Please define your F_CPU in your compiler for correct delay times
*/
#include <avr/io.h>
#include <util/delay.h>
#include "inOuts.h"

#define DL_LCD_SWITCH_ON_MS 200                                            //delay after powerup in ms
#define DL_LCD_BETWEEN_E_CHANGE_MCS 40                                     //delay between changes of E state in us
#define DL_LCD_DATA_SEND_MS 2                                             //delay after data send in ms

static inOuts pinCS;
static uint8_t data2sr;

void sendData2LCD(uint8_t dataByte, uint8_t IsSymbol) {
    /* Shift Register Outputs: */
    /* Q7 Q6 Q5 Q4 Q3 Q2 Q1 Q0 */
    /* Display Inputs */
    /* -  -  E  RS D7 D6 D5 D4 */
    if (IsSymbol)
        IsSymbol = 1;
    else 
        IsSymbol = 0;
    
    setIO(pinCS, 0); //Pull LATCH low (Important: this is necessary to start the SPI transfer!)
    
    // Set pin E to low level. Need to make sure pin E is low.
    data2sr = 0x00;
    SPDR = data2sr;  //Shift in some data
    while(!(SPSR & (1<<SPIF)));  //Wait for SPI process to finish
    //Toggle latch to copy data to the storage register
    setIO(pinCS, 1);
    setIO(pinCS, 0);
    
    _delay_us(DL_LCD_BETWEEN_E_CHANGE_MCS);
    data2sr = 0x00;

    //IsSymbol to RS pin, pin E to High , send first half-byte;
    data2sr |= ( (1 << 5) | (IsSymbol << 4) | ((dataByte >> 4) & 0x0F) );
    
    SPDR = data2sr;  //Shift in some data
    while(!(SPSR & (1<<SPIF)));  //Wait for SPI process to finish
    //Toggle latch to copy data to the storage register
    setIO(pinCS, 1);
    setIO(pinCS, 0);
    
    _delay_us(DL_LCD_BETWEEN_E_CHANGE_MCS);
    
    data2sr = 0x00;
     //IsSymbol to RS pin, pin E to Low , send first half-byte;
    data2sr |= ( (0 << 5) | (IsSymbol << 4) | ((dataByte >> 4) & 0x0F) );
    
    SPDR = data2sr;  //Shift in some data
    while(!(SPSR & (1<<SPIF)));  //Wait for SPI process to finish
    //Toggle latch to copy data to the storage register
    setIO(pinCS, 1);
    setIO(pinCS, 0);
    
    _delay_us(DL_LCD_BETWEEN_E_CHANGE_MCS);
    
    data2sr = 0x00;
    //IsSymbol to RS pin, pin E to High , send first half-byte;
    data2sr |= ( (1 << 5) | (IsSymbol << 4) | (dataByte & 0x0F) );
    
    SPDR = data2sr;  //Shift in some data
    while(!(SPSR & (1<<SPIF)));  //Wait for SPI process to finish
    //Toggle latch to copy data to the storage register
    setIO(pinCS, 1);
    setIO(pinCS, 0);
    
    _delay_us(DL_LCD_BETWEEN_E_CHANGE_MCS);
    
    data2sr = 0x00;
    //IsSymbol to RS pin, pin E to High , send first half-byte;
    data2sr |= ( (0 << 5) | (IsSymbol << 4) | (dataByte & 0x0F) );
    
    SPDR = data2sr;  //Shift in some data
    while(!(SPSR & (1<<SPIF)));  //Wait for SPI process to finish
    //Toggle latch to copy data to the storage register
    setIO(pinCS, 1);
    setIO(pinCS, 0);
    _delay_us(DL_LCD_BETWEEN_E_CHANGE_MCS);
    data2sr = 0x00;
    SPDR = data2sr;  //Shift in some data
    while(!(SPSR & (1<<SPIF)));  //Wait for SPI process to finish
    //Toggle latch to copy data to the storage register
    setIO(pinCS, 1);
    setIO(pinCS, 0);
    _delay_ms(DL_LCD_DATA_SEND_MS);
}

void setup_MELT_Display(volatile uint8_t* CS_port, uint8_t CS_pin)
{
    /*
    * Init inOuts structs
    */
    inOutsInit(&pinCS, CS_port, CS_pin);
    setOutputMode(pinCS);
    /*
    * Delay after possible power up
    */
    _delay_ms(DL_LCD_SWITCH_ON_MS);
    uint8_t bInit[] = {0b00110011,                                                   //send commands for init
                    0b00110010,
                    0b00101000,
                    0b00001100,
                    0b00000001,
                    0b00000110};
    for (uint8_t i = 0; i < 6; ++i) {
        sendData2LCD(bInit[i], 0);
    }
}

void clrscr() {
    sendData2LCD(1, 0);
}

void sendText(char* str) {
    while(*str != '\0') {
        sendData2LCD(*str, 1);
        ++str;
    }

}
 
