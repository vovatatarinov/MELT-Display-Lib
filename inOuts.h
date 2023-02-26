/*
*
* inOuts.h file
* Author: Vladimir Tatarinov
* January, 2022
* Simple GPIO on AVR
* Please be careful using this file!
* For example, it can fail on Atmega128 MCUs for PORTF. There are non-neighbor port registors.
*
*/
#ifndef INOUTS_H
#define INOUTS_H

typedef struct inOuts {
    uint8_t pin;
    volatile uint8_t *portReg;
    //DDRx = portReg - 1
    //PINx = portReg - 2
    volatile uint8_t *dirReg;
    volatile uint8_t *pinReg;
} inOuts;

static inline void inOutsInit(inOuts* inOut, volatile uint8_t *portReg, uint8_t pin) {
    inOut->portReg = portReg;
    inOut->dirReg = portReg - 1;
    inOut->pinReg = portReg - 2;
    inOut->pin = pin;
}

static inline void setOutputMode(inOuts inOut) {
    (*(inOut.dirReg)) |= (1 << inOut.pin);
}

static inline void setInputMode(inOuts inOut) {
    (*(inOut.dirReg)) &= ~(1 << inOut.pin);
}

static inline uint8_t getIO(inOuts inOut) {
    return ((*(inOut.pinReg)) >> inOut.pin) & 0b1;
}

static inline void setIO(inOuts inOut, uint8_t value) {
    if (value != 0)
        (*(inOut.portReg)) |= (1 << inOut.pin);
    else 
        (*(inOut.portReg)) &= ~(1 << inOut.pin);
}

#endif
