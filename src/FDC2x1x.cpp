/**************************************************************************/
/*!
    @file     FDC2214.cpp
    This is a library for the FDC2x1x, a EMI-Resistant 28-Bit/12-Bit 
    high-resolution, multichannel Capacitance-To-Digital Converter 
    for Proximity and Level Sensing Applications from Texas Instruments.
    Written by Fabian Voelker for promesstec GmbH.
    MIT license, all text here must be included in any redistribution.
*/
/**************************************************************************/


#include "FDC2x1x.h"

#ifdef __AVR__
#include <avr/pgmspace.h>
#elif defined(ESP8266)
#include <pgmspace.h>
#elif defined(ARDUINO_ARCH_SAMD)
// nothing special needed
#elif defined(ARDUINO_SAM_DUE)
#define PROGMEM
#define pgm_read_byte(addr) (*(const unsigned char *)(addr))
#endif



/**************************************************************************/
/*!
    @brief Write value to register.
    @param i2caddr register address
    @param wirePort value to write
*/
/**************************************************************************/
bool FDC2214::begin(uint8_t i2caddr, TwoWire &wirePort)
{
  
}