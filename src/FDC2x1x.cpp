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
    @brief Initalisize FDC2x1x Device
    @param i2caddr I2C Adress [0x2A DEFAULT]
    @param wirePort I2C Port of Device
*/
/**************************************************************************/
bool FDC2214::begin(uint8_t i2caddr, TwoWire &wirePort, uint32_t i2cSpeed)
{
  _i2cAddr = i2caddr;
  _i2cPort = &wirePort;

  _i2cPort->begin();
  _i2cPort->setClock(i2cSpeed);

  _i2cPort->beginTransmission(_i2cAddr);

  uint8_t error = _i2cPort->endTransmission();

  if(error == 0)
  {
	  return true;           //Device online!
  }
  else 
  {
    return false;          //Device not attached?
  }
}


uint16_t FDC2214::readRegister(uint8_t Register)
{
  uint16_t data;

  Wire.beginTransmission(_i2cAddr);
  Wire.write(Register);
  Wire.endTransmission(false);
  Wire.requestFrom(_i2cAddr, static_cast<uint8_t>(2));
  data = Wire.read() & 0xFF;
  data <<= 8;
  data |= Wire.read() & 0xFF;
}