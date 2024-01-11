
/*!
    @file     FDC2214.cpp
    This is a library for the FDC2x1x, a EMI-Resistant 28-Bit/12-Bit 
    high-resolution, multichannel Capacitance-To-Digital Converter 
    for Proximity and Level Sensing Applications from Texas Instruments.
    Written by Fabian Voelker for promesstec GmbH.
    MIT license, all text here must be included in any redistribution.
*/


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
	  return true;      //Device online!
  }
  else 
  {
    return false;     //Device not attached?
  }
}


/**************************************************************************/
/*!
    @brief check if the device is Connected
*/
/**************************************************************************/
bool FDC2214::isConnected(void)
{
  if(readRegister(FDC2x1x_DEVICE_ID) == 0x3054 || readRegister(FDC2x1x_DEVICE_ID) == 0x3055)
  {
    return true;
  }
  else
  {
    return false;
  }
}


/**************************************************************************/
/*!
    @brief read 16Bit register from I2C Slave
    @param Register Register of Slave
*/
/**************************************************************************/
uint16_t FDC2214::readRegister(uint8_t Register)
{
  uint16_t DataR;

  _i2cPort->beginTransmission(_i2cAddr);
  _i2cPort->write(Register);
  _i2cPort->endTransmission();
  _i2cPort->beginTransmission(_i2cAddr);
  _i2cPort->requestFrom(_i2cAddr, static_cast<uint8_t>(2));
  DataR = _i2cPort->read();
  DataR <<= 8;
  DataR |= _i2cPort->read();
  _i2cPort->endTransmission();

  return DataR;
}


/**************************************************************************/
/*!
    @brief write a 16Bit register into I2C Slave
    @param Register Register of Slave
    @param DataW Data to Write
*/
/**************************************************************************/
void FDC2214::writeRegister(uint8_t Register, uint16_t DataW)
{
  _i2cPort->beginTransmission(_i2cAddr);
  _i2cPort->write(Register);
  _i2cPort->write( (uint8_t) (DataW >> 8));
  _i2cPort->write( (uint8_t) DataW);
  _i2cPort->endTransmission();
}


/**************************************************************************/
/*!
    @brief gets the status register of the device
*/
/**************************************************************************/
uint16_t FDC2214::getStatus(void)
{
  return readRegister(FDC2x1x_STATUS);
}







/**************************************************************************/
/*!
    @brief Sets the Dividers for specific Channel
    @param channel Channel Selection ( CH0 - CH3 )
    @param sensorFreqSel Differential Config or SingleEnded Config
    @param CHxRefDivider Sets fREFx = fCLK/CHx_FREF_DIVIDER
*/
/**************************************************************************/
void FDC2214::setDividers(uint8_t channel, uint16_t sensorFreqSel, uint16_t CHxRefDivider)
{
  uint16_t DataW;

  if(sensorFreqSel<=2)
  {
    DataW |= (sensorFreqSel << 12);
  }
  if(CHxRefDivider<=1023 && CHxRefDivider>0)
  {
    DataW |= CHxRefDivider;
  }

  switch (channel)
  {
    case (0):
      writeRegister(FDC2x1x_CLOCK_DIVIDERS_CH0,DataW);
		break;
		case (1):
			writeRegister(FDC2x1x_CLOCK_DIVIDERS_CH1,DataW);
		break;
		case (2):
			writeRegister(FDC2x1x_CLOCK_DIVIDERS_CH2,DataW);
		break;
		case (3):
			writeRegister(FDC2x1x_CLOCK_DIVIDERS_CH3,DataW);
		break;
		default:
    break;
  }
}


/**************************************************************************/
/*!
    @brief Sets the Drive Current for specific Channel
    @param channel Channel Selection ( CH0 - CH3 )
    @param CHxIDrive sets Drive Current for Sensor [00000 = 16uA, 11111 = 1.5mA]
    @warning oscillation amplitude must between 1.2V and 1.8V!
             Measure the oscillation amplitude on an oscilloscope 
             and adjust the IDRIVE value
*/
/**************************************************************************/
void FDC2214::setDriveCurrent(uint8_t channel, uint16_t CHxIDrive)
{
  uint16_t DataW;

  if(CHxIDrive<32)
  {
    DataW |= (CHxIDrive << 11);
  }

  switch (channel)
  {
    case (0):
      writeRegister(FDC2x1x_DRIVE_CURRENT_CH0,DataW);
		break;
		case (1):
			writeRegister(FDC2x1x_DRIVE_CURRENT_CH1,DataW);
		break;
		case (2):
			writeRegister(FDC2x1x_DRIVE_CURRENT_CH2,DataW);
		break;
		case (3):
			writeRegister(FDC2x1x_DRIVE_CURRENT_CH3,DataW);
		break;
		default:
    break;
  }
}


/**************************************************************************/
/*!
    @brief Sets the settle time for specific Channel
    @param channel Channel Selection ( CH0 - CH3 )
    @param CHxSettleCount settling time to allow the LC sensor to
                          stabilize before initiation of a conversion
    @warning  If the amplitude has not settled prior to the conversion start, an
              Amplitude warning will be generated if reporting of this type of
              warning is enabled.
*/
/**************************************************************************/
void FDC2214::setSettleCount(uint8_t channel, uint16_t CHxSettleCount)
{
  uint16_t DataW;

  if(CHxSettleCount<=0xFFFF)
  {
    DataW |= CHxSettleCount;
  }

  switch (channel)
  {
    case (0):
      writeRegister(FDC2x1x_SETTLECOUNT_CH0,DataW);
		break;
		case (1):
			writeRegister(FDC2x1x_SETTLECOUNT_CH1,DataW);
		break;
		case (2):
			writeRegister(FDC2x1x_SETTLECOUNT_CH2,DataW);
		break;
		case (3):
			writeRegister(FDC2x1x_SETTLECOUNT_CH3,DataW);
		break;
		default:
    break;
  }
}


/**************************************************************************/
/*!
    @brief Sets the Reference Count for specific Channel
    @param channel Channel Selection ( CH0 - CH3 )
    @param CHxRefCount Reference Count Conversion Interval Time
    @warning  0x0000-0x00FF: Reserved
    @warning  0x0100-0xFFFF: Conversion Time (tC0) = (CH0_RCOUNTˣ16)/fREF0
*/
/**************************************************************************/
void FDC2214::setReferenceCount(uint8_t channel, uint16_t CHxRefCount)
{
  uint16_t DataW;

  if(CHxRefCount>=0x0100 && CHxRefCount<=0xFFFF)
  {
    DataW |= CHxRefCount;
  }

  switch (channel)
  {
    case (0):
      writeRegister(FDC2x1x_RCOUNT_CH0,DataW);
		break;
		case (1):
			writeRegister(FDC2x1x_RCOUNT_CH1,DataW);
		break;
		case (2):
			writeRegister(FDC2x1x_RCOUNT_CH2,DataW);
		break;
		case (3):
			writeRegister(FDC2x1x_RCOUNT_CH3,DataW);
		break;
		default:
    break;
  }
}


/**************************************************************************/
/*!
    @brief Sets the Offset for specific Channel
    @param channel Channel Selection ( CH0 - CH3 )
    @param CHxOffset Conversion Offset [fOFFSET_0 = (CH0_OFFSET/216)*fREF0]
*/
/**************************************************************************/
void FDC2214::setOffset(uint8_t channel, uint16_t CHxOffset)
{
  switch (channel)
  {
    case (0):
      writeRegister(FDC2x1x_OFFSET_CH0,CHxOffset);
		break;
		case (1):
			writeRegister(FDC2x1x_OFFSET_CH1,CHxOffset);
		break;
		case (2):
			writeRegister(FDC2x1x_OFFSET_CH2,CHxOffset);
		break;
		case (3):
			writeRegister(FDC2x1x_OFFSET_CH3,CHxOffset);
		break;
		default:
    break;
  }
}


/**************************************************************************/
/*!
    @brief set deglitch filter
    @param filter set filter frequency
    @brief 1Mhz, 3.3Mhz, 10Mhz, 33Mhz
    @warning  Select the lowest setting that exceeds the oscillation tank
              oscillation frequency.

*/
/**************************************************************************/
void FDC2214::setDeglitchFilter(enum DeglitchFilter filter)
{
  uint16_t DataR = readRegister(FDC2x1x_MUX_CONFIG);

  switch (filter)
  {
    case (FDC2x1x_FILTER_1MHZ):
      DataR &= ~0b111;
      DataR |= 0b001;
      writeRegister(FDC2x1x_MUX_CONFIG,DataR);
		break;
		case (FDC2x1x_FILTER_3MHZ):
      DataR &= ~0b111;
      DataR |= 0b100;
			writeRegister(FDC2x1x_MUX_CONFIG,DataR);
		break;
		case (FDC2x1x_FILTER_10MHZ):
      DataR &= ~0b111;
      DataR |= 0b101;
			writeRegister(FDC2x1x_MUX_CONFIG,DataR);
		break;
    case (FDC2x1x_FILTER_33MHZ):
      DataR &= ~0b111;
      DataR |= 0b111;
			writeRegister(FDC2x1x_MUX_CONFIG,DataR);
		break;
		default:
    break;
  }
}


/**************************************************************************/
/*!
    @brief Enable the autoscan function
    @param en En/Disable autoscan function [true/false]
    @warning if enabled Auto-Scan conversions as selected by
             MUX_CONFIG.RR_SEQUENCE register field
*/
/**************************************************************************/
void FDC2214::enableAutoScan(bool en)
{
  uint16_t DataR = readRegister(FDC2x1x_MUX_CONFIG);

  if(en)
  {
    DataR |= 0x8000;
  }
  else
  {
    DataR &= ~0x8000;
  }
  writeRegister(FDC2x1x_MUX_CONFIG,DataR);
}


/**************************************************************************/
/*!
    @brief set autoscan sequence
    @param sequence sequence
    @brief CH0-CH1, CH0-CH1-CH2, CH0-CH1-CH2-CH3
    @warning  The FDC will perform a single conversion on
              each channel in the sequence selected, and then restart the
              sequence continuously.
*/
/**************************************************************************/
void FDC2214::setRRSequence(enum AutoscanSequence sequence)
{
  uint16_t DataR = readRegister(FDC2x1x_MUX_CONFIG);

  switch (sequence)
  {
    case (FDC2x1x_SEQ_CH0_CH1):
      DataR &= ~0x3000;
      writeRegister(FDC2x1x_MUX_CONFIG,DataR);
		break;
		case (FDC2x1x_SEQ_CH0_CH1_CH2):
      DataR &= ~0x3000;
      DataR |= 0x1000;
			writeRegister(FDC2x1x_MUX_CONFIG,DataR);
		break;
		case (FDC2x1x_SEQ_CH0_CH1_CH2_CH3):
      DataR &= ~0x3000;
      DataR |= 0x2000;
			writeRegister(FDC2x1x_MUX_CONFIG,DataR);
		break;
		default:
    break;
  }
}





/**************************************************************************/
/*!
    @brief Enable the status report of Watchdog Error
    @param en En/Disable Error reporting [true/false]
*/
/**************************************************************************/
void FDC2214::enableWDError(bool en)
{
  uint16_t DataR = readRegister(FDC2x1x_ERROR_CONFIG);

  if(en)
  {
    DataR |= 0x2000;
  }
  else
  {
    DataR &= ~0x2000;
  }
  writeRegister(FDC2x1x_ERROR_CONFIG,DataR);
}


/**************************************************************************/
/*!
    @brief Enable the status report of Amplitude High Error
    @param en En/Disable Error reporting [true/false]
*/
/**************************************************************************/
void FDC2214::enableAmpHighError(bool en)
{
  uint16_t DataR = readRegister(FDC2x1x_ERROR_CONFIG);

  if(en)
  {
    DataR |= 0x1000;
  }
  else
  {
    DataR &= ~0x1000;
  }
  writeRegister(FDC2x1x_ERROR_CONFIG,DataR);
}


/**************************************************************************/
/*!
    @brief Enable the status report of Amplitude Low Error
    @param en En/Disable Error reporting [true/false]
*/
/**************************************************************************/
void FDC2214::enableAmpLowError(bool en)
{
  uint16_t DataR = readRegister(FDC2x1x_ERROR_CONFIG);

  if(en)
  {
    DataR |= 0x800;
  }
  else
  {
    DataR &= ~0x800;
  }
  writeRegister(FDC2x1x_ERROR_CONFIG,DataR);
}


/**************************************************************************/
/*!
    @brief Enable Interrupt Output with specified function
    @param function interrupt function 
    @brief 0=OFF 
    @brief 1=Watchdog Timeout Error 
    @brief 2=Data Ready
*/
/**************************************************************************/
void FDC2214::setINTB(enum IntteruptFunctions functions)
{
  uint16_t DataR0 = readRegister(FDC2x1x_ERROR_CONFIG);
  uint16_t DataR1 = readRegister(FDC2x1x_CONFIG);

  switch (functions)
  {
    case (FDC2x1x_INTB_OFF):
      DataR0 &= ~0x21;
      writeRegister(FDC2x1x_ERROR_CONFIG,DataR0);
      DataR1 &= ~0x80;
      writeRegister(FDC2x1x_CONFIG,DataR1);
		break;
		case (FDC2x1x_INTB_WD):
      DataR0 &= ~0x01;
      DataR0 |= 0x20;
			writeRegister(FDC2x1x_ERROR_CONFIG,DataR0);
      DataR1 |= 0x80;
      writeRegister(FDC2x1x_CONFIG,DataR1);
		break;
		case (FDC2x1x_INTB_DATAREADY):
      DataR0 &= ~0x20;
      DataR0 |= 0x01;
			writeRegister(FDC2x1x_ERROR_CONFIG,DataR0);
      DataR1 |= 0x80;
      writeRegister(FDC2x1x_CONFIG,DataR1);
		break;
		default:
    break;
  }
}


uint16_t FDC2214::getManufacturerID(void)
{
  return readRegister(FDC2x1x_MANUFACTURER_ID);
}

uint16_t FDC2214::getDeviceID(void)
{
  return readRegister(FDC2x1x_DEVICE_ID);
}