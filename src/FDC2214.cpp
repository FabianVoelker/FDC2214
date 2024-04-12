
/*!
    @file     FDC2214.cpp
    This is a library for the FDC2x1x, a EMI-Resistant 28-Bit/12-Bit 
    high-resolution, multichannel Capacitance-To-Digital Converter 
    for Proximity and Level Sensing Applications from Texas Instruments.
    Written by Fabian Voelker for promesstec GmbH.
    MIT license, all text here must be included in any redistribution.
*/


#include "FDC2214.h"

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



FDC2214::FDC2214()
{
  
}


/**************************************************************************/
/*!
    @brief Initalisize FDC2x1x Device
    @param i2caddr I2C Adress [0x2A DEFAULT]
    @param wirePort I2C Port of Device
*/
/**************************************************************************/
bool FDC2214::begin(uint8_t i2caddr, TwoWire &wirePort, uint32_t i2cSpeed)
{
  _i2cPort = &wirePort;
  _i2cAddr = i2caddr;

  _i2cPort->begin();

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
    @brief gets the conversation result
    @param channel Channel Selection ( CH0 - CH3 )
*/
/**************************************************************************/
uint32_t FDC2214::getReading(enum Channel channel)
{
  uint32_t Reading;

  uint8_t timeout = 200;

  uint8_t addressMSB;
	uint8_t addressLSB;
	uint8_t bitUnreadConv;

  uint16_t status = readRegister(FDC2x1x_STATUS);

  switch (channel)
  {
		case (FDC2x1x_CH0):
			addressMSB = FDC2x1x_DATA_CH0_MSB;
			addressLSB = FDC2x1x_DATA_CH0_LSB;
			bitUnreadConv = 0x08;
		break;
		case (FDC2x1x_CH1):
			addressMSB = FDC2x1x_DATA_CH1_MSB;
			addressLSB = FDC2x1x_DATA_CH1_LSB;
			bitUnreadConv = 0x04;
		break;
		case (FDC2x1x_CH2):
			addressMSB = FDC2x1x_DATA_CH2_MSB;
			addressLSB = FDC2x1x_DATA_CH2_LSB;
			bitUnreadConv = 0x02;
		break;
		case (FDC2x1x_CH3):
			addressMSB = FDC2x1x_DATA_CH3_MSB;
			addressLSB = FDC2x1x_DATA_CH3_LSB;
			bitUnreadConv = 0x01;
		break;
		default: return 0;
	}

  while (timeout && !(status & bitUnreadConv)) 
  {
        status = readRegister(FDC2x1x_STATUS);
        timeout--;
  }

  if(timeout)
  {
    Reading = (uint32_t)(readRegister(addressMSB) & 0x0FFF) << 16;
    Reading |= readRegister(addressLSB);
    return Reading;
  }
  else
  {
    return 0;
  }
}




/**************************************************************************/
/*!
    @brief Sets the Dividers for specific Channel
    @param channel Channel Selection ( CH0 - CH3 )
    @param sensorFreqSel Differential Config or SingleEnded Config
    @param CHxRefDivider Sets fREFx = fCLK/CHx_FREF_DIVIDER
*/
/**************************************************************************/
void FDC2214::setDividers(enum Channel channel, uint16_t sensorFreqSel, uint16_t CHxRefDivider)
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
    case (FDC2x1x_CH0):
      writeRegister(FDC2x1x_CLOCK_DIVIDERS_CH0,DataW);
		break;
		case (FDC2x1x_CH1):
			writeRegister(FDC2x1x_CLOCK_DIVIDERS_CH1,DataW);
		break;
		case (FDC2x1x_CH2):
			writeRegister(FDC2x1x_CLOCK_DIVIDERS_CH2,DataW);
		break;
		case (FDC2x1x_CH3):
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
    @param CHxIDrive sets Drive Current for Sensor [0x0000 = 16uA, 0xF800 = 1.5mA]
    @warning oscillation amplitude must between 1.2V and 1.8V!
             Measure the oscillation amplitude on an oscilloscope 
             and adjust the IDRIVE value
*/
/**************************************************************************/
void FDC2214::setDriveCurrent(enum Channel channel, uint16_t CHxIDrive)
{
  uint16_t DataW = CHxIDrive;

  switch (channel)
  {
    case (FDC2x1x_CH0):
      writeRegister(FDC2x1x_DRIVE_CURRENT_CH0,DataW);
		break;
		case (FDC2x1x_CH1):
			writeRegister(FDC2x1x_DRIVE_CURRENT_CH1,DataW);
		break;
		case (FDC2x1x_CH2):
			writeRegister(FDC2x1x_DRIVE_CURRENT_CH2,DataW);
		break;
		case (FDC2x1x_CH3):
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
void FDC2214::setSettleCount(enum Channel channel, uint16_t CHxSettleCount)
{
  uint16_t DataW;

  if(CHxSettleCount<=0xFFFF)
  {
    DataW |= CHxSettleCount;
  }

  switch (channel)
  {
    case (FDC2x1x_CH0):
      writeRegister(FDC2x1x_SETTLECOUNT_CH0,DataW);
		break;
		case (FDC2x1x_CH1):
			writeRegister(FDC2x1x_SETTLECOUNT_CH1,DataW);
		break;
		case (FDC2x1x_CH2):
			writeRegister(FDC2x1x_SETTLECOUNT_CH2,DataW);
		break;
		case (FDC2x1x_CH3):
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
void FDC2214::setReferenceCount(enum Channel channel, uint16_t CHxRefCount)
{
  uint16_t DataW;

  if(CHxRefCount>=0x0100 && CHxRefCount<=0xFFFF)
  {
    DataW |= CHxRefCount;
  }

  switch (channel)
  {
    case (FDC2x1x_CH0):
      writeRegister(FDC2x1x_RCOUNT_CH0,DataW);
		break;
		case (FDC2x1x_CH1):
			writeRegister(FDC2x1x_RCOUNT_CH1,DataW);
		break;
		case (FDC2x1x_CH2):
			writeRegister(FDC2x1x_RCOUNT_CH2,DataW);
		break;
		case (FDC2x1x_CH3):
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
void FDC2214::setOffset(enum Channel channel, uint16_t CHxOffset)
{
  switch (channel)
  {
    case (FDC2x1x_CH0):
      writeRegister(FDC2x1x_OFFSET_CH0,CHxOffset);
		break;
		case (FDC2x1x_CH1):
			writeRegister(FDC2x1x_OFFSET_CH1,CHxOffset);
		break;
		case (FDC2x1x_CH2):
			writeRegister(FDC2x1x_OFFSET_CH2,CHxOffset);
		break;
		case (FDC2x1x_CH3):
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
    @brief selects channel for continuous conversation
    @param channel Channel Selection ( CH0 - CH3 )
    @warning  Selects channel for continuous conversions when
              MUX_CONFIG.SEQUENTIAL is 0!
*/
/**************************************************************************/
void FDC2214::setActiveChannel(enum Channel channel)
{
  uint16_t DataR = readRegister(FDC2x1x_CONFIG);
  DataR &= ~(0b11 << 14);

  switch (channel)
  {
    case (FDC2x1x_CH0):
      DataR |= (0b00 << 14);
      writeRegister(FDC2x1x_CONFIG,DataR);
		break;
		case (FDC2x1x_CH1):
			DataR |= (0b01 << 14);
      writeRegister(FDC2x1x_CONFIG,DataR);
		break;
		case (FDC2x1x_CH2):
			DataR |= (0b10 << 14);
      writeRegister(FDC2x1x_CONFIG,DataR);
		break;
		case (FDC2x1x_CH3):
			DataR |= (0b11 << 14);
      writeRegister(FDC2x1x_CONFIG,DataR);
		break;
		default:
    break;
  }
}


/**************************************************************************/
/*!
    @brief Enter or exit low power Sleep Mode.
    @param en Enable Device or Enter Sleep mode [true/false]
*/
/**************************************************************************/
void FDC2214::activateSensor(bool en)
{
  uint16_t DataR = readRegister(FDC2x1x_CONFIG);
  DataR &= ~(1 << 13);

  if(en)
  {
    DataR &= ~(1 << 13);
  }
  else
  {
    DataR |= (1 << 13);
  }
  writeRegister(FDC2x1x_CONFIG,DataR);
}


/**************************************************************************/
/*!
    @brief Sensor Activation Mode Selection: Set the mode for sensor initialization
    @param en En/Disable Full Current Activation [true/false]
    @warning b0: Full Current Activation Mode – the FDC will drive maximum
              sensor current for a shorter sensor activation time.
    @warning b1: Low Power Activation Mode – the FDC uses the value
              programmed in DRIVE_CURRENT_CHx during sensor
              activation to minimize power consumption.
*/
/**************************************************************************/
void FDC2214::enableFullCurrentActivationMode(bool en)
{
  uint16_t DataR = readRegister(FDC2x1x_CONFIG);
  DataR &= ~(1 << 11);

  if(en)
  {
    DataR &= ~(1 << 11);
  }
  else
  {
    DataR |= (1 << 11);
  }
  writeRegister(FDC2x1x_CONFIG,DataR);
}


/**************************************************************************/
/*!
    @brief Select Reference Frequency Source
    @param oscillator select INT or EXT Oscillator as reference
    @warning b0: Use Internal oscillator as reference frequency
    @warning b1: Reference frequency is provided from CLKIN pin.
*/
/**************************************************************************/
void FDC2214::selectOscillator(enum Oscillator oscillator)
{
  uint16_t DataR = readRegister(FDC2x1x_CONFIG);
  DataR &= ~(1 << 9);

  switch (oscillator)
  {
    case (FDC2x1x_INT_OSC):
      DataR &= ~(1 << 9);
      writeRegister(FDC2x1x_CONFIG,DataR);
		break;
		case (FDC2x1x_EXT_OSC):
			DataR |= (1 << 9);
      writeRegister(FDC2x1x_CONFIG,DataR);
		break;
		default:
    break;
  }
}


/**************************************************************************/
/*!
    @brief Select Reference Frequency Source
    @param en En/Disable High Current Drive [true/false]
    @warning b0: The FDC will drive all channels with normal sensor current
                 (1.5mA max).
    @warning b1: he FDC will drive channel 0 with current >1.5mA.
                This mode is not supported if AUTOSCAN_EN = b1 (multichannel mode)
*/
/**************************************************************************/
void FDC2214::enableHighCurrentDrive(bool en)
{
  uint16_t DataR = readRegister(FDC2x1x_CONFIG);
  DataR &= ~(1 << 6);

  if(en)
  {
    DataR &= ~(1 << 6);
  }
  else
  {
    DataR |= (1 << 6);
  }
  writeRegister(FDC2x1x_CONFIG,DataR);
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