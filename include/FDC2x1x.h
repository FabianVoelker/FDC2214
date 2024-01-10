/**************************************************************************/
/*!
    @file     FDC2x1x.h
    This is a library for the FDC2x1x, a EMI-Resistant 28-Bit/12-Bit 
    high-resolution, multichannel Capacitance-To-Digital Converter 
    for Proximity and Level Sensing Applications from Texas Instruments.
    Written by Fabian Voelker for promesstec GmbH.
    MIT license, all text here must be included in any redistribution.
*/
/**************************************************************************/


#ifndef __FDC2x1x_H__
#define __FDC2x1x_H__


#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include <Adafruit_I2CDevice.h>
#include <SPI.h>
#include <Wire.h>



/*============================ Register-Map ================================================*/

#define FDC2x1x_ADDRESS_0                   0x2A    //< I2C Address for FDC2x1x [ADDR PIN = LOW] (DEFAULT)
#define FDC2x1x_ADDRESS_1                   0x2B    //< I2C Address for FDC2x1x [ADDR PIN = HIGH]

#define FDC211x_ID                          0x3054
#define FDC221x_ID                          0x3055

/* Registers */
#define FDC2x1x_DEVICE_ID                   0x7F    //< FDC2x1x-Register Device ID
#define FDC2x1x_MANUFACTURER_ID             0x7E    //< FDC2x1x-Register Manufacturer ID

#define FDC2x1x_MUX_CONFIG          		0x1B    //< FDC2x1x-Register Multiplexing Configuration
#define FDC2x1x_CONFIG              		0x1A    //< FDC2x1x-Register Conversion Configuration
#define FDC2x1x_STATUS              		0x18    //< FDC2x1x-Register Device Status Reporting
#define FDC2x1x_STATUS_CONFIG              	0x19    //< FDC2x1x-Register Device Status Reporting Configuration
#define FDC2x1x_RESET_DEVICE                0x1C    //< FDC2x1x-Register Reset Device

#define FDC2x1x_CLOCK_DIVIDERS_CH0  		0x14    //< FDC2x1x-Register Reference Divider Settings Channel 0
#define FDC2x1x_CLOCK_DIVIDERS_CH1  		0x15    //< FDC2x1x-Register Reference Divider Settings Channel 1
#define FDC2x1x_CLOCK_DIVIDERS_CH2  		0x16    //< FDC2x1x-Register Reference Divider Settings Channel 2
#define FDC2x1x_CLOCK_DIVIDERS_CH3  		0x17    //< FDC2x1x-Register Reference Divider Settings Channel 3

#define FDC2x1x_DRIVE_CURRENT_CH0           0x1E    //< FDC2x1x-Register Sensor Current Drive Configuration Channel 0    
#define FDC2x1x_DRIVE_CURRENT_CH1           0x1F    //< FDC2x1x-Register Sensor Current Drive Configuration Channel 1 
#define FDC2x1x_DRIVE_CURRENT_CH2           0x20    //< FDC2x1x-Register Sensor Current Drive Configuration Channel 2 
#define FDC2x1x_DRIVE_CURRENT_CH3           0x21    //< FDC2x1x-Register Sensor Current Drive Configuration Channel 3 

#define FDC2x1x_SETTLECOUNT_CH0     		0x10    //< FDC2x1x-Register Settling Reference Count Channel 0
#define FDC2x1x_SETTLECOUNT_CH1     		0x11    //< FDC2x1x-Register Settling Reference Count Channel 1
#define FDC2x1x_SETTLECOUNT_CH2     		0x12    //< FDC2x1x-Register Settling Reference Count Channel 2
#define FDC2x1x_SETTLECOUNT_CH3     		0x13    //< FDC2x1x-Register Settling Reference Count Channel 3

#define FDC2x1x_RCOUNT_CH0          		0x08    //< FDC2x1x-Register Reference Count Setting Channel 0
#define FDC2x1x_RCOUNT_CH1          		0x09    //< FDC2x1x-Register Reference Count Setting Channel 1
#define FDC2x1x_RCOUNT_CH2          		0x0A    //< FDC2x1x-Register Reference Count Setting Channel 2
#define FDC2x1x_RCOUNT_CH3          		0x0B    //< FDC2x1x-Register Reference Count Setting Channel 3

#define FDC2x1x_OFFSET_CH0		          	0x0C    //< FDC2x1x-Register Offset Value Channel 0
#define FDC2x1x_OFFSET_CH1          		0x0D    //< FDC2x1x-Register Offset Value Channel 1
#define FDC2x1x_OFFSET_CH2    			    0x0E    //< FDC2x1x-Register Offset Value Channel 2
#define FDC2x1x_OFFSET_CH3         			0x0F    //< FDC2x1x-Register Offset Value Channel 3

#define FDC2x1x_DATA_CH0_MSB	            0x00    //< FDC2x1x-Register Conversatioon Result Channel 0 [MSB and Status]
#define FDC2x1x_DATA_CH0_LSB    		    0x01    //< FDC2x1x-Register Conversatioon Result Channel 0 [LSB]
#define FDC2x1x_DATA_CH1_MSB	            0x02    //< FDC2x1x-Register Conversatioon Result Channel 1 [MSB and Status]
#define FDC2x1x_DATA_CH1_LSB    		    0x03    //< FDC2x1x-Register Conversatioon Result Channel 1 [LSB]
#define FDC2x1x_DATA_CH2_MSB	            0x04    //< FDC2x1x-Register Conversatioon Result Channel 2 [MSB and Status]
#define FDC2x1x_DATA_CH2_LSB    		    0x05    //< FDC2x1x-Register Conversatioon Result Channel 2 [LSB]
#define FDC2x1x_DATA_CH3_MSB	            0x06    //< FDC2x1x-Register Conversatioon Result Channel 3 [MSB and Status]
#define FDC2x1x_DATA_CH3_LSB    		    0x07    //< FDC2x1x-Register Conversatioon Result Channel 3 [LSB]


/*=============================================================================================*/


class FDC2214
{

public:

    FDC2214();
    bool begin(uint8_t i2caddr = FDC2x1x_ADDRESS_0, TwoWire &wirePort = Wire);
    bool isConnected();

private:
    TwoWire *_i2cPort;  
    uint8_t	_i2cAddr = FDC2x1x_ADDRESS_0;

    uint16_t readRegister(uint8_t Register);
    uint16_t writeRegister(uint8_t Register, uint16_t DataW);
};


#endif