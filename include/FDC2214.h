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
#include <Wire.h>


#define I2C_SPEED_STANDARD        100000
#define I2C_SPEED_FAST            400000


/*============================ Register-Map ================================================*/

#define FDC2x1x_ADDRESS_0                   0x2A    //< I2C Address for FDC2x1x [ADDR PIN = LOW] (DEFAULT)
#define FDC2x1x_ADDRESS_1                   0x2B    //< I2C Address for FDC2x1x [ADDR PIN = HIGH]

#define FDC211x_ID                          0x3054  //< Device ID FDC211x
#define FDC221x_ID                          0x3055  //< Device ID FDC221x

/* Registers */
#define FDC2x1x_DEVICE_ID                   0x7F    //< FDC2x1x-Register Device ID
#define FDC2x1x_MANUFACTURER_ID             0x7E    //< FDC2x1x-Register Manufacturer ID

#define FDC2x1x_MUX_CONFIG          		0x1B    //< FDC2x1x-Register Multiplexing Configuration
#define FDC2x1x_CONFIG              		0x1A    //< FDC2x1x-Register Conversion Configuration
#define FDC2x1x_STATUS              		0x18    //< FDC2x1x-Register Device Status Reporting
#define FDC2x1x_ERROR_CONFIG              	0x19    //< FDC2x1x-Register Device Status Reporting Configuration
#define FDC2x1x_RESET_DEVICE                0x1C    //< FDC2x1x-Register Reset Device


enum Channel
{
    FDC2x1x_CH0,
    FDC2x1x_CH1,
    FDC2x1x_CH2,
    FDC2x1x_CH3,
};

enum IntteruptFunctions 
{
    FDC2x1x_INTB_OFF,
    FDC2x1x_INTB_WD,
    FDC2x1x_INTB_DATAREADY,
};

enum DeglitchFilter 
{
    FDC2x1x_FILTER_1MHZ,
    FDC2x1x_FILTER_3MHZ,
    FDC2x1x_FILTER_10MHZ,
    FDC2x1x_FILTER_33MHZ,
};

enum AutoscanSequence 
{
    FDC2x1x_SEQ_CH0_CH1,
    FDC2x1x_SEQ_CH0_CH1_CH2,
    FDC2x1x_SEQ_CH0_CH1_CH2_CH3,
};

enum Oscillator
{
    FDC2x1x_INT_OSC,
    FDC2x1x_EXT_OSC,
};


#define FDC2x1x_CLOCK_DIVIDERS_CH0  		0x14    //< FDC2x1x-Register Reference Divider Settings Channel 0
#define FDC2x1x_CLOCK_DIVIDERS_CH1  		0x15    //< FDC2x1x-Register Reference Divider Settings Channel 1
#define FDC2x1x_CLOCK_DIVIDERS_CH2  		0x16    //< FDC2x1x-Register Reference Divider Settings Channel 2
#define FDC2x1x_CLOCK_DIVIDERS_CH3  		0x17    //< FDC2x1x-Register Reference Divider Settings Channel 3

#define CHx_FIN_SEL_DIFFERENTIAL            0b01    
#define CHx_FIN_SEL_SINGLEENDED             0b10

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
    bool begin(uint8_t i2caddr = FDC2x1x_ADDRESS_0, TwoWire &wirePort = Wire, uint32_t i2cSpeed = I2C_SPEED_STANDARD);
    bool isConnected(void);

    uint16_t getManufacturerID(void);
    uint16_t getDeviceID(void);

    uint16_t getStatus(void);



    uint32_t getReading(enum Channel channel);



    void configureSingleShotChannel(enum Channel channel, uint16_t sensorFreqSel, uint16_t CHxRefDivider);

    void setDividers(enum Channel channel, uint16_t sensorFreqSel, uint16_t CHxRefDivider);
    void setDriveCurrent(enum Channel channel, uint16_t CHxIDrive);
    void setSettleCount(enum Channel channel, uint16_t CHxSettleCount);
    void setReferenceCount(enum Channel channel, uint16_t CHxRefCount);
    void setOffset(enum Channel channel, uint16_t CHxOffset);

    void setDeglitchFilter(enum DeglitchFilter filter);
    void enableAutoScan(bool en);
    void setRRSequence(enum AutoscanSequence sequence);

    void setActiveChannel(enum Channel channel);
    void activateSensor(bool en);
    void enableFullCurrentActivationMode(bool en);
    void selectOscillator(enum Oscillator oscillator);
    void enableHighCurrentDrive(bool en);

    void enableWDError(bool en);
    void enableAmpHighError(bool en);
    void enableAmpLowError(bool en);
    void setINTB(enum IntteruptFunctions functions);

    


    

private:
    TwoWire *_i2cPort;  
    uint8_t	_i2cAddr = FDC2x1x_ADDRESS_0;

    uint16_t _deviceid;

    uint16_t readRegister(uint8_t Register);
    void writeRegister(uint8_t Register, uint16_t DataW);
};


#endif