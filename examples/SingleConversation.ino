#include <Arduino.h>
#include <FDC2214.h>

FDC2214 FDC;

uint32_t Reading;

void setup(void)
{
    Serial.begin(115200);
    Wire.begin();

    if(!FDC.begin())
    {
        Serial.println("Init failed!");
    }
    else
    {
        Serial.println("found Device!");
    }

    FDC.setReferenceCount(FDC2x1x_CH0, 0xFFFF);
    FDC.setSettleCount(FDC2x1x_CH0, 0x64);
    FDC.setDividers(FDC2x1x_CH0,2,1);
    FDC.setINTB(FDC2x1x_INTB_DATAREADY);
    FDC.setDeglitchFilter(FDC2x1x_FILTER_10MHZ);
    FDC.setDriveCurrent(FDC2x1x_CH0, 0x7C00); // Measure with Oscilloscope and adjust [between 1.2V and 1.8V!!!]
    FDC.setActiveChannel(FDC2x1x_CH0);
    FDC.activateSensor(true);
    FDC.enableFullCurrentActivationMode(true);
    FDC.selectOscillator(FDC2x1x_EXT_OSC);
}

void loop(void)
{
  Reading = FDC.getReading(FDC2x1x_CH0);
  Serial.println(Reading);

  delay(100);
}