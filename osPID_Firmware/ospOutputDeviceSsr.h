#if !defined (OSPOUTPUTDEVICESSR_H)
#define OSPOUTPUTDEVICESSR_H

#include "ospIODevice.h"
#include "ospSettingsHelper.h"

class ospOutputDeviceSsr : 
  public ospBaseOutputDevice 
{
private:
  ospDecimalValue<1> outputWindowSeconds;
  unsigned long outputWindowMilliseconds;

public:
  ospOutputDeviceSsr() : 
    ospBaseOutputDevice(), 
    outputWindowSeconds((ospDecimalValue<1>){50}),
    // default of output cycle length 5s 
    // this is OK for SSR depending on the load
    // needs to be longer for electromechanical relay
    outputWindowMilliseconds(5000), 
    ioType(OUTPUT_SSR)
  { 
  }

  // input and output types
  byte ioType;
  
  void initialize() 
  {
    pinMode(SsrPin, OUTPUT);
  }
  
  ospDecimalValue<1> getOutputWindowSeconds()
  {
    return outputWindowSeconds;
  }  
  
  void setOutputWindowSeconds(ospDecimalValue<1> newOutputWindowSeconds)
  {
    if (newOutputWindowSeconds.rawValue() >= 10) // minimum output cycle length 1 second
    {
      outputWindowSeconds = newOutputWindowSeconds;
      outputWindowMilliseconds = (unsigned long) outputWindowSeconds.rawValue() * 100;
    }
  }  

  const __FlashStringHelper *IODeviceIdentifier() 
  { 
    return F("SSR Output"); 
  }

  // how many settings does this device have
  byte floatSettingsCount()
  { 
    return 1; 
  }
/*
  byte integerSettingsCount() 
  { 
    return 0; 
  }
*/

  // read settings from the device
  double readFloatSetting(byte index) 
  {
    if (index == 0)
      return double(outputWindowSeconds);
    return NAN;
  }
/*
  int readIntegerSetting(byte index) 
  {
    return -1;
  }
*/

  // write settings to the device
  bool writeFloatSetting(byte index, double val) 
  {
    if (index == 0) 
    {
      this->setOutputWindowSeconds(makeDecimal<1>(val));
      return true;
    }
    return false;
  }
/*
  bool writeIntegerSetting(byte index, int val)
  {
    return false;
  } 
*/

  // describe the available settings
  const __FlashStringHelper *describeFloatSetting(byte index) 
  {
    if (index == 0) 
    {
      return F("Output PWM cycle length in seconds");
    }
    return NULL;
  }
/*
  const __FlashStringHelper *describeIntegerSetting(byte index) 
  {
    return NULL;
  }
*/

  // save and restore settings to/from EEPROM using the settings helper
  void saveSettings(ospSettingsHelper& settings) 
  {
    settings.save(outputWindowSeconds);
  }

  void restoreSettings(ospSettingsHelper& settings) 
  {
    settings.restore(outputWindowSeconds);
    this->setOutputWindowSeconds(outputWindowSeconds);
  }

  void setOutputPercent(double percent)
  {
    unsigned long wind = millis() % outputWindowMilliseconds;
    unsigned long oVal = (unsigned long)(percent * 0.01 * (double)outputWindowMilliseconds);
    digitalWrite(SsrPin, (oVal > wind) ? HIGH : LOW);
  }
};

#endif
