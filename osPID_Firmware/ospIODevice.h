// This header defines the base and utility classes for input and output devices  
#if !defined OSPIODEVICE_H
#define OSPIODEVICE_H

// classes defined with dummy methods to avoid overhead of pure  functions

class ospSettingsHelper;

// a base class for both input and output IO devices
class ospBaseIODevice 
{
public:
  ospBaseIODevice() { }

  // device types
  enum 
  {
    INPUT_THERMISTOR = 0,
    INPUT_ONEWIRE,
    INPUT_THERMOCOUPLE,
    OUTPUT_SSR,
    INPUT_SIMULATOR
  };

  // setup the IO device 
  void initialize() {}; 

  // return an identifying name for this IO device, as a PSTR
  const __FlashStringHelper *IODeviceIdentifier() { return NULL; }; 

  // how many settings does this IO device have
  byte floatSettingsCount() { return 0xFF; }; 
  // byte integerSettingsCount() { return 0xFF; }; 

  // read settings from the IO device
  double readFloatSetting(byte index) { return NAN; };
  // int readIntegerSetting(byte index) { return -1; };

  // write settings to the IO device
  bool writeFloatSetting(byte index, double val) { return false; };
  // bool writeIntegerSetting(byte index, int val) { return false; };
  
  // return a text description of the N'th setting, as a PSTR
  // also returns the number of decimal places
  const __FlashStringHelper *describeFloatSetting(byte index) { return NULL; };
  // const __FlashStringHelper *describeIntegerSetting(byte index) { return NULL; };

  // save and restore settings to/from EEPROM using the settings helper
  void saveSettings(ospSettingsHelper& settings) {};
  void restoreSettings(ospSettingsHelper& settings) {};
};

class ospBaseInputDevice  : public ospBaseIODevice 
{
public:
  ospBaseInputDevice()  :
    ospBaseIODevice() 
  {
  }

  double readInput() { return NAN; };
  void setCalibration(ospDecimalValue<1> newCalibration) {};
  ospDecimalValue<1> getCalibration() { return (ospDecimalValue<1>){-19999}; };
};

class ospBaseOutputDevice : public ospBaseIODevice 
{
public:
  ospBaseOutputDevice() :
   ospBaseIODevice()
  {
  }
 
  void setOutputPercent(double percentage) {};
  void setOutputWindowSeconds(ospDecimalValue<1> newOutputWindowSeconds) {};
  ospDecimalValue<1> getOutputWindowSeconds() { return (ospDecimalValue<1>){-19999}; };
  
};

#endif
