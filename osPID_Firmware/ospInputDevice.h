#if !defined OSPINPUTDEVICE_H
#define OSPINPUTDEVICE_H

#include "ospIODevice.h"
#include "ospSettingsHelper.h"
#include "OneWire_local.h"
#include "DallasTemperature_local.h"
#include "MAX31855_local.h"


// class using crude switches 
// instead of pretty virtual methods
// that there is not space to accommodate

enum { INPUT_THERMISTOR = 0, INPUT_ONEWIRE, INPUT_THERMOCOUPLE };
byte inputType = INPUT_THERMISTOR;


class ospInputDevice : 
  public ospBaseInputDevice 
{
private:
  
  enum // settings
  { 
    CALIBRATION_THERMISTOR = 0,
    CALIBRATION_ONEWIRE,
    CALIBRATION_THERMOCOUPLE,  
    NOMINAL,
    BCOEFFICIENT,
    TEMPERATURE,
    REFERENCE
  };    

  bool initializationStatus;
  double inputSetting[7];

  OneWire oneWire;
  DallasTemperature oneWireDevice;
  DeviceAddress oneWireDeviceAddress;

  MAX31855 thermocouple;

  // convert the thermistor voltage to a temperature
  double thermistorVoltageToTemperature(int voltage)
  {
    double R = inputSetting[REFERENCE] / (1024.0/(double)voltage - 1);
    double steinhart;
    steinhart = R / inputSetting[NOMINAL];                           // (R/Ro)
    steinhart = log(steinhart);                                      // ln(R/Ro)
    steinhart /= inputSetting[BCOEFFICIENT];                         // 1/B * ln(R/Ro)
    steinhart += 1.0 / (inputSetting[TEMPERATURE] + 273.15);         // + (1/To)
    steinhart = 1.0 / steinhart;                                     // Invert
    steinhart -= 273.15;                                             // convert to Celsius
  }
  

public:
  ospInputDevice() :
    ospBaseInputDevice(),
    initializationStatus(false),
    oneWire(oneWireBus),
    oneWireDevice(&oneWire),
    thermocouple(thermocoupleCLK_Pin, thermocoupleCS_Pin, thermocoupleSO_Pin)
  { 
    inputSetting[NOMINAL]       = THERMISTOR_NOMINAL_RESISTANCE;
    inputSetting[BCOEFFICIENT]  = THERMISTOR_B_COEFFICIENT;
    inputSetting[TEMPERATURE]   = THERMISTOR_TEMPERATURE_NOMINAL;
    inputSetting[REFERENCE]     = THERMISTOR_REFERENCE_RESISTANCE;
  }
  
  void initialize() 
  {
    if (inputType == INPUT_ONEWIRE)
    {
      oneWireDevice.begin();
      if (!oneWireDevice.getAddress(oneWireDeviceAddress, 0)) 
      {
        initializationStatus = false;
        return;
      }
      else 
      {
        oneWireDevice.setResolution(oneWireDeviceAddress, 12);
        oneWireDevice.setWaitForConversion(false);
      }
    }
    else if (inputType == INPUT_THERMISTOR)
    {
      pinMode(thermistorPin, INPUT);
    }
    initializationStatus = true;
  }
  
  const __FlashStringHelper *IODeviceIdentifier()
  {
    switch (inputType)
    {
    case INPUT_THERMISTOR:
      return F("NTC thermistor");
    case INPUT_ONEWIRE:
      return F("DS18B20+");
    case INPUT_THERMOCOUPLE: 
      return F("K-type thermocouple");
    default:
      return NULL;
    }
  }
  
  // how many settings does this device have
  byte floatSettingsCount() 
  {
    return 7;
  }  

  // read settings from the device
  double readFloatSetting(byte index) 
  {
    if (index > 6)
      return NULL;
    return inputSetting[index];
  }
    
  // write settings to the device
  bool writeFloatSetting(byte index, double val) 
  {
    if (index > 6)
      return false;
    inputSetting[index] = val;
    return true;
  }
  
  // describe the device settings
  const __FlashStringHelper *describeFloatSetting(byte index) 
  {
    switch (index)
    {
    case 0:
      return F("Thermistor calibration value");
    case 1:
      return F("DS18B20+ calibration value");
    case 2:
      return F("Thermocouple calibration value");
    case 3:
      return F("Thermistor nominal resistance (Kohms)");
    case 4:
      return F("Reference resistor value (Kohms)");
    case 5:
      return F("Thermistor B coefficient");
    case 6:
      return F("Thermistor reference temperature (Celsius)");
    default:
      return NULL;
    }
  }

  // save and restore settings to/from EEPROM using the settings helper
  void saveSettings(ospSettingsHelper& settings) 
  {
    for (byte i = 0; i < 7; i++)
    {
      settings.save(inputSetting[i]);
    }
  }

  void restoreSettings(ospSettingsHelper& settings) 
  {
    for (byte i = 0; i < 7; i++)
    {
      settings.restore(inputSetting[i]);
    }
  }  

/*
  byte integerSettingsCount() 
  {
    return 0; 
  }

  int readIntegerSetting(byte index) 
  {
    return -1;
  }

  bool writeIntegerSetting(byte index, int val) 
  {
    return false;
  }

  const __FlashStringHelper *describeIntegerSetting(byte index) 
  {
    switch (index) 
    {
    default:
      return NULL;
    }
  }
*/

  // request input
  // returns conversion time in milliseconds
  unsigned long requestInput() 
  {
    if (inputType == INPUT_ONEWIRE)
    {
      oneWireDevice.requestTemperatures();
      return 750;
    }
    return INPUT_MINIMUM_SAMPLE_TIME;
  }

  double readInput()
  {
    double temperature;
    switch (inputType)
    {
    case INPUT_THERMISTOR:
      int voltage;
      voltage = analogRead(thermistorPin);
      temperature = thermistorVoltageToTemperature(voltage);
      break;
    case INPUT_ONEWIRE:
      temperature = oneWireDevice.getTempCByIndex(0);
      break;
    case INPUT_THERMOCOUPLE: 
      temperature = thermocouple.readThermocouple(CELSIUS);
      if ((temperature == FAULT_OPEN) || (temperature = FAULT_SHORT_GND) || (temperature == FAULT_SHORT_VCC))
        return NAN;
      break;
    default:
      return NAN;
    }
#if !defined UNITS_FAHRENHEIT
    return temperature + inputSetting[inputType];
#else
    return (temperature * 1.8 + 32.0) + inputSetting[inputType];
#endif
  }
  
  // get initialization status
  bool getInitializationStatus()
  {
    return initializationStatus;
  }

  // set initialization status
  void setInitializationStatus(bool newInitializationStatus)
  {
    initializationStatus = newInitializationStatus;
  }

  // get calibration
  ospDecimalValue<1> getCalibration()
  {
    return makeDecimal<1>(inputSetting[inputType]);
  }

  // set calibration
  void setCalibration(ospDecimalValue<1> newCalibration)
  {
    inputSetting[inputType] = double(newCalibration);
  }  
};


#endif
