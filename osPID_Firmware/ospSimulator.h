#if !defined (OSPSIMULATOR_H)
#define OSPSIMULATOR_H

#include "ospIODevice.h"
#include "ospSettingsHelper.h"

// a "device" which simulates a simple plant including a proportional heating
// term, a thermal loss rate, and some measurement noise
class ospSimulator : public ospBaseInputDevice, public ospBaseOutputDevice 
{
private:
  // NB size of theta[] reduced from 30 to 10 to save memory
  double kpmodel, taup, theta[10];
  double processValue;
  int    modelDelay;

  static const double outputStart = 50.0f;  
  static const double processValueStart = 100.0f;
  
  bool initializationStatus;

public:
  ospSimulator()
    : ospBaseInputDevice(), ospBaseOutputDevice()
  {
    ioType = INPUT_SIMULATOR;
  }

  // input and output types
  byte ioType;

  // setup the device
  void initialize() 
  {
    kpmodel = 2.0;
    taup = 100.0;
    modelDelay = 10;
    processValue = processValueStart;
    for (int i = 0; i < modelDelay; i++)
    {
      theta[i] = outputStart;
    }
    this->setInitializationStatus(true);
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
  
  unsigned long requestInput() 
  {
    return 250; // 250 ms read delay
  }

  // pretend to read an input from the input device
  double readInput() 
  {
    updateModel();
    return processValue;
  }

  // return the device identifier
  const char *IODeviceIdentifier() { return "SIML"; }

  // how many settings does this device have
  byte floatSettingsCount() { return 3; }

  // read settings from the device
  double readFloatSetting(byte index) 
  {
    switch (index) 
    {
    case 0:
      return kpmodel;
    case 1:
      return taup;
    case 2:
      return modelDelay;
    default:
      return -1.0f;
    }
  }

  // write settings to the device
  bool writeFloatSetting(byte index, double val) 
  {
    switch (index) 
    {
    case 0:
      kpmodel = val;
      return true;
    case 1:
      taup = val;
      return true;
    case 2:
      modelDelay = (int)val;
      return true;
    default:
      return false;
    }
  }

  // describe the device settings
  const __FlashStringHelper *describeFloatSetting(byte index) 
  {
    switch (index)
    {
    case 0:
      return F("Simulated process gain");
    case 1:
      return F("Simulated lag value");
    case 2:
      return F("Simulated model delay");
    default:
      return NULL;
    }
  }

  // save and restore settings to/from EEPROM using the settings helper
  void saveSettings(ospSettingsHelper& settings) 
  {
    settings.save(kpmodel);
    settings.save(taup);
    settings.save(modelDelay);
  }

  void restoreSettings(ospSettingsHelper& settings) 
  {
    settings.restore(kpmodel);
    settings.restore(taup);
    settings.restore(modelDelay);
  }

  // pretend to write a control signal to the output device
  void setOutputPercent(double percent) 
  {
    theta[modelDelay - 1] = percent;
  }

private:
  void updateModel()
  {
    // cycle the dead time
    for (byte i = 0; i < (modelDelay - 1); i++)
    {
      theta[i] = theta[i + 1];
    }
    
    // compute the process value
    processValue = (kpmodel / taup) * (theta[0] - outputStart) + 
            (processValue - processValueStart) * (1.0 - 1.0 / taup) + 
            processValueStart + 
            ((double) random(-10, 10)) / 100.0;
  }
};

#endif
