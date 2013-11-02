#if !defined OSPPROFILE_H
#define OSPPROFILE_H

#include <Arduino.h>
#include <avr/pgmspace.h>

#include "ospDecimalValue.h"

// an ospProfile encapsulates the information for a setpoint profile
class ospProfile 
{
  
public:

  // constants
  
  // profile step codes
  // note that only (stepTypes[i] & 0x7F) is significant: we reserve one bit
  // which can be freely toggled to make sure that we never get a CRC-16 of
  // 0x0000 when the profile is saved
  static const byte STEP_RAMP_TO_SETPOINT  = 0;
  static const byte STEP_SOAK_AT_VALUE     = 1;
  static const byte STEP_JUMP_TO_SETPOINT  = 2;
  static const byte STEP_WAIT_TO_CROSS     = 3;
  static const byte STEP_HOLD_UNTIL_CANCEL = 4;
  static const byte LAST_VALID_STEP        = STEP_HOLD_UNTIL_CANCEL;
  static const byte STEP_FLAG_BUZZER       = 0x40;
  static const byte STEP_EEPROM_SWIZZLE    = 0x80;
  static const byte STEP_INVALID           = 0x7F;
  static const byte STEP_CONTENT_MASK      = 0x7F;
  static const byte STEP_TYPE_MASK         = 0x3F;

  static const byte NR_STEPS    = 16;
  static const byte NAME_LENGTH = 15;
  
  // variables

  char name[NAME_LENGTH+1];
  byte nextStep;
  byte stepTypes[NR_STEPS];
  unsigned long stepDurations[NR_STEPS];
  ospDecimalValue<1> stepEndpoints[NR_STEPS];
  
  // methods

  ospProfile() 
  {
    clear();
  }

  bool addStep(byte type, unsigned long duration, ospDecimalValue<1> endpoint)
  {
    if (nextStep == NR_STEPS)
    {
      return false;
    }
    if (type & STEP_EEPROM_SWIZZLE)
    {
      return false;
    }
    if ((type & STEP_TYPE_MASK) > LAST_VALID_STEP)
    {
      return false;
    }

    stepTypes[nextStep] = type;
    stepDurations[nextStep] = duration;
    stepEndpoints[nextStep] = endpoint;
    nextStep++;
    return true;
  }

  void clear() 
  {
    nextStep = 0;
    strcpy_P(name, PSTR("No Profile     "));
    memset(stepTypes, STEP_INVALID, sizeof(stepTypes));
    memset(stepDurations, -1, sizeof(stepDurations));
    memset(stepEndpoints, -1, sizeof(stepEndpoints));
  }
};

#endif

