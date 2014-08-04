/* This file contains implementations of various user-triggered actions */

#include "ospConfig.h"
#include "ospProfile.h"
#include "ospAssert.h"

#undef BUGCHECK
#define BUGCHECK() ospBugCheck(PSTR("PROF"), __LINE__);

// prototype definitions
extern Tuning tuningRule[PID::NO_OVERSHOOT_PID + 1];
extern void recordProfileStepCompletion(byte);
extern void stopProfile();
extern void recordProfileStart();

// a program invariant has been violated: suspend the controller and
// just flash a debugging message until the unit is power cycled
void ospBugCheck(const char *block, int line)
{
  // note that block is expected to be PROGMEM

  lcd.noCursor();
    
  lcd.clear();
  for (int i = 0; i < 4; i++)
    lcd.print((char) pgm_read_byte_near(&block[i]));
  lcd.print(F(" Err"));

  lcd.setCursor(0, 1);
  lcd.print(F("Line "));
  lcd.print(line);

  // just lock up, flashing the error message
  while (true)
  {
    lcd.display();
    delay(500);
    lcd.noDisplay();
    delay(500);
  }
}

struct ProfileState 
{
  unsigned long stepEndMillis;
  unsigned long stepDuration;
  ospDecimalValue<1> targetSetpoint;
  ospDecimalValue<1> initialSetpoint;
  byte stepType;
  bool temperatureRising;
};

ProfileState profileState;

static void getProfileStepData(byte profileIndex, byte i, byte *type, unsigned long *duration, ospDecimalValue<1> *endpoint);

bool startCurrentProfileStep()
{
  byte stepType;
  getProfileStepData(activeProfileIndex, currentProfileStep,
    &stepType, &profileState.stepDuration, &profileState.targetSetpoint);

  if (stepType == ospProfile::STEP_INVALID)
  {
    return false;
  }

#if !defined (SILENCE_BUZZER)
  if (stepType & ospProfile::STEP_FLAG_BUZZER)
  {
    buzzMillis(1000);
  }
  else
  {
    buzzOff;
  }
#endif

  profileState.stepType = stepType & ospProfile::STEP_TYPE_MASK;
  profileState.stepEndMillis = now + profileState.stepDuration;

  switch (profileState.stepType)
  {
  case ospProfile::STEP_RAMP_TO_SETPOINT:
    profileState.initialSetpoint = makeDecimal<1>(activeSetPoint);
    break;
  case ospProfile::STEP_SOAK_AT_VALUE:
    // targetSetpoint ignored
    // activeSetpoint stays at initial set point
    break;
  case ospProfile::STEP_JUMP_TO_SETPOINT:
    activeSetPoint = double(profileState.targetSetpoint);
    break;
  case ospProfile::STEP_WAIT_TO_CROSS:
    profileState.temperatureRising = (lastGoodInput < double(profileState.targetSetpoint));
    break;
  case ospProfile::STEP_HOLD_UNTIL_CANCEL: // not implemented
  default:
    return false;
  }

  return true;
}

// this function gets called every iteration of loop() while a profile is
// running
void profileLoopIteration()
{
  double delta;
  double target = double(profileState.targetSetpoint);
  
#if !defined (ATMEGA_32kB_FLASH)
  ospAssert(!myPID.isTuning());
  ospAssert(runningProfile);
#endif  

  long int stepTimeLeft = profileState.stepEndMillis - now;
  switch (profileState.stepType)
  {
  case ospProfile::STEP_RAMP_TO_SETPOINT:
    if (stepTimeLeft <= 0)
    {
      activeSetPoint = target;
      break;
    }
    delta = target - double(profileState.initialSetpoint);
    activeSetPoint = target - 
      (delta * stepTimeLeft / profileState.stepDuration);
    return;
  case ospProfile::STEP_SOAK_AT_VALUE:
  case ospProfile::STEP_JUMP_TO_SETPOINT:
    if (0 < stepTimeLeft)
    {
      return;
    }
    break;
  case ospProfile::STEP_WAIT_TO_CROSS:
    if ((lastGoodInput < target) && profileState.temperatureRising)
    {
      return; // not there yet
    }
    if ((target < lastGoodInput) && !profileState.temperatureRising)
    {
      return;
    }
    break;
  }
  
  // this step is done: load the next one if it exists
  recordProfileStepCompletion(currentProfileStep);
  if (currentProfileStep < ospProfile::NR_STEPS) 
  {
    currentProfileStep++;
    if (startCurrentProfileStep()) // returns false if there are no more steps
    {
      return;
    }
  }

  // the profile is finished
  stopProfile();
}

static void startProfile()
{
  
#if !defined (ATMEGA_32kB_FLASH)
  ospAssert(!runningProfile);
#endif  

  currentProfileStep = 0;
  recordProfileStart();
  runningProfile = true;

  if (!startCurrentProfileStep())
  {
    stopProfile();
  }
}

void stopProfile()
{
  
#if !defined (ATMEGA_32kB_FLASH)
  ospAssert(runningProfile);
#endif

#if !defined (SILENCE_BUZZER)
  buzzOff;
#endif

  recordProfileCompletion();
  runningProfile = false;
}

