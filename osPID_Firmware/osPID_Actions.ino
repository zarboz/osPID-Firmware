/* This file contains implementations of various user-triggered actions */

#include "ospConfig.h"
#include "ospProfile.h"
#include "ospAssert.h"

#undef BUGCHECK
#define BUGCHECK() ospBugCheck(PSTR("PROF"), __LINE__);


// a program invariant has been violated: suspend the controller and
// just flash a debugging message until the unit is power cycled
void ospBugCheck(const char *block, int line)
{
  // note that block is expected to be PROGMEM

  theLCD.noCursor();
    
  theLCD.clear();
  for (int i = 0; i < 4; i++)
    theLCD.print((char) pgm_read_byte_near(&block[i]));
  theLCD.print(F(" Err"));

  theLCD.setCursor(0, 1);
  theLCD.print(F("Line "));
  theLCD.print(line);

  // just lock up, flashing the error message
  while (true)
  {
    theLCD.display();
    delay(500);
    theLCD.noDisplay();
    delay(500);
  }
}

byte ATuneModeRemember;
ospDecimalValue<1> manualOutputRemember;

static void startAutoTune()
{
  ATuneModeRemember = myPID.GetMode();
  manualOutputRemember = manualOutput;
  
  // step value, avoiding output limits 
  ospDecimalValue<1> s = makeDecimal<1>(output);
  if ((ospDecimalValue<1>){ 500 } < s)
  {
    s = (ospDecimalValue<1>){ 1000 } - s; 
  }
  if (aTuneStep < s)
  {
    s = aTuneStep;
  }
  aTune.SetOutputStep(double(s));
  
  switch (aTuneMethod) // or any other PI method
  {
    case ZIEGLER_NICHOLS_PID: 
    case PESSEN_INTEGRAL_PID:
    case SOME_OVERSHOOT_PID:
    case NO_OVERSHOOT_PID:
    case TYREUS_LUYBEN_PID:
    case CIANCONE_MARLIN_PID:
      // and all other PID controllers
      break;
    default:  
      // PI controllers
      // ensure that derivative gain is zero
      myPID.SetTunings(aTune.GetKp(), aTune.GetKi(), 0.0);
  }
 
  myPID.SetMode(MANUAL);
  aTune.SetControlType(aTuneMethod); 
  aTune.SetNoiseBand(double(aTuneNoise));
  aTune.SetLookbackSec(aTuneLookBack);
  tuning = true;
}

static void stopAutoTune()
{
  aTune.Cancel();
  tuning = false;

  modeIndex = ATuneModeRemember;

  // restore the output to the last manual command; it will be overwritten by the PID
  // if the loop is active
  manualOutput = manualOutputRemember;
  setOutputToManualOutput();
  myPID.SetMode(modeIndex);
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

static bool startCurrentProfileStep()
{
  byte stepType;
  getProfileStepData(activeProfileIndex, currentProfileStep,
    &stepType, &profileState.stepDuration, &profileState.targetSetpoint);

  if (stepType == ospProfile::STEP_INVALID)
  {
    return false;
  }

#if !defined SILENCE_BUZZER
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
static void profileLoopIteration()
{
  double delta;
  double target = double(profileState.targetSetpoint);
#if !defined ATMEGA_32kB_FLASH
  ospAssert(!tuning);
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
#if !defined ATMEGA_32kB_FLASH
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

static void stopProfile()
{
#if !defined ATMEGA_32kB_FLASH
  ospAssert(runningProfile);
#endif
#if !defined SILENCE_BUZZER
  buzzOff;
#endif
  recordProfileCompletion();
  runningProfile = false;
}

