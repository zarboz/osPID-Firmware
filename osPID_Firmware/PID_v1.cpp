/**********************************************************************************************
 * Arduino PID Library - Version 1.0.1
 * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 *
 * This Library is licensed under a GPLv3 License
 **********************************************************************************************/

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "PID_v1_local.h" //renamed to avoid conflict if PID library is installed on IDE

/*Constructor (...)*********************************************************
* The parameters specified here are those for for which we can't set up
* reliable defaults, so we need to have the user set them.
***************************************************************************/
PID::PID(double* Input, double* Output, double* Setpoint,
        ospDecimalValue<3> Kp, ospDecimalValue<3> Ki, ospDecimalValue<3> Kd, byte ControllerDirection)
{
  // default output limit corresponds to
  // the arduino pwm limits
  PID::setOutputLimits(0, 255); 
 
  //default Controller Sample Time is 0.1 seconds
  SampleTime = 100;

  PID::setControllerDirection(ControllerDirection);
  PID::setTunings(Kp, Ki, Kd);

  lastTime = millis() - SampleTime;
  tuning = false;
  mode = MANUAL;
  myOutput = Output;
  myInput = Input;
  mySetpoint = Setpoint;
}
 
 
/* compute() **********************************************************************
* This, as they say, is where the magic happens. this function should be called
* every time "void loop()" executes. the function will decide for itself whether a new
* pid Output needs to be computed
**********************************************************************************/
void PID::compute()
{
  if (mode == MANUAL) 
  {
   return;
  }
  unsigned long now = millis();
  unsigned long timeChange = (now - lastTime);
  if (timeChange >= (unsigned long) SampleTime)
  {
    /*Compute all the working error variables*/
    double input = *myInput;
    double error = *mySetpoint - input;
    ITerm += (ki * error);
    limit(&ITerm);
    double dInput = (input - lastInput);
 
    // Compute PID Output
    double output = kp * error + ITerm - kd * dInput;
      
    limit(&output); 
    *myOutput = output;

    // Remember some variables for next time
    lastInput = input;
    lastTime = now;
  }
}

/* limit(...)*****************************************************************
*  Applies outMin and outMax limits to the supplied variable
******************************************************************************/
void PID::limit(double *var)
{
  if (*var > outMax) 
  {
    *var = outMax;
  }
  else if (*var < outMin) 
  {
    *var = outMin;
  }
}

/* setTunings(...)*************************************************************
* This function allows the controller's dynamic performance to be adjusted.
* it's called automatically from the constructor, but tunings can also
* be adjusted on the fly during normal operation
******************************************************************************/
void PID::setTunings(ospDecimalValue<3> Kp, ospDecimalValue<3> Ki, ospDecimalValue<3> Kd)
{
  if (
    (Kp < (ospDecimalValue<3>){0.0}) || 
    (Ki < (ospDecimalValue<3>){0.0}) || 
    (Kd < (ospDecimalValue<3>){0.0}) 
  )
  {
    return;
  }
 
  dispKp = Kp; 
  dispKi = Ki; 
  dispKd = Kd;
   
  double SampleTimeInSec = SampleTime * 0.001;
  kp = double(Kp);
  ki = double(Ki) * SampleTimeInSec;
  kd = double(Kd) / SampleTimeInSec;
 
  if (controllerDirection == REVERSE)
  {
    kp = (0.0 - kp);
    ki = (0.0 - ki);
    kd = (0.0 - kd);
  }
}
  
/* setSampleTime(...) *********************************************************
* sets the period, in Milliseconds, at which the calculation is performed
******************************************************************************/
void PID::setSampleTime(int NewSampleTime)
{
  if (NewSampleTime > 0)
  {
    double ratio = (double) NewSampleTime / (double) SampleTime;
    ki *= ratio;
    kd /= ratio;
    SampleTime = (unsigned long) NewSampleTime;
  }
}
 
/* setOutputLimits(...)****************************************************
* This function will be used far more often than SetInputLimits. while
* the input to the controller will generally be in the 0-1023 range (which is
* the default already,) the output will be a little different. maybe they'll
* be doing a time window and will need 0-8000 or something. or maybe they'll
* want to clamp it from 0-125. who knows. at any rate, that can all be done
* here.
**************************************************************************/
void PID::setOutputLimits(double Min, double Max)
{
  if (Min >= Max) 
  {
    return;
  }
  outMin = Min;
  outMax = Max;
 
  if (mode == AUTOMATIC)
  {
    limit(myOutput);
    limit(&ITerm);  
  }
}

/* setMode(...)****************************************************************
* Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
* when the transition from manual to auto occurs, the controller is
* automatically initialized
******************************************************************************/
void PID::setMode(byte newMode)
{
  if (newMode != mode)
  { 
    // we just went from manual to auto
    PID::initialize();
  }
  mode = newMode;
}
 
/* initialize()****************************************************************
* does all the things that need to happen to ensure a bumpless transfer
* from manual to automatic mode.
******************************************************************************/
void PID::initialize()
{
  ITerm = *myOutput;
  lastInput = *myInput;
  limit(&ITerm);
}

/* SetControllerDirection(...)*************************************************
* The PID will either be connected to a DIRECT acting process (+Output leads
* to +Input) or a REVERSE acting process(+Output leads to -Input.) we need to
* know which one, because otherwise we may increase the output when we should
* be decreasing. This is called from the constructor.
******************************************************************************/
void PID::setControllerDirection(byte Direction)
{
  if ((mode == AUTOMATIC) && (Direction != controllerDirection))
  {
    kp = (0.0 - kp);
    ki = (0.0 - ki);
    kd = (0.0 - kd);
  }
  controllerDirection = Direction;
}

/* setTuning(...)*************************************************
* set Boolean tuning variable
******************************************************************************/
void PID::setTuning(bool newTuning)
{
  tuning = newTuning;
}

/* Status Functions************************************************************
* Just because you set the Kp=-1 doesn't mean it actually happened. these
* functions query the internal state of the PID. they're here for display
* purposes. this are the functions the PID Front-end uses for example
******************************************************************************/
ospDecimalValue<3> PID::getKp() { return dispKp; }
ospDecimalValue<3> PID::getKi() { return dispKi; }
ospDecimalValue<3> PID::getKd() { return dispKd; }
byte PID::getMode() { return mode; }
byte PID::getDirection() { return controllerDirection; }
bool PID::isTuning() { return tuning; }
