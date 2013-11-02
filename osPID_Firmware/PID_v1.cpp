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
        ospDecimalValue<3> Kp, ospDecimalValue<3> Ki, ospDecimalValue<3> Kd, int ControllerDirection)
{
  // default output limit corresponds to
  // the arduino pwm limits
  PID::SetOutputLimits(0, 255); 
 
  //default Controller Sample Time is 0.1 seconds
  SampleTime = 100;

  PID::SetControllerDirection(ControllerDirection);
  PID::SetTunings(Kp, Ki, Kd);

  lastTime = millis() - SampleTime;
  inAuto = false;
  myOutput = Output;
  myInput = Input;
  mySetpoint = Setpoint;
}
 
 
/* Compute() **********************************************************************
* This, as they say, is where the magic happens. this function should be called
* every time "void loop()" executes. the function will decide for itself whether a new
* pid Output needs to be computed
**********************************************************************************/
void PID::Compute()
{
  if (!inAuto) 
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
    Limit(&ITerm);
    double dInput = (input - lastInput);
 
    // Compute PID Output
    double output = kp * error + ITerm - kd * dInput;
      
    Limit(&output); 
    *myOutput = output;

    // Remember some variables for next time
    lastInput = input;
    lastTime = now;
  }
}

/* Limit(...)*****************************************************************
*  Applies outMin and outMax limits to the supplied variable
******************************************************************************/
void PID::Limit(double *var)
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

/* SetTunings(...)*************************************************************
* This function allows the controller's dynamic performance to be adjusted.
* it's called automatically from the constructor, but tunings can also
* be adjusted on the fly during normal operation
******************************************************************************/
void PID::SetTunings(ospDecimalValue<3> Kp, ospDecimalValue<3> Ki, ospDecimalValue<3> Kd)
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
  
/* SetSampleTime(...) *********************************************************
* sets the period, in Milliseconds, at which the calculation is performed
******************************************************************************/
void PID::SetSampleTime(int NewSampleTime)
{
  if (NewSampleTime > 0)
  {
    double ratio = (double) NewSampleTime / (double) SampleTime;
    ki *= ratio;
    kd /= ratio;
    SampleTime = (unsigned long) NewSampleTime;
  }
}
 
/* SetOutputLimits(...)****************************************************
* This function will be used far more often than SetInputLimits. while
* the input to the controller will generally be in the 0-1023 range (which is
* the default already,) the output will be a little different. maybe they'll
* be doing a time window and will need 0-8000 or something. or maybe they'll
* want to clamp it from 0-125. who knows. at any rate, that can all be done
* here.
**************************************************************************/
void PID::SetOutputLimits(double Min, double Max)
{
  if (Min >= Max) 
  {
    return;
  }
  outMin = Min;
  outMax = Max;
 
  if (inAuto)
  {
    Limit(myOutput);
    Limit(&ITerm);  
  }
}

/* SetMode(...)****************************************************************
* Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
* when the transition from manual to auto occurs, the controller is
* automatically initialized
******************************************************************************/
void PID::SetMode(byte Mode)
{
  bool newAuto = (Mode == AUTOMATIC);
  if (newAuto == !inAuto)
  { 
    // we just went from manual to auto
    PID::Initialize();
  }
  inAuto = newAuto;
}
 
/* Initialize()****************************************************************
* does all the things that need to happen to ensure a bumpless transfer
* from manual to automatic mode.
******************************************************************************/
void PID::Initialize()
{
  ITerm = *myOutput;
  lastInput = *myInput;
  Limit(&ITerm);
}

/* SetControllerDirection(...)*************************************************
* The PID will either be connected to a DIRECT acting process (+Output leads
* to +Input) or a REVERSE acting process(+Output leads to -Input.) we need to
* know which one, because otherwise we may increase the output when we should
* be decreasing. This is called from the constructor.
******************************************************************************/
void PID::SetControllerDirection(byte Direction)
{
  if (inAuto && (Direction != controllerDirection))
  {
    kp = (0.0 - kp);
    ki = (0.0 - ki);
    kd = (0.0 - kd);
  }
  controllerDirection = Direction;
}

/* Status Funcions*************************************************************
* Just because you set the Kp=-1 doesn't mean it actually happened. these
* functions query the internal state of the PID. they're here for display
* purposes. this are the functions the PID Front-end uses for example
******************************************************************************/
ospDecimalValue<3> PID::GetKp(){ return dispKp; }
ospDecimalValue<3> PID::GetKi(){ return dispKi;}
ospDecimalValue<3> PID::GetKd(){ return dispKd;}
byte PID::GetMode(){ return inAuto ? AUTOMATIC : MANUAL;}
byte PID::GetDirection(){ return controllerDirection;}


