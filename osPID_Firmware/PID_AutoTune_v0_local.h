#if !defined PID_AutoTune_v0
#define PID_AutoTune_v0
#define AUTO_TUNE_LIBRARY_VERSION 0.0.1

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "ospConfig.h"
#include "ospDecimalValue.h"

// compilation options

// verbose debug option
// requires open Serial port
#undef AUTOTUNE_DEBUG

// AMIGOf tuning rule
// we have made space for this, yay!
  #define AUTOTUNE_AMIGOF_PI

// defining this option implements relay bias
// this is useful to adjust the relay output values
// during the auto tuning to recover symmetric
// oscillations 
// this can compensate for load disturbance
// and equivalent signals arising from nonlinear
// or non-stationary processes 
// any improvement in the tunings seems quite modest 
// but sometimes unbalanced oscillations can be 
// persuaded to converge where they might not 
// otherwise have done so
#undef AUTOTUNE_RELAY_BIAS

// Ziegler-Nichols type auto tune rules
// in tabular form
struct Tuning
{
  byte _divisor[3];
  
  bool PI_controller()
  {
    return pgm_read_byte_near(&_divisor[2]) == 0;
  }
  
  double divisor(byte index)  
  {
    return (double)pgm_read_byte_near(&_divisor[index]) * 0.05;
  }
};

class PID_ATune
{

public:
  // constants **********************************************************************************
  
  // configurable tolerances
  //
  // average amplitude of successive peaks must differ by no more than this proportion
  // relative to half the difference between maximum and minimum of last 2 cycles
  static const double PEAK_AMPLITUDE_TOLERANCE = 0.05;
  //
  // ratio of up/down relay step duration should differ by no more than this tolerance
  // biasing the relay con give more accurate estimates of the tuning parameters but
  // setting the tolerance too low will prolong the autotune procedure unnecessarily
  // this parameter also sets the minimum bias in the relay as a proportion of its amplitude
  static const double STEP_ASYMMETRY_TOLERANCE = 0.20;
  //
  // auto tune terminates if waiting too long between peaks or relay steps
  // set larger value for processes with long delays or time constants
  static const unsigned long MAX_WAIT = 5 * 60 * 1000; // 5 minutes
  
  // auto tune methods
  enum
  {
    ZIEGLER_NICHOLS_PI    = 0,
    ZIEGLER_NICHOLS_PID,
    TYREUS_LUYBEN_PI,
    TYREUS_LUYBEN_PID,
    CIANCONE_MARLIN_PI,
    CIANCONE_MARLIN_PID,
    PESSEN_INTEGRAL_PID,
    SOME_OVERSHOOT_PID,
    NO_OVERSHOOT_PID,
  
#if defined AUTOTUNE_AMIGOF_PI  
    AMIGOF_PI,
    LAST_AUTO_TUNE_METHOD = AMIGOF_PI
#else  
    LAST_AUTO_TUNE_METHOD = NO_OVERSHOOT_PID
#endif

  };

  // peak types
  enum
  {
    NOT_A_PEAK,
    MINIMUM,
    MAXIMUM  
  };
  
  // auto tuner states
  enum
  {
    AUTOTUNER_OFF              =   0, 
    STEADY_STATE_AT_BASELINE   =   1,
    STEADY_STATE_AFTER_STEP_UP =   2,
    RELAY_STEP_UP              =   4,
    RELAY_STEP_DOWN            =   8,
    CONVERGED                  =  16,
    FAILED                     = 128
  };

  // tuning rule divisor indexes
  enum
  {
    KP_DIVISOR = 0,
    TI_DIVISOR,
    TD_DIVISOR 
  };
  
  // irrational constants
  static const double CONST_PI          = 3.14159265358979323846;
  static const double CONST_PI_DIV_2    = 1.57079632679489661923;
  static const double CONST_SQRT2_DIV_2 = 0.70710678118654752440;
  
  // default auto tune algorithm and parameters
  static const byte   DEFAULT_METHOD             = ZIEGLER_NICHOLS_PID;	
  static const int    DEFAULT_OUTPUT_STEP        = 100;
  static const double DEFAULT_NOISE_BAND_CELSIUS = 0.5;
  static const int    DEFAULT_LOOKBACK_SEC       = 10;

  // commonly used methods **********************************************************************
  PID_ATune(double*, double*);          // * Constructor.  links the Autotune to a given PID
  bool Runtime();                       // * Similar to the PID Compute function, 
                                        //   returns true when done, otherwise returns false
  void Cancel();                        // * Stops the AutoTune 

  void SetOutputStep(                   // * how far above and below the starting value will
      ospDecimalValue<1>);              //   the output step?   
  double GetOutputStep();               // 

  void SetControlType(byte);            // * Determines tuning algorithm
  byte GetControlType();                // * Returns tuning algorithm

  void SetLookbackSec(int);             // * how far back are we looking to identify peaks
  int GetLookbackSec();                 //

  void SetNoiseBand(                    // * the autotune will ignore signal chatter smaller 
      ospDecimalValue<1>);              //   than this value
  double GetNoiseBand();                //   this should be accurately set

  double GetKp();                       // * once autotune is complete, these functions contain the
  double GetKi();                       //   computed tuning parameters.  
  double GetKd();                       //

private:
  double processValueOffset(double,     // * returns an estimate of the process value offset
     double);                           //   as a proportion of the amplitude 
                                                                               

  double *input;
  double *output;
  double setpoint;

  double oStep;
  double noiseBand;
  byte nLookBack;
  byte controlType;                     // * selects autotune algorithm

  byte state;                           // * state of autotuner finite state machine
  unsigned long lastTime;
  unsigned long sampleTime;
  byte peakType;
  unsigned long lastPeakTime[5];        // * peak time, most recent in array element 0
  double lastPeaks[5];                  // * peak value, most recent in array element 0
  byte peakCount;
  double inputOffset;
  ospDecimalValue<3> inputOffsetChange;
  ospDecimalValue<3> lastInputs[101];   // * process values, most recent in array element 0
  byte inputCount;
  double outputStart;
  double inducedAmplitude;
  double Kp, Ti, Td;

#if defined AUTOTUNE_AMIGOF_PI  
  double calculatePhaseLag(double);     // * calculate phase lag from noiseBand and inducedAmplitude
  double fastArcTan(double);
  double originalNoiseBand;
  double newNoiseBand;
  double K_process;
#endif
  
#if defined AUTOTUNE_RELAY_BIAS  
  double relayBias;
  unsigned long lastStepTime[5];        // * step time, most recent in array element 0
  double sumInputSinceLastStep[5];      // * integrated process values, most recent in array element 0
  byte stepCount;
#endif  
};

#endif
