#if !defined (OSPID_ENGINE_H)
#define OSPID_ENGINE_H
#define OSPID_ENGINE_VERSION   2.0.0

#include "ospDecimalValue.h"

// compilation options for auto tune

// verbose debug option
// requires open Serial port
#undef AUTOTUNE_DEBUG

// AMIGOf tuning rule
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

class PID
{
  public:
  
  // static constants ****************************************************************************
  
    // configurable constants
  
    // how often to step the PID loop, in milliseconds: it is impractical to set this
    // to less than ~1000 (i.e. faster than 1 Hz), since (a) the input has up to 750 ms
    // of latency, and (b) the controller needs time to handle the LCD, EEPROM, and serial I/O
#if !defined (USE_SIMULATOR)
    static const long DEFAULT_LOOP_SAMPLE_TIME = 1000;
#else
    static const long DEFAULT_LOOP_SAMPLE_TIME = 250;
#endif
  
    // average amplitude of successive peaks must differ by no more than this proportion
    // relative to half the difference between maximum and minimum of last 2 cycles
    static const double AUTOTUNE_PEAK_AMPLITUDE_TOLERANCE = 0.05;
  
    // ratio of up/down relay step duration should differ by no more than this tolerance
    // biasing the relay con give more accurate estimates of the tuning parameters but
    // setting the tolerance too low will prolong the autotune procedure unnecessarily
    // this parameter also sets the minimum bias in the relay as a proportion of its amplitude
    static const double AUTOTUNE_STEP_ASYMMETRY_TOLERANCE = 0.20;
  
    // auto tune terminates if waiting too long between peaks or relay steps
    // set larger value for processes with long delays or time constants
    static const unsigned long AUTOTUNE_MAX_WAIT = 5 * 60 * (unsigned long) 1000; // 5 minutes    
    
    // automatic or manual control
    static const byte MANUAL    = 0;
    static const byte AUTOMATIC = 1;
  
    // sign of controller gain
    static const byte DIRECT    = 0;
    static const byte REVERSE   = 1;
  
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
      LAST_AUTOTUNE_METHOD = AMIGOF_PI
#else  
      LAST_AUTOTUNE_METHOD = NO_OVERSHOOT_PID
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
      AUTOTUNE_OFF                        =   0, 
      AUTOTUNE_STEADY_STATE_AT_BASELINE   =   1,
      AUTOTUNE_STEADY_STATE_AFTER_STEP_UP =   2,
      AUTOTUNE_RELAY_STEP_UP              =   4,
      AUTOTUNE_RELAY_STEP_DOWN            =   8,
      AUTOTUNE_CONVERGED                  =  16,
      AUTOTUNE_FAILED                     = 128
    }; 

    // tuning rule divisor indexes
    enum
    {
      AUTOTUNE_KP_DIVISOR = 0,
      AUTOTUNE_TI_DIVISOR,
      AUTOTUNE_TD_DIVISOR 
    };
  
    // irrational constants
    static const double CONST_PI          = 3.14159265358979323846;
    static const double CONST_PI_DIV_2    = 1.57079632679489661923;
    static const double CONST_SQRT2_DIV_2 = 0.70710678118654752440;
  
    // default auto tune algorithm and parameters
    static const byte   AUTOTUNE_DEFAULT_METHOD             = ZIEGLER_NICHOLS_PID;	
    static const int    AUTOTUNE_DEFAULT_OUTPUT_STEP        = 100;
    static const double AUTOTUNE_DEFAULT_NOISE_BAND_CELSIUS = 0.5;
    static const int    AUTOTUNE_DEFAULT_LOOKBACK_SEC       = 10;

  // commonly used functions **************************************************************************

    PID(double*, double*, double*,        // * constructor.  links the PID to the Input, Output, and 
        ospDecimalValue<3>,               //   Setpoint.  Initial tuning parameters are also set here
        ospDecimalValue<3>,
        ospDecimalValue<3>, byte);
	
    void setMode(byte);                   // * sets PID to either Manual (0) or Auto (1)

    void compute();                       // * performs the PID calculation.  it should be
                                          //   called every time loop() cycles. ON/OFF and
                                          //   calculation frequency can be set using SetMode
                                          //   SetSampleTime respectively

    void setOutputLimits(double, double); // clamps the output to a specific range. 0-255 by default, but
					  // it's likely the user will want to change this depending on
					  // the application

  // public auto tuner methods ***********************************************************************
    
    void startAutoTune(byte,              // start auto tune
      ospDecimalValue<1>,
      ospDecimalValue<3>,
      int);
      
    void stopAutoTune();                  // cancel auto tuning

    void setAtuneOutputStep(              // * how far above and below the starting value will
      ospDecimalValue<1>);                //   the output step?   
    double getAtuneOutputStep();          // 

    void setAtuneControlType(byte);       // * Determines tuning algorithm
    byte getAtuneControlType();           // * Returns tuning algorithm

    void setAtuneLookBackSec(int);        // * how far back are we looking to identify peaks
    int getAtuneLookBackSec();            //

    void setAtuneNoiseBand(               // * the autotune will ignore signal chatter smaller 
      ospDecimalValue<3>);                //   than this value
    double getAtuneNoiseBand();           //   this should be accurately set

    double getAtuneKp();                  // * once autotune is complete, these functions contain the
    double getAtuneKi();                  //   computed tuning parameters.  
    double getAtuneKd();                  //

  // available but not commonly used functions ********************************************************

    void setTunings(ospDecimalValue<3>,   // * While most users will set the tunings once in the    
        ospDecimalValue<3>,         	  //   constructor, this function gives the user the option  
        ospDecimalValue<3>);              //   of changing tunings during runtime for adaptive control

    void setControllerDirection(byte);	  // * Sets the Direction, or "Action" of the controller. DIRECT
					  //   means the output will increase when error is positive. REVERSE
				          //   means the opposite.  it's very unlikely that this will be needed
					  //   once it is set in the constructor.

    void setSampleTime(int);              // * sets the frequency, in Milliseconds, with which 
                                          //   the PID calculation is performed.  default is 100
										  
  // display functions *********************************************************************************

    ospDecimalValue<3> getKp();           // These functions query the PID for interal values.
    ospDecimalValue<3> getKi();	          // they were created mainly for the PID front-end,
    ospDecimalValue<3> getKd();           // where it's important to know what is actually 
    byte getMode();			  // inside the PID.
    byte getDirection();		  //
    bool isTuning();                      //
										  
  // private variables and methods **********************************************************************

  private:
  
    void initialize();
    void limit(double*);
	
    ospDecimalValue<3> dispKp;            // * we'll hold on to the tuning parameters in user-entered 
    ospDecimalValue<3> dispKi;            //   format for display purposes
    ospDecimalValue<3> dispKd;            //
    
    double kp;                            // * (P)roportional Tuning Parameter
    double ki;                            // * (I)ntegral Tuning Parameter
    double kd;                            // * (D)erivative Tuning Parameter

    byte controllerDirection;

    double *myInput;                      // * Pointers to the Input, Output, and Setpoint variables
    double *myOutput;                     //   This creates a hard link between the variables and the 
    double *mySetpoint;                   //   PID, freeing the user from having to constantly tell us
                                          //   what these values are.  with pointers we'll just know.
			  
    bool tuning;                          // * flag whether auto tune is running for this PID

    byte mode;                            // * automatic or manual control

    unsigned long lastTime;
    double iTerm, lastInput;

    int sampleTime;
    double outMin, outMax;
										  
  // private variables and methods for auto tuner *********************************************************
  
    bool autoTune();                      // * called from PID compute function, 
                                          //   returns true when done, otherwise returns false
                                          
    void completeAutoTune();              // * set tunings and finish

    byte ATuneModeRemember;
    ospDecimalValue<1> manualOutputRemember;
    
    double oStep;
    double noiseBand;
    byte nLookBack;
    byte controlType;                     // * selects autotune algorithm

    byte state;                           // * state of autotuner finite state machine    
    double setpoint;
    double outputStart;
    double workingNoiseBand;
    double workingOstep;
    byte peakType;
    unsigned long lastPeakTime[5];        // * peak time, most recent in array element 0
    double lastPeaks[5];                  // * peak value, most recent in array element 0
    byte peakCount;
    double inputOffset;
    ospDecimalValue<3> inputOffsetChange;
    ospDecimalValue<3> lastInputs[101];   // * process values, most recent in array element 0
    byte inputCount;
    double Kp, Ti, Td;

#if defined AUTOTUNE_AMIGOF_PI  
    double calculatePhaseLag(double);     // * calculate phase lag from noiseBand and inducedAmplitude
    double fastArcTan(double);
    double newWorkingNoiseBand;
    double K_process;
#endif
  
#if defined AUTOTUNE_RELAY_BIAS  
    double processValueOffset(double,     // * returns an estimate of the process value offset
      double);                            //   as a proportion of the amplitude 
    double relayBias;
    unsigned long lastStepTime[5];        // * step time, most recent in array element 0
    double sumInputSinceLastStep[5];      // * integrated process values, most recent in array element 0
    byte stepCount;
#endif  

};

#endif


