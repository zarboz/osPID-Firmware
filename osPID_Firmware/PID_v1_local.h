#if !defined PID_v1_h
#define PID_v1_h
#define PID_LIBRARY_VERSION	1.0.0

#include "ospDecimalValue.h"

class PID
{
  public:

  // constants ***************************************************************************************
  
    // automatic or manual control
    static const byte MANUAL    = 0;
    static const byte AUTOMATIC = 1;
  
    // sign of controller gain
    static const byte DIRECT    = 0;
    static const byte REVERSE   = 1;

   // how often to step the PID loop, in milliseconds: it is impractical to set this
   // to less than ~1000 (i.e. faster than 1 Hz), since (a) the input has up to 750 ms
   // of latency, and (b) the controller needs time to handle the LCD, EEPROM, and serial I/O
     static const long LOOP_SAMPLE_TIME = 1000;

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

    void setTuning(bool);                 // set Boolean tuning variable

  // available but not commonly used functions ********************************************************

    void setTunings(ospDecimalValue<3>,   // * While most users will set the tunings once in the    
        ospDecimalValue<3>,         	  //   constructor, this function gives the user the option  
        ospDecimalValue<3>);              //   of changing tunings during runtime for Adaptive control

    void setControllerDirection(byte);	  // * Sets the Direction, or "Action" of the controller. DIRECT
					  //   means the output will increase when error is positive. REVERSE
				          //   means the opposite.  it's very unlikely that this will be needed
					  //   once it is set in the constructor.

    void setSampleTime(int);              // * sets the frequency, in Milliseconds, with which 
                                          //   the PID calculation is performed.  default is 100
										  
  // display functions ****************************************************************

    ospDecimalValue<3> getKp();           // These functions query the PID for interal values.
    ospDecimalValue<3> getKi();	          // they were created mainly for the PID front-end,
    ospDecimalValue<3> getKd();           // where it's important to know what is actually 
    byte getMode();			  // inside the PID.
    byte getDirection();		  //
    bool isTuning();                      //

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
    double ITerm, lastInput;

    int SampleTime;
    double outMin, outMax;
};
#endif


