#if !defined OSPCONFIG_H
#define OSPCONFIG_H

// global defines file included in the header of all .ino files

/*
 *          CONTROLLER IDENTITY
 */

// the controller name displayed in the startup banner and the identifY response
const char CONTROLLER_NAME[] = "Stripboard_osPID";
PROGMEM const char PcontrollerName[] = "Stripboard_osPID";

// the version tag displayed in the startup banner and the identifY response
const char VERSION_TAG[] = "v1.0";
PROGMEM const char Pversion[] = "v1.0";


/*
 *          HARDWARE  DEFINITIONS
 */

// pin assignment for LCD display
enum
{
  lcdRsPin     = 2, 
  lcdEnablePin = 3, 
  lcdD0Pin     = 7, 
  lcdD1Pin     = 6, 
  lcdD2Pin     = 5, 
  lcdD3Pin     = 4
};

// pin assignments for input devices 
enum { thermistorPin  = A0 };
enum { oneWireBus     = A0 };
enum 
{ 
  thermocoupleSO_Pin  = A0,
  thermocoupleCS_Pin  = A1,
  thermocoupleCLK_Pin = A2 
}; 

// pin assignment for SSR output
enum { SsrPin = A3 };

// pin assignment for analogue buttons
enum { buttonsPin = A4 };

// pin assignment for buzzer
enum { buzzerPin = A5 };


/*
 *          COMPILATION  OPTIONS
 */

// quiet mode (buzzer off) 
#undef SILENCE_BUZZER

// use Fahrenheit temperature units
// NB This option only changes the units for the
//    input sensor measurements. Temperature settings 
//    saved on the EEPROM -- set points, calibration values,
//    trip limits, and profile settings -- will not be upated
//    if this setting is changed.
#define UNITS_FAHRENHEIT
#if defined UNITS_FAHRENHEIT
const bool unitsFahrenheit = true;
#else
const bool unitsFahrenheit = false;
#endif

// use simulator for input/output
#undef USE_SIMULATOR

// omit serial processing commands for standalone controller
// setting this option will compile a hex file several kB shorter
#undef STANDALONE_CONTROLLER

// necessary omissions to compil on Atmega microcontrollers with 32 kB flash
#define ATMEGA_32kB_FLASH

// NB test compilation length using longest options: #undef STANDALONE_CONTROLLER, USE_SIMULATOR, SILENCE_BUZZER, and #define UNITS_FAHRENHEIT

// default auto tune algorithm and parameters
#define AUTO_TUNE_DEFAULT_METHOD                0       // ZIEGLER_NICHOLS_PI
#define AUTO_TUNE_DEFAULT_OUTPUT_STEP           20
#define AUTO_TUNE_DEFAULT_NOISE_BAND_CELSIUS    0.5
#define AUTO_TUNE_DEFAULT_LOOKBACK_SEC          10

// Default parameters forthermistor
const double THERMISTOR_NOMINAL_RESISTANCE   = 10.0f;
const double THERMISTOR_B_COEFFICIENT        = 1.0f;
const double THERMISTOR_TEMPERATURE_NOMINAL  = 293.15;
const double THERMISTOR_REFERENCE_RESISTANCE = 10.0f;

// how often to step the PID loop, in milliseconds: it is impractical to set this
// to less than ~1000 (i.e. faster than 1 Hz), since (a) the input has up to 750 ms
// of latency, and (b) the controller needs time to handle the LCD, EEPROM, and serial
// I/O
enum { PID_LOOP_SAMPLE_TIME = 1000 };

// minimum refresh rate for input measurements
// NB OneWire devices have a considerably longer latency than this
enum { INPUT_MINIMUM_SAMPLE_TIME = 100 };

// autorepeat constants, in milliseconds
enum 
{
  AUTOREPEAT_DELAY  = 250,
  AUTOREPEAT_PERIOD = 350
};
 
// power-on options
enum 
{
  POWERON_DISABLE = 0,
  POWERON_CONTINUE_LOOP,
  POWERON_RESUME_PROFILE
};

#define DEFAULT_POWER_ON_BEHAVIOR POWERON_CONTINUE_LOOP

#endif
