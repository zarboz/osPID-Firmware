#if !defined (OSPCONFIG_H)
#define OSPCONFIG_H

// global defines file included in the header of all .ino files

/********************************************
 *
 *          CONTROLLER IDENTITY
 *
 ********************************************/

// the controller name displayed in the startup banner and the identifY response
static const char CONTROLLER_NAME[] = "DabPer_Enail";
PROGMEM const char PcontrollerName[] = "DabPer_Enail";

// the version tag displayed in the startup banner and the identifY response
static const char VERSION_TAG[] = "By Dabper and friends";
PROGMEM const char Pversion[] = "By Dabper and friends";

/********************************************
 *
 *          HARDWARE  DEFINITIONS
 *
 ********************************************/

// pin assignment for LCD display
static const byte lcdRsPin     = 7; 
static const byte lcdEnablePin = 8; 
static const byte lcdD4Pin     = 9; 
static const byte lcdD5Pin     = 10; 
static const byte lcdD6Pin     = 11; 
static const byte lcdD7Pin     = 12;
static const byte lcdREDPin    = 3;
static const byte lcdGRNPin    = 5;
static const byte lcdBLUPin    = 6;
//misc LCD shit
static const int  brightness   = 255;


// pin assignments for input devices 
static const byte thermistorPin       = A0;
static const byte oneWireBus          = A0;
static const byte thermocoupleSO_Pin  = A0;
static const byte thermocoupleCS_Pin  = A1;
static const byte thermocoupleCLK_Pin = A2;

// pin assignment for SSR output
static const byte SsrPin              = A3;

// pin assignment for analogue buttons
static const byte buttonsPin          = A4;

// pin assignment for buzzer
static const byte buzzerPin           = A5;

/********************************************
 *
 *          COMPILATION  OPTIONS
 *
 ********************************************/

// quiet mode (buzzer off) 
#define SILENCE_BUZZER

// use Fahrenheit temperature units
// NB This option only changes the units for the
//    input sensor measurements. Temperature settings 
//    saved on the EEPROM -- set points, calibration values,
//    trip limits, and profile settings -- will not be upated
//    if this setting is changed.
#define UNITS_FAHRENHEIT  // we will need to use this flag for US distributed units and #undef it for UK units or units that use metric measurements
#if defined (UNITS_FAHRENHEIT)
static const bool unitsFahrenheit = true;
#else
static const bool unitsFahrenheit = false;
#endif

// use simulator for input/output
#undef USE_SIMULATOR

// necessary omissions to compile on Atmega microcontrollers with 32 kB flash
#if defined (__AVR_ATmega328P__) || defined (__AVR_ATmega32U4__)
#define ATMEGA_32kB_FLASH
#else
#undef ATMEGA_32kB_FLASH
#endif

// omit serial processing commands for standalone controller
// setting this option will compile a hex file several kB shorter
#define STANDALONE_CONTROLLER

// Leonardo bootloader is larger (4K) so we don't have room for serial communication
#if defined (__AVR_ATmega32U4__)
#define STANDALONE_CONTROLLER
#endif

// default serial communication speed (baud rate)
static const long baudrate = 9600;

// NB test compilation length using longest options: #undef STANDALONE_CONTROLLER, USE_SIMULATOR, SILENCE_BUZZER, and #define UNITS_FAHRENHEIT

// autorepeat constants, in milliseconds
static const int AUTOREPEAT_DELAY  = 250;
static const int AUTOREPEAT_PERIOD = 350;
 
#define DEFAULT_POWER_ON_BEHAVIOR POWERON_CONTINUE_LOOP

// NB Auto tune defaults in PID_AutoTune header file

// NB Thermistor default constants in ospInputDevice.h header file

#endif
