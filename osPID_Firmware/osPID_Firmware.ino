//* This file contains the setup() and loop() logic for the controller. */

#include <Arduino.h>
#include <LiquidCrystal.h>
#include <avr/pgmspace.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "ospConfig.h"
#include "ospDecimalValue.h"
#include "PID_v1_local.h"
#include "PID_AutoTune_v0_local.h"
#include "ospAnalogButton.h"
#include "ospProfile.h"

#undef BUGCHECK
#define BUGCHECK() ospBugCheck(PSTR("MAIN"), __LINE__);

/*******************************************************************************
 * The stripboard PID Arduino shield uses firmware based on the osPID but
 * simplified hardware. Instead of swappable output cards there is a simple
 * 5V logical output that can drive an SSR. In place of output cards there
 * is a terminal block that can be used for a 1-wire bus or NTC thermistor,
 * and female pin headers that can interface a MAX31855 thermocouple amplifier
 * breakout board. Each of these inputs is supported by different
 * device drivers & libraries. The input in use is specified by a menu option.
 * This saves having to recompile the firmware when changing input sensors.
 *
 * Inputs
 * ======
 *    DS18B20+ 1-wire digital thermometer with data pin on A0, OR
 *    10K NTC thermistor with voltage divider input on pin A0, OR
 *    MAX31855KASA interface to type-K thermocouple on pins A0-A2.
 *
 * Output
 * ======
 *    1 SSR output on pin A3.
 *
 * For firmware development, there is also the ospSimulator which acts as both
 * the input and output device and simulates the controller being attached to a
 * simple system.
 *******************************************************************************/




#if !defined USE_SIMULATOR
#include "ospOutputDeviceSsr.h"
#include "ospInputDevice.h"
enum { numInputDevices = 3 };
enum { numOutputDevices = 1 };
ospInputDevice theInputDevice;
ospOutputDeviceSsr theOutputDevice;
#else
#include "ospSimulator.h"
enum { numInputDevices = 1 };
enum { numOutputDevices = 1 };
ospSimulator theInputDevice;
#define theOutputDevice theInputDevice
#endif



// we use the LiquidCrystal library to drive the LCD screen
LiquidCrystal theLCD(lcdRsPin, lcdEnablePin, lcdD0Pin, lcdD1Pin, lcdD2Pin, lcdD3Pin);
extern void __attribute__ ((noinline)) LCDprintln(const PROGMEM char* s);

// our AnalogButton library provides debouncing and interpretation
// of the analog-multiplexed button channel
ospAnalogButton<buttonsPin, 100, 253, 454, 657> theButtonReader;

// an in-memory buffer that we use when receiving a profile over USB
ospProfile profileBuffer;

// the 0-based index of the active profile while a profile is executing
byte activeProfileIndex;
byte currentProfileStep;
boolean runningProfile = false;

// the gain coefficients of the PID controller
ospDecimalValue<3> PGain = { 1000 }, IGain = { 0 }, DGain = { 0 };

// the direction flag for the PID controller
byte ctrlDirection = PID::DIRECT;

// whether the controller is executing a PID law or just outputting a manual
// value
byte modeIndex = PID::MANUAL;

// the 4 setpoints we can easily switch between
#if !defined UNITS_FAHRENHEIT
ospDecimalValue<1> setPoints[4] = { { 250 }, { 650 }, { 1000 }, { 1250 } };
#else
ospDecimalValue<1> setPoints[4] = { { 800 }, { 1500 }, { 2120 }, { 2600 } };
#endif

// the index of the selected setpoint
byte setPointIndex = 0;

// set value for PID controller
double activeSetPoint;

// the most recent measured input value
double input = NAN; 

// last good input value, used by PID controller
double lastGoodInput = 25.0;

// the output duty cycle calculated by PID controller
double output = 0.0;   

// the manually-commanded output value
ospDecimalValue<1> manualOutput = { 0 };

// temporary fixed point decimal values for display and data entry
ospDecimalValue<1> displaySetpoint = { 250 }, displayInput = { -19999 }, displayCalibration = { 0 }, 
  displayWindow = { 50 }; 

// the hard trip limits
#if !defined UNITS_FAHRENHEIT
ospDecimalValue<1> lowerTripLimit = { 0 } , upperTripLimit = { 1250 };
#else
ospDecimalValue<1> lowerTripLimit = { 0 } , upperTripLimit = { 2600 };
#endif
bool tripLimitsEnabled;
bool tripped;
bool tripAutoReset;

byte powerOnBehavior = DEFAULT_POWER_ON_BEHAVIOR;

bool controllerIsBooting = true;

// auto tune algorithm
byte aTuneMethod = AUTO_TUNE_DEFAULT_METHOD;

// the parameters for the autotuner
ospDecimalValue<1> aTuneStep  = { AUTO_TUNE_DEFAULT_OUTPUT_STEP * 10 };
#if !defined UNITS_FAHRENHEIT
ospDecimalValue<1> aTuneNoise = { AUTO_TUNE_DEFAULT_NOISE_BAND_CELSIUS * 18 }; 
#else 
ospDecimalValue<1> aTuneNoise = { AUTO_TUNE_DEFAULT_NOISE_BAND_CELSIUS * 10 }; 
#endif
int aTuneLookBack = AUTO_TUNE_DEFAULT_LOOKBACK_SEC; 
PID_ATune aTune(&lastGoodInput, &output);

// whether the autotuner is active
bool tuning = false;

// the actual PID controller
PID myPID(&lastGoodInput, &output, &activeSetPoint, PGain, IGain, DGain, PID::DIRECT);

// timekeeping to schedule the various tasks in the main loop
unsigned long now, lcdTime, readInputTime;

extern void drawNotificationCursor(char icon);

// some constants in flash memory, for reuse

#if !defined UNITS_FAHRENHEIT
const __FlashStringHelper *FdegCelsius() { return F(" °C"); }
#else
const __FlashStringHelper *FdegFahrenheit() { return F(" °F"); }
#endif

const PROGMEM char Pprofile[] = "Profile ";

char hex(byte b)
{
  return ((b < 10) ? (char) ('0' + b) : (char) ('A' - 10 + b));
}

void __attribute__ ((noinline)) updateTimer()
{
  now = millis();
}

// check time avoiding overflow
bool __attribute__ ((noinline)) after(unsigned long targetTime)
{
  unsigned long u = (targetTime - now);
  return ((u & 0x80000000) > 0);
}

#define MINIMUM(a,b) (((a)<(b))?(a):(b))
#define MAXIMUM(a,b) (((b)>(a))?(a):(b))

#if !defined SILENCE_BUZZER
// buzzer 
volatile int buzz = 0; // countdown timer for intermittent buzzer
enum { BUZZ_OFF = 0, BUZZ_UNTIL_CANCEL = -769 };
#define buzzMillis(x)    buzz = (x)*4
#define buzzUntilCancel  buzz = BUZZ_UNTIL_CANCEL
#define buzzOff          buzz = BUZZ_OFF

ISR (TIMER2_COMPA_vect)
{
  const byte     buzzerPort     = digitalPinToPort(buzzerPin);
  volatile byte *buzzerOut      = portOutputRegister(buzzerPort);
  const byte     buzzerBitMask  = digitalPinToBitMask(buzzerPin);

  // buzz counts down 
  if (buzz == BUZZ_OFF)
  {
    // write buzzer pin low
    *buzzerOut &= ~buzzerBitMask;
  }
  else
  {      
    if (((unsigned int)buzz & 0x3F0) == 0) // intermittent chirp
    {
      // toggle buzzer pin to make tone
      *buzzerOut ^= buzzerBitMask;
    }
    else
    {
      // write buzzer pin low
      *buzzerOut &= ~buzzerBitMask;
    }
    buzz--;
    if (buzz == BUZZ_UNTIL_CANCEL - 1024)
    {
      buzz = BUZZ_UNTIL_CANCEL;
    }
  }
}
#endif

void __attribute__((noinline)) setOutputToManualOutput()
{
  output = double(manualOutput);
}

void __attribute__((noinline)) updateActiveSetPoint()
{
  activeSetPoint = double(setPoints[setPointIndex]);
}




// initialize the controller: this is called by the Arduino runtime on bootup
void setup()
{
  // initialize pins
  for (byte pin = 8; pin < 13; pin++)
  {
    pinMode(pin, INPUT_PULLUP);
  }
  pinMode(buzzerPin, OUTPUT);
  
  // set up timer2 for buzzer interrupt
#if !defined SILENCE_BUZZER
  cli();                   // disable interrupts
  OCR2A = 249;              // set up timer2 CTC interrupts for buzzer
  TCCR2A |= (1 << WGM21);  // CTC Mode
  TIMSK2 |= (1 << OCIE2A); // set interrupt on compare match
  GTCCR  |= (1 << PSRASY); // reset timer2 prescaler
  TCCR2B |= (7 << CS20);   // prescaler 1024
  sei();                   // enable interrupts
#endif  

  // set up the LCD,show controller name
  theLCD.begin(16, 2);
  drawStartupBanner();

  lcdTime = 25;
  updateTimer();

  // load the EEPROM settings
  setupEEPROM();
  
  // set up the peripheral devices
  theInputDevice.initialize();
  theOutputDevice.initialize();

  // set up the serial interface
#if !defined STANDALONE_CONTROLLER
  setupSerial();
#endif

  long int pause = now + 2000 - millis();
  if (pause < 10)
  {
    pause = 10;
  }
  delay(pause);
  updateTimer();

  // configure the PID loop 
  updateActiveSetPoint();
  myPID.SetSampleTime(PID_LOOP_SAMPLE_TIME);
  myPID.SetOutputLimits(0, 100);
  myPID.SetTunings(PGain, IGain, DGain);
  myPID.SetControllerDirection(ctrlDirection);

  if (powerOnBehavior == POWERON_DISABLE) 
  {
    modeIndex = PID::MANUAL;
    setOutputToManualOutput();
  }
  myPID.SetMode(modeIndex);

  // finally, check whether we were interrupted in the middle of a profile
  if (profileWasInterrupted())
  {
    if (powerOnBehavior == POWERON_RESUME_PROFILE)
    {
      drawResumeProfileBanner();
      runningProfile = true;
      startCurrentProfileStep();
    }
    else
    {
      recordProfileCompletion(); // we don't want to pick up again, so mark it completed
    }
  }

  // kick things off by requesting sensor input
  updateTimer();
  if (theInputDevice.getInitializationStatus())
  {
    readInputTime = now + theInputDevice.requestInput();
  }

  controllerIsBooting = false;
}

// Letting a button auto-repeat without redrawing the LCD in between leads to a
// poor user interface
bool lcdRedrawNeeded;

// keep track of which button is being held, and for how long
byte heldButton;
byte autoRepeatCount;
unsigned long autoRepeatTriggerTime;

// test the buttons and look for button presses or long-presses
static void checkButtons()
{
  byte button = theButtonReader.get();
  byte executeButton = BUTTON_NONE;

  if (button != BUTTON_NONE)
  {
    if (heldButton == BUTTON_NONE)
    {
      autoRepeatTriggerTime = now + AUTOREPEAT_DELAY;
    }
    else if (heldButton == BUTTON_OK)
    {
      // OK does long-press/short-press, not auto-repeat
    }
    else if (after(autoRepeatTriggerTime))
    {
      // don't auto-repeat until 100 ms after the redraw
      if (lcdRedrawNeeded)
      {
        autoRepeatTriggerTime = now + 150;
        return;
      }

      // auto-repeat
      executeButton = button;
      autoRepeatCount += 1;
      autoRepeatTriggerTime = now + AUTOREPEAT_PERIOD;
    }
    heldButton = button;
  }
  else if (heldButton != BUTTON_NONE)
  {
    if (heldButton == BUTTON_OK && (after(autoRepeatTriggerTime + 400 - AUTOREPEAT_DELAY)))
    {
      // BUTTON_OK was held for at least 400 ms: execute a long-press
      bool longPress = okKeyLongPress();

      if (!longPress)
      {
        // no long-press action defined, so fall back to a short-press
        executeButton = BUTTON_OK;
      }
    }
    else if (autoRepeatCount == 0)
    {
      // the button hasn't triggered auto-repeat yet; execute it on release
      executeButton = heldButton;
    }

    // else: the button was in auto-repeat, so don't execute it again on release
    heldButton = BUTTON_NONE;
    autoRepeatCount = 0;
  }

  if (executeButton == BUTTON_NONE)
    return;

  lcdRedrawNeeded = true;

  switch (executeButton)
  {
  case BUTTON_RETURN:
    backKeyPress();
    break;

  case BUTTON_UP:
  case BUTTON_DOWN:
    updownKeyPress(executeButton == BUTTON_UP);
    break;

  case BUTTON_OK:
    okKeyPress();
    break;
  }
}

static void completeAutoTune()
{
  // We're done, set the tuning parameters
  PGain = makeDecimal<3>(aTune.GetKp());
  IGain = makeDecimal<3>(aTune.GetKi());
  DGain = makeDecimal<3>(aTune.GetKd());

  // set the PID controller to accept the new gain settings
  // use whatever direction of control is currently set
  //myPID.SetControllerDirection(PID::DIRECT);
  myPID.SetMode(PID::AUTOMATIC);

  if (PGain < (ospDecimalValue<3>){0})
  {
    // the auto-tuner found a negative gain sign: convert the coefficients
    // to positive and change the direction of controller action
    PGain = -PGain;
    IGain = -IGain;
    DGain = -DGain;
    if (ctrlDirection == PID::DIRECT)
    {
      myPID.SetControllerDirection(PID::REVERSE);
      ctrlDirection = PID::REVERSE;
    }
    else
    {
      myPID.SetControllerDirection(PID::DIRECT);
      ctrlDirection = PID::DIRECT;
    }      
  }

  myPID.SetTunings(PGain, IGain, DGain);

  // this will restore the user-requested PID controller mode
  stopAutoTune();

  markSettingsDirty();
}

bool settingsWritebackNeeded;
unsigned long settingsWritebackTime;

// record that the settings have changed, and need to be written to EEPROM
// as soon as they are done changing
static void markSettingsDirty()
{
  settingsWritebackNeeded = true;

  // wait until nothing has changed for 5s before writing to EEPROM
  // this reduces EEPROM wear by not writing every time a digit is changed
  settingsWritebackTime = now + 5000;
}

// This is the Arduino main loop.
//
// There are two goals this loop must balance: the highest priority is
// that the PID loop be executed reliably and on-time; the other goal is that
// the screen, buttons, and serial interfaces all be responsive. However,
// since things like redrawing the LCD may take tens of milliseconds -- and responding
// to serial commands can take 100s of milliseconds at low bit rates -- a certain
// measure of cleverness is required.
//
// Alongside the real-time task of the PID loop, there are 6 other tasks which may
// need to be performed:
// 1. handling a button press
// 2. executing a step of the auto-tuner
// 3. executing a step of a profile
// 4. redrawing the LCD
// 5. saving settings to EEPROM
// 6. processing a serial-port command
//
// Characters from the serial port are received asynchronously: it is only the
// command _processing_ which needs to be scheduled.

// whether loop() is permitted to do LCD, EEPROM, or serial I/O: this is set
// to false when loop() is being re-entered during some slow operation
bool blockSlowOperations;

void realtimeLoop()
{
  if (controllerIsBooting)
    return;

  blockSlowOperations = true;
  loop();
  blockSlowOperations = false;
}

#if !defined STANDALONE_CONTROLLER
// we accumulate characters for a single serial command in this buffer
char serialCommandBuffer[33];
byte serialCommandLength;
#endif

void loop()
{
  // first up is the realtime part of the loop, which is not allowed to perform
  // EEPROM writes or serial I/O
  updateTimer();

  // highest priority task is to update the SSR output
  theOutputDevice.setOutputPercent(output);

  // read input, if it is ready
  if (theInputDevice.getInitializationStatus() && after(readInputTime))
  {
    input = theInputDevice.readInput();
    if (!isnan(input))
    {
      lastGoodInput = input;
      displayInput = makeDecimal<1>(input);
    }
    else
    {
      displayInput = (ospDecimalValue<1>){-19999}; // display Err
    }
    readInputTime += theInputDevice.requestInput();
  }

  if (tuning)
  {
    byte val = aTune.Runtime();

    if (val != 0)
    {
      tuning = false;
      completeAutoTune();
    }
  }
  else
  {
    // step the profile, if there is one running
    // this may call ospSettingsHelper::eepromClearBits(), but not
    // ospSettingsHelper::eepromWrite()
    if (runningProfile)
    {
      profileLoopIteration();
      
      // update displayed set point
      displaySetpoint = makeDecimal<1>(activeSetPoint);
    }

    // update the PID
    myPID.Compute();  
  }  
  // update the displayed output
  // unless in manual mode, in which case a new value may have been entered
  if (tuning || (modeIndex != PID::MANUAL))
    manualOutput = makeDecimal<1>(output);

  // after the PID has updated, check the trip limits
  if (tripLimitsEnabled)
  {
    if (tripAutoReset)
    {
      tripped = false; 
    }

    if ((displayInput != (ospDecimalValue<1>){-19999}) && ((displayInput < lowerTripLimit) || (upperTripLimit < displayInput ) || tripped))
    {
      output = 0.0;
      manualOutput = (ospDecimalValue<1>){0};
      tripped = true;
#if !defined SILENCE_BUZZER  
      if (buzz >= BUZZ_OFF)
        buzzUntilCancel; // could get pretty annoying
#endif      
    }

#if !defined SILENCE_BUZZER      
    if (!tripped)
      buzzOff;
#endif      
  }   
  else
  {
    tripped= false;
#if !defined SILENCE_BUZZER 
    buzzOff;
#endif    
  }  

  // after the realtime part comes the slow operations, which may re-enter
  // the realtime part of the loop but not the slow part
  if (blockSlowOperations)
    return;

  // update the time after each major operation;
  updateTimer();

  // we want to monitor the buttons as often as possible
  checkButtons();
  
  // we try to keep an LCD frame rate of 4 Hz, plus refreshing as soon as
  // a button is pressed
  updateTimer();
  if (after(lcdTime) || lcdRedrawNeeded)
  {
    drawMenu();
    lcdRedrawNeeded = false;
    lcdTime += 250;
  }

  // can't do much without input, so initializing input is next in line 
  if (!theInputDevice.getInitializationStatus())
  {
    input = NAN;
    displayInput = (ospDecimalValue<1>){-19999}; // Display Err
    theInputDevice.initialize();
  }     

  updateTimer();
  if (settingsWritebackNeeded && after(settingsWritebackTime))
  {
    // clear settingsWritebackNeeded first, so that it gets re-armed if the
    // realtime loop calls markSettingsDirty()
    settingsWritebackNeeded = false;

    // display a '$' instead of the cursor to show that we're saving to EEPROM
    drawNotificationCursor('$');
    saveEEPROMSettings();
  }

#if !defined STANDALONE_CONTROLLER
  // accept any pending characters from the serial buffer
  byte avail = Serial.available();
  while (avail--)
  {
    char ch = Serial.read();  
    
    if (
      (serialCommandLength == 0) &&
      ((ch == '\n') || (ch == '\r') || (ch == '\0'))
    )
      continue;
      
    if (serialCommandLength < 32)
    {
      // throw away excess characters
      serialCommandBuffer[serialCommandLength++] = ch;
    }

    if ((ch == '\n') || (ch == '\r'))
    {
      // a complete command has been received
      serialCommandBuffer[serialCommandLength - 1] = '\n';
      drawNotificationCursor('*');

/*
Serial.print(F("Arduino hears:"));
Serial.write('"');
Serial.print(serialCommandBuffer);
Serial.write('"');
Serial.println();
*/

      processSerialCommand();
      
      serialCommandLength = 0;
    }
  }
#endif
}


