#include <Arduino.h>
#include <LiquidCrystal.h>
#include <Wire.h>
#include <avr/pgmspace.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "ospConfig.h"
#include "ospDecimalValue.h"
#include "osPID_Engine.h"
#include "ospAnalogButton.h"
#include "ospProfile.h"

#undef BUGCHECK
#define BUGCHECK() ospBugCheck(PSTR("MAIN"), __LINE__);

#if !defined USE_SIMULATOR
#include "ospOutputDeviceSsr.h"
#include "ospInputDevice.h"
ospInputDevice theInputDevice;
ospOutputDeviceSsr theOutputDevice;
#else
#include "ospSimulator.h"
ospSimulator theInputDevice;
#define theOutputDevice theInputDevice
#endif

#define MINIMUM(a,b) (((a)<(b))?(a):(b))
#define MAXIMUM(a,b) (((b)>(a))?(a):(b))

extern void __attribute__ ((noinline)) LCDprintln(const char* s);
extern void drawStartupBanner();
extern void setupEEPROM();
extern void setupSerial();
extern bool profileWasInterrupted();
extern void drawResumeProfileBanner();
extern bool startCurrentProfileStep();
extern void recordProfileCompletion();
extern bool okKeyLongPress();
extern void backKeyPress();
extern void updownKeyPress(bool);
extern void okKeyPress();
extern void markSettingsDirty();
extern void profileLoopIteration();
extern void drawMenu();
extern void saveEEPROMSettings();
extern void processSerialCommand();

enum 
{
  POWERON_DISABLE = 0,
  POWERON_CONTINUE_LOOP,
  POWERON_RESUME_PROFILE
};


LiquidCrystal lcd(lcdRsPin, lcdEnablePin, lcdD0Pin, lcdD1Pin, lcdD2Pin, lcdD3Pin);
ospAnalogButton<buttonsPin, 100, 253, 454, 657> theButtonReader;
ospProfile profileBuffer;

byte activeProfileIndex;
byte currentProfileStep;
boolean runningProfile = false;
ospDecimalValue<3> PGain = { 1600 }, IGain = { 200 }, DGain = { 0 };

#if !defined (UNITS_FAHRENHEIT)
ospDecimalValue<1> setPoints[4] = { { 250 }, { 650 }, { 1000 }, { 1250 } };
#else
ospDecimalValue<1> setPoints[4] = { { 800 }, { 1500 }, { 2120 }, { 2600 } };
#endif

byte setPointIndex = 0;

double activeSetPoint;

double input = NAN; 

double lastGoodInput = 25.0;

double output = 0.0;   

ospDecimalValue<1> manualOutput = { 0 };

//Temporarily fill LCD with preset data these numbers are pointless

ospDecimalValue<1> displaySetpoint    = { 710 };
ospDecimalValue<1> displayInput       = { -19999 }; // NaN
ospDecimalValue<1> displayCalibration = { 0 }; 
ospDecimalValue<1> displayWindow      = { 10 }; 

#if !defined (UNITS_FAHRENHEIT)
ospDecimalValue<1> lowerTripLimit =   { 0 } , upperTripLimit = { 1250 };
#else
ospDecimalValue<1> lowerTripLimit = { 320 } , upperTripLimit = { 2600 };
#endif

bool tripLimitsEnabled;
bool tripped;
bool tripAutoReset;

byte powerOnBehavior = DEFAULT_POWER_ON_BEHAVIOR;

bool controllerIsBooting = true;

PID myPID(&lastGoodInput, &output, &activeSetPoint, PGain, IGain, DGain, PID::DIRECT);

byte aTuneMethod = PID::AUTOTUNE_DEFAULT_METHOD; 
ospDecimalValue<1> aTuneStep  = (ospDecimalValue<1>){PID::AUTOTUNE_DEFAULT_OUTPUT_STEP};
int aTuneLookBack             = PID::AUTOTUNE_DEFAULT_LOOKBACK_SEC;

#if !defined (UNITS_FAHRENHEIT)
ospDecimalValue<3> aTuneNoise = makeDecimal<3>(PID::AUTOTUNE_DEFAULT_NOISE_BAND_CELSIUS);
#else
ospDecimalValue<3> aTuneNoise = makeDecimal<3>(PID::AUTOTUNE_DEFAULT_NOISE_BAND_CELSIUS * 1.8);
#endif 

unsigned long now, lcdTime, readInputTime;

extern void drawNotificationCursor(char icon);

#if !defined (UNITS_FAHRENHEIT)
const __FlashStringHelper *FdegCelsius() { return F(" °C"); }
#else
const __FlashStringHelper *FdegFahrenheit() { return F(" °F"); }
#endif

PROGMEM const char Pprofile[] = "Profile ";

char hex(byte b)
{
  return ((b < 10) ? (char) ('0' + b) : (char) ('A' - 10 + b));
}

void __attribute__ ((noinline)) updateTimer()
{
  now = millis();
}

bool __attribute__ ((noinline)) after(unsigned long targetTime)
{
  unsigned long u = (targetTime - now);
  return ((u & 0x80000000) > 0);
}

void __attribute__((noinline)) setOutputToManualOutput()
{
  output = double(manualOutput);
}

void __attribute__((noinline)) updateActiveSetPoint()
{
  displaySetpoint = setPoints[setPointIndex];
  activeSetPoint = double(displaySetpoint);
}

void setup()
{
  for (byte pin = 8; pin < 13; pin++)
  {
    pinMode(pin, INPUT_PULLUP);
  }
  pinMode(buzzerPin, OUTPUT);
  
  // set up the LCD,show controller name
  lcd.begin(16, 2);
  drawStartupBanner();
  
  //Color swirl on bootup
  pinMode(lcdREDPin, OUTPUT);
  pinMode(lcdGRNPin, OUTPUT);
  pinMode(lcdBLUPin, OUTPUT);
  
  brightness = 200;
  
//Code to correct the red color from overpowering the other RGB colors  
  //start red to green
  for (int i = 0; i < 255; i++) {
    setBacklight(i, 0, 255-i);
    delay(2);
  }
  //fade red to green
  for (int i = 0; i < 255; i++) {
    setBacklight(255-i, i, 0);
    delay(2);
  }
  //fade green to blue
  for (int i = 0; i < 255; i++) {
    setBacklight(0, 255-i, i);
    delay(2);
  }
  setBacklight(0,255,0);
  

  lcdTime = 25;
  updateTimer();

  // load the EEPROM settings
  setupEEPROM();
  
  // set up the peripheral devices
  theInputDevice.initialize();
  theOutputDevice.initialize();

  // set up the serial interface
#if !defined (STANDALONE_CONTROLLER)
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
  myPID.setSampleTime(PID::DEFAULT_LOOP_SAMPLE_TIME);
  myPID.setOutputLimits(0, 100);
  myPID.setTunings(PGain, IGain, DGain);

  if (powerOnBehavior == POWERON_DISABLE) 
  {
    myPID.setMode(PID::MANUAL);
    setOutputToManualOutput();
  }

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

bool settingsWritebackNeeded;
unsigned long settingsWritebackTime;

void markSettingsDirty()
{
  settingsWritebackNeeded = true;
  settingsWritebackTime = now + 5000;
}


bool blockSlowOperations;

void realtimeLoop()
{
  if (controllerIsBooting)
  {
    return;
  }

  blockSlowOperations = true;
  loop();
  blockSlowOperations = false;
}

#if !defined (STANDALONE_CONTROLLER)
char serialCommandBuffer[33];
byte serialCommandLength;
#endif

void loop()
{

  updateTimer();

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
  
int trip1 = (displaySetpoint - 15);
int trip2 = (displaySetpoint + 15);
int trip3 = (displaySetpoint - 35);
int trip4 = (displaySetpoint + 35);
int trip5 = (displayInput == 150);  
  
if (displayInput >= trip4) { 
      setBacklight(255, 0, 0);
        delay(5);
	}

else if (displayInput <= trip3) {
      setBacklight(255, 162, 0);
        delay(5);	   
}
else if (displayInput <= trip1) {
    setBacklight(225, 162, 0);
}
else if (displayInput == trip2 || trip1) {
     setBacklight(0,255,0);
     delay(5);
}
else if (displayInput <= trip5){
      setBacklight(0,0,255);
      delay(5);
}

else if (displayInput <=  trip5);{
      setBacklight(0,0,255);
      delay(5);
}

if (displayInput <= -19999);{
      setBacklight(255,0,255);
      delay(5);
}

  if (runningProfile)
  {
    profileLoopIteration();
    updateActiveSetPoint();
  }

  myPID.compute();  
  
  if (myPID.isTuning || (myPID.getMode() != PID::MANUAL))
  {
    manualOutput = makeDecimal<1>(output);
  }

  if (tripLimitsEnabled)
  {
    if (tripAutoReset)
    {
      tripped = false; 
    }

    if (
      (displayInput != (ospDecimalValue<1>){-19999}) && 
      ((displayInput < lowerTripLimit) || (upperTripLimit < displayInput ) || tripped)
    )
    {
      //This is a test right here altering output from 0.0 -> 10.0
      output = 10.0;
      manualOutput = (ospDecimalValue<1>){0};
      tripped = true;
          

    }
    
  }   
  else
  {
    tripped = false;  

  }  

  if (blockSlowOperations)
    return;

  updateTimer();

  checkButtons();
  
  updateTimer();
  if (after(lcdTime) || lcdRedrawNeeded)
  {
    drawMenu();
    lcdRedrawNeeded = false;
    lcdTime += 250;
  }

  if (!theInputDevice.getInitializationStatus())
  {
    input = NAN;
    displayInput = (ospDecimalValue<1>){-19999}; 
    theInputDevice.initialize();
  }     

  updateTimer();
  if (settingsWritebackNeeded && after(settingsWritebackTime))
  {
    settingsWritebackNeeded = false;

    drawNotificationCursor('$');
    saveEEPROMSettings();
  }

#if !defined (STANDALONE_CONTROLLER)
  
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
      serialCommandBuffer[serialCommandLength++] = ch;
    }

    if ((ch == '\n') || (ch == '\r'))
    {
      serialCommandBuffer[serialCommandLength - 1] = '\n';
      drawNotificationCursor('*');

      processSerialCommand();
      
      serialCommandLength = 0;
    }
  }
#endif

}


