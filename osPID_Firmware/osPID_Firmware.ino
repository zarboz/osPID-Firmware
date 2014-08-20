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
ospDecimalValue<3> PGain = { 1600  }, IGain = { 60 }, DGain = { 20 };

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

#if !defined (SILENCE_BUZZER)
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
  
  // set up timer2 for buzzer interrupt
#if !defined (SILENCE_BUZZER)
  cli();                   // disable interrupts
  #if defined (__AVR_ATmega328P__)
  OCR2A = 249;             // set up timer2 CTC interrupts for buzzer
  TCCR2A |= (1 << WGM21);  // CTC Mode
  TIMSK2 |= (1 << OCIE2A); // set interrupt on compare match
  GTCCR  |= (1 << PSRASY); // reset timer2 prescaler
  TCCR2B |= (7 << CS20);   // prescaler 1024
  #elif defined (__AVR_ATmega32U4__)
  OCR3AH = 0;              // set up timer3 CTC interrupts for buzzer
  OCR3AL = 249;            // set up timer3 CTC interrupts for buzzer
  TCCR3A |= (1 << WGM32);  // CTC Mode
  TIMSK3 |= (1 << OCIE3A); // set interrupt on compare match
  GTCCR  |= (1 << PSRASY); // reset timer3 prescaler
  TCCR3B |= (5 << CS30);   // prescaler 1024
  #endif
  sei();                   // enable interrupts
#endif  

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
  
/*
License for IntelliLCD
 License Summary

License does not expire.
Can be used on unlimited sites, servers
Source-code or binary products cannot be resold or distributed
Commercial use allowed under the following conditions :
Only To be used by Dabper ENAILS aside from the OSPID code this is original work and is to be treated as such
Cannot modify source-code for any purpose (cannot create derivative works)
Attribution to software creator must be made as specified:
mugenman1111@gmail.com
 For more information on the license summary topics
 Terms and conditions

Preamble: This Agreement, signed on Aug 19, 2014 [hereinafter: Effective Date] governs the relationship between Bryan Smith, a Business Entity, (hereinafter: Licensee) and DabPer enail , a Partnership under the laws of CO, United States whose principal place of business is CO, United States (Hereinafter: Licensor). This Agreement sets the terms, rights, restrictions and obligations on using [IntelliLCD] (hereinafter: The Software) created and owned by Licensor, as detailed herein
License Grant: Licensor hereby grants Licensee a Personal, Non-assignable & non-transferable, Commercial with terms, Without the rights to create derivative works, Non-exclusive license, all with accordance with the terms set forth and other legal restrictions set forth in 3rd party software used while running Software.
Limited: Licensee may use Software for the purpose of:
Running Software on Licensee’s Website[s] and Server[s];
Allowing 3rd Parties to run Software on Licensee’s Website[s] and Server[s];
Publishing Software’s output to Licensee and 3rd Parties;
Distribute verbatim copies of Software’s output (including compiled binaries);
Modify Software to suit Licensee’s needs and specifications.
Binary Restricted: Licensee may sublicense Software as a part of a larger work containing more than Software, distributed solely in Object or Binary form under a personal, non-sublicensable, limited license. Such redistribution shall be limited to unlimited codebases.
Non Assignable & Non-Transferable: Licensee may not assign or transfer his rights and duties under this license.
Commercial use allowed with restrictions: Only To be used by Dabper ENAILS aside from the OSPID code this is original work and is to be treated as such
With Attribution Requirements﻿: mugenman1111@gmail.com
Term & Termination: The Term of this license shall be until terminated. Licensor may terminate this Agreement, including Licensee’s license in the case where Licensee :
became insolvent or otherwise entered into any liquidation process; or
exported The Software to any jurisdiction where licensor may not enforce his rights under this agreements in; or
Licensee was in breach of any of this license's terms and conditions and such breach was not cured, immediately upon notification; or
Licensee in breach of any of the terms of clause 2 to this license; or
Licensee otherwise entered into any arrangement which caused Licensor to be unable to enforce his rights under this License.
Payment: In consideration of the License granted under clause 2, Licensee shall pay Licensor a fee, via Credit-Card, PayPal or any other mean which Licensor may deem adequate. Failure to perform payment shall construe as material breach of this Agreement.
Upgrades, Updates and Fixes: Licensor may provide Licensee, from time to time, with Upgrades, Updates or Fixes, as detailed herein and according to his sole discretion. Licensee hereby warrants to keep The Software up-to-date and install all relevant updates and fixes, and may, at his sole discretion, purchase upgrades, according to the rates set by Licensor. Licensor shall provide any update or Fix free of charge; however, nothing in this Agreement shall require Licensor to provide Updates or Fixes.
Upgrades: for the purpose of this license, an Upgrade shall be a material amendment in The Software, which contains new features and or major performance improvements and shall be marked as a new version number. For example, should Licensee purchase The Software under version 1.X.X, an upgrade shall commence under number 2.0.0.
Updates: for the purpose of this license, an update shall be a minor amendment in The Software, which may contain new features or minor improvements and shall be marked as a new sub-version number. For example, should Licensee purchase The Software under version 1.1.X, an upgrade shall commence under number 1.2.0.
Fix: for the purpose of this license, a fix shall be a minor amendment in The Software, intended to remove bugs or alter minor features which impair the The Software's functionality. A fix shall be marked as a new sub-sub-version number. For example, should Licensee purchase Software under version 1.1.1, an upgrade shall commence under number 1.1.2.
Support: Software is provided under an AS-IS basis and without any support, updates or maintenance. Nothing in this Agreement shall require Licensor to provide Licensee with support or fixes to any bug, failure, mis-performance or other defect in The Software.
Bug Notification: Licensee may provide Licensor of details regarding any bug, defect or failure in The Software promptly and with no delay from such event; Licensee shall comply with Licensor's request for information regarding bugs, defects or failures and furnish him with information, screenshots and try to reproduce such bugs, defects or failures.
Feature Request: Licensee may request additional features in Software, provided, however, that (i) Licensee shall waive any claim or right in such feature should feature be developed by Licensor; (ii) Licensee shall be prohibited from developing the feature, or disclose such feature request, or feature, to any 3rd party directly competing with Licensor or any 3rd party which may be, following the development of such feature, in direct competition with Licensor; (iii) Licensee warrants that feature does not infringe any 3rd party patent, trademark, trade-secret or any other intellectual property right; and (iv) Licensee developed, envisioned or created the feature solely by himself.
Liability:  To the extent permitted under Law, The Software is provided under an AS-IS basis. Licensor shall never, and without any limit, be liable for any damage, cost, expense or any other payment incurred by Licensee as a result of Software’s actions, failure, bugs and/or any other interaction between The Software  and Licensee’s end-equipment, computers, other software or any 3rd party, end-equipment, computer or services.  Moreover, Licensor shall never be liable for any defect in source code written by Licensee when relying on The Software or using The Software’s source code.
Warranty:  
Intellectual Property: Licensor hereby warrants that The Software does not violate or infringe any 3rd party claims in regards to intellectual property, patents and/or trademarks and that to the best of its knowledge no legal action has been taken against it for any infringement or violation of any 3rd party intellectual property rights.
No-Warranty: The Software is provided without any warranty; Licensor hereby disclaims any warranty that The Software shall be error free, without defects or code which may cause damage to Licensee’s computers or to Licensee, and that Software shall be functional. Licensee shall be solely liable to any damage, defect or loss incurred as a result of operating software and undertake the risks contained in running The Software on License’s Server[s] and Website[s].
Prior Inspection: Licensee hereby states that he inspected The Software thoroughly and found it satisfactory and adequate to his needs, that it does not interfere with his regular operation and that it does meet the standards and scope of his computer systems and architecture. Licensee found that The Software interacts with his development, website and server environment and that it does not infringe any of End User License Agreement of any software Licensee may use in performing his services. Licensee hereby waives any claims regarding The Software's incompatibility, performance, results and features, and warrants that he inspected the The Software.
No Refunds: Licensee warrants that he inspected The Software according to clause 7(c) and that it is adequate to his needs. Accordingly, as The Software is intangible goods, Licensee shall not be, ever, entitled to any refund, rebate, compensation or restitution for any reason whatsoever, even if The Software contains material flaws.
Indemnification: Licensee hereby warrants to hold Licensor harmless and indemnify Licensor for any lawsuit brought against it in regards to Licensee’s use of The Software in means that violate, breach or otherwise circumvent this license, Licensor's intellectual property rights or Licensor's title in The Software. Licensor shall promptly notify Licensee in case of such legal action and request Licensee’s consent prior to any settlement in relation to such lawsuit or claim.
Governing Law, Jurisdiction: Licensee hereby agrees not to initiate class-action lawsuits against Licensor in relation to this license and to compensate Licensor for any legal fees, cost or attorney fees should any claim brought by Licensee against Licensor be denied, in part or in full.
*/
  
//IntelliLCD coding. By Bryan Smith <Mugenman1111@gmail.com> 
//This code allows the LCD to change RGB shift colors based on set value
//IE If probe reads within +-15 degrees of set value LCD will be green
//If probe reads -35 of set value LCD will fade to blue
//if probe reads +35 of set valuye LCD will fade to red
//Evenetually will update code to have menu selection for colors rather than a set variable color

int trip1 = (displaySetpoint - 15);
int trip2 = (displaySetpoint + 15);
int trip3 = (displaySetpoint - 35);
int trip4 = (displaySetpoint + 35);
  
if (displayInput > trip4) { 
      setBacklight(255, 0, 0);
        delay(5);
	}

else if (displayInput < trip3) {
      setBacklight(0, 255, 0);
        delay(5);;	   
}
else if (displayInput == trip1) {
    setBacklight(0, 0, 255);
}
else if (displayInput == trip2 || trip1) {
     setBacklight(0,255,0);
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
      
#if !defined (SILENCE_BUZZER)  
      if (buzz >= BUZZ_OFF)
      {
        buzzUntilCancel; // could get pretty annoying
      }
#endif      

    }

#if !defined (SILENCE_BUZZER)      
    if (!tripped)
    {
      buzzOff;
    }
#endif      
  }   
  else
  {
    tripped = false;
    
#if !defined (SILENCE_BUZZER) 
    buzzOff;
#endif    

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


