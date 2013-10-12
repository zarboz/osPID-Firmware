/* This file contains the routines implementing serial-port (i.e. USB) communications
   between the controller and a command computer */

#include <math.h>
#include <avr/pgmspace.h>
#include "ospConfig.h"
#include "ospAssert.h"
#include "ospProfile.h"

#undef BUGCHECK
#define BUGCHECK() ospBugCheck(PSTR("COMM"), __LINE__);

/* The serial command format for the controller is meant to be human-readable.
It consists of a one-letter command mnemonic, followed by one or more parameter values
separated by spaces (' '). The command is terminated by a newline '\n'; an optional
carriage-return '\r' before the newline is ignored.

Each command may return some data, and WILL return a response code on a separate
line. (Command responses use '\n' as the newline separator.)

Notation: #Number is a simple floating point number of the form -?NNN(.NNN) where
NNN is any number of decimal digits. #N1-N1 is a small integer of in the range N1
to N2, inclusive. #Integer is an integer. #String is a character string, defined
as all characters received up to the '\n' or '\r\n' which terminates the command.
There is no special quoting: embedded spaces are included in the string.

Commands marked with a '?' also have a query form, consisting of the mnemonic,
followed by a question mark '?' and a newline. These commands return the value
or values in the same order as they would be written in the "set" version of the command.
Multiple values are returned one to a line.

Command list:
  A? #0-1 -- query / set Auto-tune on / off: 0 = off, 1 = on

  a? #Number #Number #Integer -- query / set Autotune parameters: step, noise, and lookback

  B? #Number -- query / set input device temperature caliBration value
  
  C -- cancel profile execution

  c? #Integer -- set the Comm speed, in kbps

  d? #Number -- set D gain

  E #String -- Execute the profile of the given name
    FIXME can get rid of this command if we are short of space

  e #0-2 -- Execute the profile of the given number

  I? #Number -- set Input value

  i? #Number -- set I gain

  J? #0-3 -- Select setpoint -- changes which setpoint is active

  L? #0-1 -- enabLe or disabLe temperature limit

  l? #Number -- query / set alarm Lower temperature limit

  M? #0-1 -- set the loop to manual/automatic Mode: 0 = manual, 1 = automatic

  N? #String -- clear the profile buffer and give it a Name, or query 
     the Name of the active profile

  O? #Number -- set Output value

  o? #Integer -- set power-On behavior

  P? #Integer #Integer #Number -- add a steP to the profile buffer with the
     numbers being {type, duration, endpoint}, or query which Profile step
     being run, returning {profile step, type, duration, targetSetPoint, time} 
     where time in seconds is the time remaining until the end of the step, 
     exceptfor type STEP_WAIT_TO_CROSS where it is the time elapsed since the 
     beginning uof the step
   
  p? #Number -- set P gain
  
  Q -- Query -- returns status lines: "S {setpoint}", "I {input}", 
    "O {output}" plus "N {profile name}" "P {profile step, ... as above}" 
    if a profile is active or else "A 1" if currently auto-tuning
    and "A 0" if the auto-tuner is off

  R? #0-1 -- diRection -- set PID gain sign: 0 = direct, 1 = reverse

  r #Integer -- Reset the memory to the hardcoded defaults, if and only if the number is -999

  S? #Number -- Setpoint -- change the (currently active) setpoint

  s? #0-3 -- query or set input sensor

  T? -- query Trip state or clear a trip

  t? #0-1 -- trip auto-reseT -- enable or disable automatic recovery from trips

  u? #Number -- query / set alarm Upper temperature limit

  V #0-2 -- saVe the profile buffer to profile N

  W? -- query output Window size in seconds or set Window size

  X -- eXamine: dump the unit's settings

  x #0-2 -- eXamine profile: dump profile N

  Y -- identifY -- returns two lines: "osPid vX.YYtag" and "Unit {unitName}"
  
  
Codes removed on Arduinos with 32kB flash memory:

  K #Integer -- peeK at memory address, +ve number = SRAM, -ve number = EEPROM; returns
  the byte value in hexadecimal

  k #Integer #Integer -- poKe at memory address: the first number is the address,
  the second is the byte to write there, in decimal

Response codes:
  OK -- success
  EINV -- invalid command or value
  EMOD -- unit in wrong mode for command (e.g. an attempt to set the gains during an auto-tune)

Programming in a profile is performed as a multi-command process. First the
profile buffer is opened by
  N profNam
where profNam is a <= 15 character name for the profile. Then 0-16 profile steps
are programmed through
  P stepType durationInMilliseconds targetSetPoint
and finally the profile is saved to profile buffer #N with
  V #N
. The profile can then be executed by name or number with
  E profNam
or
  e #N
. Profiles _must_ be saved before they can be executed.
*/


// not static because called from elsewhere
void setupSerial()
{
#ifndef ATMEGA_32kB_FLASH
  ospAssert((serialSpeed >= 0) && (serialSpeed < 7));
#endif  

  Serial.end();
  unsigned int kbps = baudRate(serialSpeed);
  Serial.begin(kbps * 100);
}

static const char * parseDecimal(const char *str, long *out, byte *decimals)
{
  long value = 0;
  byte dec = 0;

  bool isNegative = false;
  bool hasFraction = false;

  if (str[0] == '-')
  {
    isNegative = true;
    str++;
  }

  while (true)
  {
    char c = *str;

    if (c == '.')
    {
      if (hasFraction)
        goto end_of_number;
      hasFraction = true;
      str++;
      continue;
    }

    if (c < '0' || c > '9')
    {
end_of_number:
      if (isNegative)
        value = -value;

      *out = value;
      *decimals = dec;
      return str;
    }

    str++;
    value = value * 10 + (c - '0');

    if (hasFraction)
      dec++;
  }
}

static int pow10(byte n)
{
  int result = 1;

  while (n--)
    result *= 10;

  return result;
}

static int coerceToDecimal(long val, byte currentDecimals, byte desiredDecimals)
{
  if (currentDecimals < desiredDecimals)
    return int(val * pow10(desiredDecimals - currentDecimals));
  else if (desiredDecimals < currentDecimals)
  {
    // need to do a rounded division
    int divisor = pow10(currentDecimals - desiredDecimals);
    int quot = val / divisor;
    int rem  = val % divisor;

    if (abs(rem) >= divisor / 2)
    {
      if (val < 0)
        quot--;
      else
        quot++;
    }
    return quot;
  }
  else
    return int(val);
}

template<int D> static __attribute__ ((noinline)) ospDecimalValue<D> makeDecimal(long val, byte currentDecimals)
{
  return (ospDecimalValue<D>) {coerceToDecimal(val, currentDecimals, D)};
}

// since the serial buffer is finite, we perform a realtime loop iteration
// between each serial writes
template<typename T> static void __attribute__((noinline)) serialPrint(T t)
{
  Serial.print(t);
  realtimeLoop();
}


template<typename T> static void __attribute__((noinline)) serialPrintln(T t)
{
  Serial.print(t);
  Serial.println();
  realtimeLoop();
}

static void __attribute__ ((noinline)) serialPrintDecimal(int val, byte decimals)
{
  char buffer[8];
  char *p = formatDecimalValue(buffer, val, decimals);
  serialPrint(p);
}

template<int D> static void __attribute__ ((noinline)) serialPrint(ospDecimalValue<D> val)
{
  serialPrintDecimal(val.rawValue(), D);
}

template<int D> static void __attribute__ ((noinline)) serialPrintln(ospDecimalValue<D> val)
{
  serialPrint(val);
  Serial.println();
}

template<int D> static void __attribute__ ((noinline)) serialPrintlnTemp(ospDecimalValue<D> val)
{
  serialPrint(val);
#ifndef UNITS_FAHRENHEIT
  serialPrintln(FdegCelsius());
#else
  serialPrintln(FdegFahrenheit());
#endif
}

static void __attribute__ ((noinline)) serialPrintlnFloatTemp(double val)
{
  serialPrint(val);
#ifndef UNITS_FAHRENHEIT
  serialPrintln(FdegCelsius());
#else
  serialPrintln(FdegFahrenheit());
#endif
}

static void __attribute__ ((noinline)) serialPrintFAutotuner()
{
  serialPrint(F("Auto-tuner "));
}

static void __attribute__ ((noinline)) serialPrintFcolon()
{
  serialPrint(F(": "));
}

static void serialPrintDeviceFloatSettings(bool inputDevice)
{
  serialPrintln(F("put device settings:"));
  byte count = (inputDevice ? theInputDevice.floatSettingsCount() : theOutputDevice.floatSettingsCount());
  const __FlashStringHelper *description;
  for (byte i = 0; i < count; i++)
  {
    serialPrint(F("  "));
    serialPrint(char(i + '0'));
    serialPrintFcolon();
    description = (inputDevice ? theInputDevice.describeFloatSetting(i) : theOutputDevice.describeFloatSetting(i));
    serialPrint(description);
    serialPrint(F(" = "));
    serialPrintln(inputDevice ? theInputDevice.readFloatSetting(i) : theOutputDevice.readFloatSetting(i));
  }
}

static bool cmdSetSerialSpeed(const int& speed)
{
  for (byte i = 0; i < (sizeof(serialSpeedTable) / sizeof(serialSpeedTable[0])); i++)
  {
    unsigned int s = baudRate(i);
    if (s == speed)
    {
      // this is a speed we can do: report success, and then reset the serial
      // interface to the new speed
      serialSpeed = i;
      serialPrintln(F("OK"));
      Serial.flush();

      // we have to report success _first_, because changing the serial speed will
      // break the connection!
      markSettingsDirty();
      setupSerial();
      return true;
    }
  }
  return false;
}

static bool cmdStartProfile(const char *name)
{
  for (byte i = 0; i < NR_PROFILES; i++)
  {
    byte ch = 0;
    const char *p = name;
    bool match = true;

    while (*p && (ch < ospProfile::NAME_LENGTH))
    {
      if (*p != getProfileNameCharAt(i, ch))
        match = false;
      p++;
      ch++;
    }

    if (match && (ch <= ospProfile::NAME_LENGTH))
    {
      // found the requested profile: start it
      activeProfileIndex = i;
      startProfile();
      return true;
    }
  }
  return false;
}

#ifndef ATMEGA_32kB_FLASH
static void cmdPeek(int address)
{
  byte val;

  if (address <= 0)
    ospSettingsHelper::eepromRead(-address, val);
  else
    val = * (byte *)address;

  serialPrint(hex(val >> 4));  
  serialPrintln(hex(val & 0xF));
}

static void cmdPoke(int address, byte val)
{
  if (address <= 0)
    ospSettingsHelper::eepromWrite(-address, val);
  else
    *(byte *)address = val;
}
#endif

static void cmdIdentify()
{
  serialPrint(F("Unit \""));
  serialPrintln(reinterpret_cast<const __FlashStringHelper *>(PcontrollerName));
  serialPrintln("\"\nVersion ");
  serialPrintln(reinterpret_cast<const __FlashStringHelper *>(Pversion));
}

static void serialPrintProfileName(byte profileIndex)
{
  serialPrint(char('"'));
  for (byte i = 0; i < ospProfile::NAME_LENGTH; i++)
  {
    char ch = getProfileNameCharAt(profileIndex, i);
    if (ch == 0)
      break;
    Serial.write(ch);
  }
  serialPrint(char('"'));
}

static void cmdQuery()
{
  serialCommandBuffer[0] = 'S';
  serialCommandBuffer[1] = '?';
  serialCommandBuffer[2] = '\n';
  serialCommandLength = 3;
  processSerialCommand();        // S?
  serialCommandBuffer[0] = 'I';
  processSerialCommand();        // I?
  serialCommandBuffer[0] = 'O';
  processSerialCommand();        // O?
  
  if (runningProfile)
  {
    serialCommandBuffer[0] = 'N';
    processSerialCommand();        // N?
    serialCommandBuffer[0] = 'P';
    processSerialCommand();        // P?
  }
  else
  {
    serialCommandBuffer[0] = 'A';
    processSerialCommand();        // A?
  }
}

static void cmdExamineSettings()
{
  unsigned int crc16;
  serialPrint(F("EEPROM checksum: "));
  ospSettingsHelper::eepromRead(0, crc16);
  serialPrintln(crc16);

  if (modeIndex == AUTOMATIC)
    serialPrintln(F("PID mode"));
  else
    serialPrintln(F("Manual mode"));

  if (ctrlDirection == DIRECT)
    serialPrint(F("Direct action"));
  else
    serialPrint(F("Reverse action"));

  // write out the setpoints, with a '*' next to the active one
  for (byte i = 0; i < 4; i++)
  {
    if (i == setPointIndex)
      serialPrint('*');
    else
      serialPrint(' ');
    serialPrint(F("SP"));
    serialPrint(char('1' + i));
    serialPrintFcolon();
    serialPrint(setPoints[i]);
#ifndef UNITS_FAHRENHEIT
    serialPrint(FdegCelsius());
#else
    serialPrint(FdegFahrenheit());
#endif
    if (i & 1 == 0)
      serialPrint('\t');
    else
      serialPrint('\n');
  }

  Serial.println();

  serialPrint(F("Comm speed (bps): "));
  serialPrint(baudRate(serialSpeed));

  serialPrint(F("Power-on: "));
  switch (powerOnBehavior)
  {
  case POWERON_DISABLE:
    serialPrintln(F("go to manual"));
    break;
  case POWERON_RESUME_PROFILE:
    serialPrint(F("continue profile or "));
    // run on into next case with no break;
  case POWERON_CONTINUE_LOOP:
    serialPrintln(F("hold last setpoint"));
    break;
  }

  Serial.println();

  // auto-tuner settings
  serialPrintFAutotuner(); serialPrint(F("step size: "));
  serialPrintln(aTuneStep);
  serialPrintFAutotuner(); serialPrint(F("noise size: "));
  serialPrintln(aTuneNoise);
  serialPrintFAutotuner(); serialPrint(F("look-back: "));
  serialPrintln(aTuneLookBack);

  Serial.println();

  // peripheral device settings
  serialPrint(F("In"));
  serialPrintDeviceFloatSettings(true);
  // same for integer settings, if any

  serialPrint(F("Out"));
  //serialPrintFCalibrationData();  
  serialPrintDeviceFloatSettings(false);
  // same for integer settings, if any
}

static void serialPrintProfileState(byte profileIndex, byte stepIndex)
{
  byte type;
  unsigned long duration;
  ospDecimalValue<1> endpoint;

  getProfileStepData(profileIndex, stepIndex, &type, &duration, &endpoint);

  if (type == ospProfile::STEP_INVALID)
    return;
  if (type & ospProfile::STEP_FLAG_BUZZER)
    serialPrint(F(" *"));
  else
    serialPrint(F("  "));   
  serialPrint(type & ospProfile::STEP_TYPE_MASK);  
  serialPrint(' ');
  serialPrint(duration); 
  serialPrint(' ');
  serialPrintln(endpoint);
  
}
  
static void cmdExamineProfile(byte profileIndex)
{
  serialPrint(reinterpret_cast<const __FlashStringHelper *>(Pprofile));
  serialPrint(char('0' + profileIndex));
  serialPrintFcolon();

  serialPrintProfileName(profileIndex);
  Serial.println();

  serialPrint(F("Checksum: "));
  serialPrintln(getProfileCrc(profileIndex));

  for (byte i = 0; i < ospProfile::NR_STEPS; i++)
  {
    serialPrintProfileState(profileIndex, i);
  }
}

static bool __attribute__ ((noinline)) trySetGain(ospDecimalValue<3> *p, long val, byte decimals)
{
  ospDecimalValue<3> gain = makeDecimal<3>(val, decimals);

  if (((ospDecimalValue<3>){32767} < gain) || (gain < (ospDecimalValue<3>){0}))
    return false;

  *p = gain;
  return true;
}

static bool __attribute__ ((noinline)) trySetTemp(ospDecimalValue<1> *p, long val, byte decimals)
{
  ospDecimalValue<1> temp = makeDecimal<1>(val, decimals);

  if (((ospDecimalValue<1>){9999} < temp) || (temp < (ospDecimalValue<1>){-9999}))
    return false;

  *p = temp;
  return true;
}

// The command line parsing is table-driven, which saves more than 1.5 kB of code
// space.

enum {
  ARGS_NONE = 0,
  ARGS_ONE_NUMBER,
  ARGS_TWO_NUMBERS,
  ARGS_THREE_NUMBERS,
  ARGS_STRING,
  ARGS_NOT_FOUND = 0x0F,
  ARGS_FLAG_PROFILE_NUMBER = 0x10, // must be 0 to NR_PROFILES-1
  ARGS_FLAG_FIRST_IS_01 = 0x20, // must be 0 or 1
  ARGS_FLAG_NONNEGATIVE = 0x40, // must be >= 0
  ARGS_FLAG_QUERYABLE = 0x80,
  ARGS_MASK = 0x0F
};

struct SerialCommandParseData 
{
  char mnemonic;
  byte args;
};

// FIXME: this table is now dense enough that it would be better to have
// two 26-byte arrays separately for the entire upper and lowercase alphabets

// this table must be sorted in ASCII order, that is A-Z then a-z
PROGMEM SerialCommandParseData commandParseData[] = 
{
  { 'A', ARGS_ONE_NUMBER | ARGS_FLAG_FIRST_IS_01 | ARGS_FLAG_QUERYABLE },
  { 'B', ARGS_ONE_NUMBER | ARGS_FLAG_QUERYABLE },
  { 'C', ARGS_NONE },
  { 'E', ARGS_STRING },
  { 'I', ARGS_ONE_NUMBER | ARGS_FLAG_NONNEGATIVE | ARGS_FLAG_QUERYABLE },
  { 'J', ARGS_ONE_NUMBER | ARGS_FLAG_NONNEGATIVE | ARGS_FLAG_QUERYABLE },
#ifndef ATMEGA_32kB_FLASH
  { 'K', ARGS_ONE_NUMBER },
#endif
  { 'L', ARGS_ONE_NUMBER | ARGS_FLAG_FIRST_IS_01 | ARGS_FLAG_QUERYABLE },
  { 'M', ARGS_ONE_NUMBER | ARGS_FLAG_FIRST_IS_01 | ARGS_FLAG_QUERYABLE },
  { 'N', ARGS_STRING | ARGS_FLAG_QUERYABLE },
  { 'O', ARGS_ONE_NUMBER | ARGS_FLAG_NONNEGATIVE | ARGS_FLAG_QUERYABLE },
  { 'P', ARGS_THREE_NUMBERS | ARGS_FLAG_QUERYABLE },
  { 'Q', ARGS_NONE },
  { 'R', ARGS_ONE_NUMBER | ARGS_FLAG_FIRST_IS_01 | ARGS_FLAG_QUERYABLE },
  { 'S', ARGS_ONE_NUMBER | ARGS_FLAG_QUERYABLE },
  { 'T', ARGS_NONE | ARGS_FLAG_QUERYABLE },
  { 'U', ARGS_STRING },
  { 'V', ARGS_ONE_NUMBER | ARGS_FLAG_PROFILE_NUMBER },
  { 'W', ARGS_ONE_NUMBER | ARGS_FLAG_NONNEGATIVE | ARGS_FLAG_QUERYABLE },
  { 'X', ARGS_NONE },
  { 'Y', ARGS_NONE },
  { 'a', ARGS_THREE_NUMBERS },
  { 'b', ARGS_THREE_NUMBERS | ARGS_FLAG_FIRST_IS_01 | ARGS_FLAG_QUERYABLE },
  { 'c', ARGS_ONE_NUMBER | ARGS_FLAG_NONNEGATIVE | ARGS_FLAG_QUERYABLE },
  { 'd', ARGS_ONE_NUMBER | ARGS_FLAG_NONNEGATIVE | ARGS_FLAG_QUERYABLE },
  { 'e', ARGS_ONE_NUMBER | ARGS_FLAG_PROFILE_NUMBER },
  { 'i', ARGS_ONE_NUMBER | ARGS_FLAG_NONNEGATIVE | ARGS_FLAG_QUERYABLE },
#ifndef ATMEGA_32kB_FLASH
  { 'k', ARGS_TWO_NUMBERS },
#endif
  { 'l', ARGS_ONE_NUMBER | ARGS_FLAG_QUERYABLE },
  { 'o', ARGS_ONE_NUMBER | ARGS_FLAG_NONNEGATIVE | ARGS_FLAG_QUERYABLE },
  { 'p', ARGS_ONE_NUMBER | ARGS_FLAG_NONNEGATIVE | ARGS_FLAG_QUERYABLE },
  { 'r', ARGS_ONE_NUMBER },
  { 's', ARGS_ONE_NUMBER | ARGS_FLAG_NONNEGATIVE | ARGS_FLAG_QUERYABLE },
  { 't', ARGS_ONE_NUMBER | ARGS_FLAG_FIRST_IS_01 | ARGS_FLAG_QUERYABLE },
  { 'u', ARGS_ONE_NUMBER | ARGS_FLAG_QUERYABLE },
  { 'x', ARGS_ONE_NUMBER | ARGS_FLAG_PROFILE_NUMBER }
};

// perform a binary search for the argument descriptor of the given mnemonic
static byte argsForMnemonic(char mnemonic)
{
  byte start = 0, end = sizeof(commandParseData) / sizeof(commandParseData[0]) - 1;

  while (true)
  {
    if (start + 1 == end)
    {
      if (pgm_read_byte_near(&(commandParseData[start].mnemonic)) == mnemonic)
        return pgm_read_byte_near(&(commandParseData[start].args));
      if (pgm_read_byte_near(&(commandParseData[end].mnemonic)) == mnemonic)
        return pgm_read_byte_near(&(commandParseData[end].args));
      return ARGS_NOT_FOUND;
    }

    byte pivot = (start + end) / 2;
    char m = pgm_read_byte_near(&(commandParseData[pivot].mnemonic));
    if (m < mnemonic)
      start = pivot;
    else if (mnemonic < m)
      end = pivot;
    else // found it!
      return pgm_read_byte_near(&(commandParseData[pivot].args));
  }
}


  
// this is the entry point to the serial command processor: it is called
// when a '\n' has been received over the serial connection, and therefore
// a full command is buffered in serialCommandBuffer
static void processSerialCommand()
{
  const char *p = &serialCommandBuffer[1], *p2;
  long i1, i2, i3;
  byte d1, d2, d3;
  byte argDescriptor;

  if (serialCommandBuffer[--serialCommandLength] != '\n')
    goto out_EINV;

  // first parse the arguments
  argDescriptor = argsForMnemonic(serialCommandBuffer[0]);
  if (argDescriptor == ARGS_NOT_FOUND)
    goto out_EINV;

  if ((argDescriptor & ARGS_FLAG_QUERYABLE) && (*p == '?'))
  {
    // this is a query
    if (p[1] != '\n' && !(p[1] == '\r' && p[2] == '\n'))
      goto out_EINV;

    switch (serialCommandBuffer[0])
    {
    case 'A':
      serialPrintln(tuning);
      break;
    case 'a':
      serialPrintln(aTuneStep);
      serialPrintln(aTuneNoise);
      serialPrintln(aTuneLookBack);
      break;
    case 'B':
      serialPrintlnTemp(theInputDevice.getCalibration());
      break;
    case 'c':
      serialPrint(baudRate(serialSpeed));
      serialPrintln("00");
      break;
    case 'd':
      serialPrintln(DGain);
      break;
    case 'I':
      serialPrintlnFloatTemp(input);
      break;
    case 'i':
      serialPrintln(IGain);
      break;
    case 'J':
      serialPrintln(setPointIndex);
      break;
    case 'l':
      serialPrintlnTemp(lowerTripLimit);
      break;
    case 'L':
      serialPrintln(tripLimitsEnabled);
      break;
    case 'M':
      serialPrintln(modeIndex);
      break;
    case 'N':
      if (!runningProfile)
        goto out_EINV;
      serialPrintProfileName(activeProfileIndex);
      Serial.println();
      break;
    case 'O':
      serialPrint(output);
      serialPrintln(F(" %"));
      break;
    case 'o':
      serialPrintln(powerOnBehavior);
      break;
    case 'P':
      if (!runningProfile)
        goto out_EINV;
      serialPrint(F("P "));
      serialPrint(currentProfileStep);
      serialPrint(' ');
      serialPrint(profileState.stepType);
      serialPrint(' ');
      serialPrint(profileState.stepDuration);
      serialPrint(' ');
      serialPrint(profileState.targetSetpoint);
      if (
        (profileState.stepType == ospProfile::STEP_RAMP_TO_SETPOINT) || 
        (profileState.stepType == ospProfile::STEP_JUMP_TO_SETPOINT) ||
        (profileState.stepType == ospProfile::STEP_SOAK_AT_VALUE)
      )
      {
        serialPrint(profileState.stepEndMillis);
      }
      else if (profileState.stepType == ospProfile::STEP_WAIT_TO_CROSS)
      {
        long elapsed = now - profileState.stepEndMillis + profileState.stepDuration;
        serialPrint(elapsed);
      }
      Serial.println();
      break;
    case 'p':
      serialPrintln(PGain);
      break;
    case 'R':
      serialPrintln(ctrlDirection);
      break;
    case 'S':
      serialPrintlnFloatTemp(activeSetPoint);
      break;
    case 's':
      serialPrintln(inputType);
      break;
    case 'T':
      serialPrintln(tripped);
      break;
    case 't':
      serialPrintln(tripAutoReset);
      break;
    case 'u':
      serialPrintlnTemp(upperTripLimit);
      break;
    case 'W':
      serialPrint(theOutputDevice.getOutputWindowSeconds());
      serialPrintln(" seconds");
      break;
    default:
      goto out_EINV;
    }
    goto out_OK_NO_ACK;
  }

#define CHECK_SPACE()                                   \
  if ((*p++) != ' ')                                    \
    goto out_EINV;                                      \
  else do { } while (0)
#define CHECK_P2()                                      \
  if (p2 == p)                                          \
    goto out_EINV;                                      \
  else do { p = p2; } while (0)
#define CHECK_CMD_END()                                 \
  if (*p != '\n' && !(*p == '\r' && *(++p) == '\n'))    \
    goto out_EINV;                                      \
  else do { } while (0)

  // not a query, so parse it against the argDescriptor
  switch (argDescriptor & ARGS_MASK)
  {
  case ARGS_NONE:
    CHECK_CMD_END();
    break;
  case ARGS_THREE_NUMBERS: // i3, i2, i1
    CHECK_SPACE();
    p2 = parseDecimal(p, &i3, &d3);
    CHECK_P2();
    // fall through
  case ARGS_TWO_NUMBERS: // i2, i1
    CHECK_SPACE();
    p2 = parseDecimal(p, &i2, &d2);
    CHECK_P2();
    // fall through
  case ARGS_ONE_NUMBER: // i1
    CHECK_SPACE();
    p2 = parseDecimal(p, &i1, &d1);
    CHECK_P2();
    CHECK_CMD_END();
    break;
  case ARGS_STRING: // p
    CHECK_SPACE();
    // remove the trailing '\n' or '\r\n' from the #String
    if (serialCommandBuffer[serialCommandLength - 1] == '\r')
      serialCommandBuffer[--serialCommandLength] = '\0';
    else
      serialCommandBuffer[serialCommandLength] = '\0';
    // p now points to the #String
    break;
  default:
#ifndef ATMEGA_32kB_FLASH
    BUGCHECK();
#else    
    ;
#endif
  }

  // perform bounds checking
  if (argDescriptor & (ARGS_FLAG_NONNEGATIVE | ARGS_FLAG_PROFILE_NUMBER | ARGS_FLAG_FIRST_IS_01))
  {
    if (i1 < 0)
      goto out_EINV;
  }

  if ((argDescriptor & ARGS_FLAG_PROFILE_NUMBER) && (i1 >= NR_PROFILES))
    goto out_EINV;

  if ((argDescriptor & ARGS_FLAG_FIRST_IS_01) && (i1 > 1))
    goto out_EINV;

#undef CHECK_CMD_END
#undef CHECK_SPACE
#undef CHECK_P2
#define BOUNDS_CHECK(f, min, max)                       \
  if (((f) < (min)) || ((max) < (f)))                   \
    goto out_EINV;                                      \
  else do { } while (0)

  // arguments successfully parsed: try to execute the command
  switch (serialCommandBuffer[0])
  {
  case 'A': // stop/start auto-tuner
    if ((tuning ^ (byte) i1) == 0) // autotuner already on/off
      goto out_OK; // no EEPROM writeback needed
    tuning = i1;
    if (tuning)
      startAutoTune();
    else
      stopAutoTune();
    break;
  case 'a': // set the auto-tune parameters
    aTuneStep = makeDecimal<1>(i3, d3);
    aTuneNoise = makeDecimal<1>(i2, d2);
    aTuneLookBack = i1;
    break;
  case 'B':
    ospDecimalValue<1> cal;
    cal = makeDecimal<1>(i1, d1);
    BOUNDS_CHECK(cal, (ospDecimalValue<1>){-999}, (ospDecimalValue<1>){999});
    theInputDevice.setCalibration(cal);
    displayCalibration = cal;
    break;
  case 'C': // cancel profile execution
    if (runningProfile)
      stopProfile();
    else
      goto out_EMOD;
    goto out_OK; // no EEPROM writeback needed
  case 'c': // set the comm speed
    if (cmdSetSerialSpeed(i1)) // since this resets the interface, just return
      return;
    goto out_EINV;
  case 'd': // set the D gain
#ifdef PI_CONTROLLER
    goto out_EMOD;
#else // PID controller    
    if (tuning)
      goto out_EMOD;
    if (!trySetGain(&DGain, i1, d1))
      goto out_EINV;
    break;
#endif
  case 'E': // execute a profile by name
    if (!cmdStartProfile(p))
      goto out_EINV;
    goto out_OK; // no EEPROM writeback needed
  case 'e': // execute a profile by number
    if (tuning || runningProfile || modeIndex != AUTOMATIC)
      goto out_EMOD;

    activeProfileIndex = i1;
    startProfile();
    goto out_OK; // no EEPROM writeback needed
  case 'I': // directly set the input command
    goto out_EMOD; // (I don't think so)
  case 'i': // set the I gain
    if (tuning)
      goto out_EMOD;
    if (!trySetGain(&IGain, i1, d1))
      goto out_EINV;
    break;
  case 'J': // change the active setpoint
    if (i1 >= 4)
      goto out_EINV;

    setPointIndex = i1;
    updateActiveSetPoint();
    break;
#ifndef ATMEGA_32kB_FLASH  
  case 'K': // memory peek
    cmdPeek(i1);
    goto out_OK; // no EEPROM writeback needed
  case 'k': // memory poke
    BOUNDS_CHECK(i1, 0, 255);

    cmdPoke(i2, i1);
    goto out_OK; // no EEPROM writeback needed
#endif    
  case 'l': // set trip lower limit
    {
      if (!trySetTemp(&lowerTripLimit, i1, d1))
        goto out_EINV;
    }
    break;
  case 'L': // set limit trip enabled
    tripLimitsEnabled = i1;
    break;
  case 'M': // set the controller mode (PID or manual)
    modeIndex = i1;
    if (modeIndex == MANUAL)
      setOutputToManualOutput();
    myPID.SetMode(i1);
    break;
  case 'N': // clear and name the profile buffer
    if (strlen(p) > ospProfile::NAME_LENGTH)
      goto out_EINV;

    profileBuffer.clear();
    memset(profileBuffer.name, 0, sizeof(profileBuffer.name));
    strcpy(profileBuffer.name, p);
    break;
  case 'O': // directly set the output command
    {
      ospDecimalValue<1> o = makeDecimal<1>(i1, d1);
      if ((ospDecimalValue<1>){1000} < o)
        goto out_EINV;

      if (tuning || runningProfile || modeIndex != MANUAL)
        goto out_EMOD;

      manualOutput = o;
      output = double(manualOutput);
    }
    break;
  case 'o': // set power-on behavior
    if (i1 > 2)
      goto out_EINV;
    powerOnBehavior = i1;
    break;
  case 'P': // define a profile step
    if (!profileBuffer.addStep(i3, i2, makeDecimal<1>(i1, d1)))
      goto out_EINV;
    break;
  case 'p': // set the P gain
    if (tuning)
      goto out_EMOD;
    if (!trySetGain(&PGain, i1, d1))
      goto out_EINV;
    break;
  case 'Q': // query current status
    cmdQuery();
    goto out_OK; // no EEPROM writeback needed
  case 'R': // set the controller action direction
    ctrlDirection = i1;
    myPID.SetControllerDirection(i1);
    break;
  case 'r': // reset memory
    if (i1 != -999)
      goto out_EINV;

    clearEEPROM();
    serialPrintln(F("Memory marked for reset."));
    serialPrintln(F("Reset the unit to complete."));
    goto out_OK; // no EEPROM writeback needed or wanted!
  case 'S': // change the setpoint
    {
      if (tuning)
        goto out_EMOD;
      if (!trySetTemp(&setPoints[setPointIndex], i1, d1))      
        goto out_EINV;
      updateActiveSetPoint();
    }
    break;
  case 's': // set the inputType
    BOUNDS_CHECK(i1, 0, 2);
    inputType = i1;
    break;
  case 'T': // clear a trip
    if (!tripped)
      goto out_EMOD;
    tripped = false;
#ifndef SILENCE_BUZZER    
    buzzOff;
#endif    
    goto out_OK; // no EEPROM writeback needed
  case 't': // set trip auto-reset
    tripAutoReset = i1;
    break;
  case 'u': // set trip upper limit
    {
      if (!trySetTemp(&lowerTripLimit, i1, d1))
        goto out_EINV;
    }
    break;
  case 'V': // save the profile buffer to EEPROM
    saveEEPROMProfile(i1);
    goto out_OK; // no EEPROM writeback needed
  case 'W': // set the output window size in seconds
    ospDecimalValue<1> window;
    window = makeDecimal<1>(i1, d1);
    BOUNDS_CHECK(window, (ospDecimalValue<1>){10}, (ospDecimalValue<1>){9999});
    theOutputDevice.setOutputWindowSeconds(window);
    displayWindow = window;
    break;
  case 'X': // examine: dump the controller settings
    cmdExamineSettings();
    goto out_OK; // no EEPROM writeback needed
  case 'x': // examine a profile: dump a description of the give profile
    cmdExamineProfile(i1);
    goto out_OK; // no EEPROM writeback needed
  case 'Y': // identify
    cmdIdentify();
    goto out_OK; // no EEPROM writeback needed
  default:
    goto out_EINV;
  }

#undef BOUNDS_CHECK

  // we wrote a setting of some sort: schedule an EEPROM writeback
  markSettingsDirty();
out_OK:
  serialPrint(F("ACK::"));
  // acknowledge command by printing it back out
  serialPrintln(serialCommandBuffer);
  return;
  
out_OK_NO_ACK:  
  serialPrintln(F("::OK"));
  return;

out_EINV:
  serialPrintln(F("::EINV"));
  return;

out_EMOD:
  serialPrintln(F("::EMOD"));
  return;
}

