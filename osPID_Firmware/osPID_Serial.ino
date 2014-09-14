/* This file contains the routines implementing serial-port (i.e. USB) communications
   between the controller and a command computer */
   
#if !defined (STANDALONE_CONTROLLER)

#include <math.h>
#include <avr/pgmspace.h>
#include "ospConfig.h"
#include "ospAssert.h"
#include "ospProfile.h"

#undef BUGCHECK
#define BUGCHECK() ospBugCheck(PSTR("COMM"), __LINE__);

// prototype definition
extern int pow10(byte);

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
  
  C -- Cancel profile execution

  c? -- query the Comm speed, in kbps; it is fixed so can't set it

  D? #0-1 -- Direction -- set PID gain sign: 0 = direct, 1 = reverse

  d? #Number -- set D gain

  I? #Number -- set Input value

  i? #Number -- set I gain

  J? #0-3 -- Select setpoint -- changes which setpoint is active

  L? #0-1 -- enabLe or disabLe temperature limit

  l? #Number -- query / set alarm Lower temperature limit

  M? #0-1 -- set the loop to manual/automatic Mode: 0 = manual, 1 = automatic

  m? #Integer -- select or query auto tune Method

  N? #String -- clear the profile buffer and give it a Name, or query 
     the Names of the uploaded profiles

  O? #Number -- set Output value

  o? #Integer -- set power-On behavior

  P? #Integer #Integer #Number -- add a steP to the profile buffer with the
     numbers being {type, duration, endpoint}, or query which Profile step
     being run, returning {profile step, type, targetSetPoint, duration, time} 
     where time since the beginning uof the step
   
  p? #Number -- set P gain
  
  Q -- Query -- returns status lines: "T {alarm status}", "S {setpoint}", "I {input}", 
    "O {output}" plus "P {profile step, ... as above}" 
    if a profile is active or else "A 1" if currently auto-tuning
    and "A 0" if the auto-tuner is off

  R? #0-2 -- Run the profile of the given number, or query whether a profile is running

  r #Integer -- Reset the memory to the hardcoded defaults, if and only if the number is -999

  S? #Number -- Setpoint -- change the (currently active) setpoint

  s? #0-3 -- query or set input sensor

  T? -- query Trip state or clear a trip

  t? #0-1 -- trip auto-reseT -- enable or disable automatic recovery from trips

  u? #Number -- query / set alarm Upper temperature limit

  V #0-2 -- saVe the profile buffer to profile N

  W? -- query output Window size in seconds or set Window size

  x #0-2 -- eXamine profile: dump profile N

  Y -- identifY -- returns two lines: "osPid vX.YYtag" and "Unit {unitName}"
  
  
Codes removed on Arduinos with 32kB flash memory:

  E #String -- Execute the profile of the given name

  K #Integer -- peeK at memory address, +ve number = SRAM, -ve number = EEPROM; returns
  the byte value in hexadecimal

  k #Integer #Integer -- poKe at memory address: the first number is the address,
  the second is the byte to write there, in decimal
  
  X -- eXamine: dump the unit's settings

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

 /*******************************************************************************
 *
 *
 *                              COMMAND  FUNCTIONS
 *
 *
 *******************************************************************************/


// not static because called from elsewhere
void setupSerial()
{
  Serial.end();
  Serial.begin(baudrate);
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
      {
        goto end_of_number;
      }
      hasFraction = true;
      str++;
      continue;
    }

    if (c < '0' || c > '9')
    {
end_of_number:
      if (isNegative)
      {
        value = -value;
      }

      *out = value;
      *decimals = dec;
      return str;
    }

    str++;
    value = value * 10 + (c - '0');

    if (hasFraction)
    {
      dec++;
    }
  }
}

static int coerceToDecimal(long val, byte currentDecimals, byte desiredDecimals)
{
  if (currentDecimals < desiredDecimals)
  {
    return int(val * pow10(desiredDecimals - currentDecimals));
  }
  else if (desiredDecimals < currentDecimals)
  {
    // need to do a rounded division
    int divisor = pow10(currentDecimals - desiredDecimals);
    int quot = val / divisor;
    int rem  = val % divisor;

    if (abs(rem) >= divisor / 2)
    {
      if (val < 0)
      {
        quot--;
      }
      else
      {
        quot++;
      }
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
  
#if !defined (UNITS_FAHRENHEIT)
  serialPrintln(FdegCelsius());
#else
  serialPrintln(FdegFahrenheit());
#endif

}

static void __attribute__ ((noinline)) serialPrintlnFloatTemp(double val)
{
  serialPrint(val);
  
#if !defined (UNITS_FAHRENHEIT)
  serialPrintln(FdegCelsius());
#else
  serialPrintln(FdegFahrenheit());
#endif

}

#if !defined (ATMEGA_32kB_FLASH)
static void __attribute__ ((noinline)) serialPrintFAutotuner()
{
  serialPrint(F("Auto-tuner "));
}
#endif

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
      {
        match = false;
      }
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

#if !defined (ATMEGA_32kB_FLASH)
static void cmdPeek(int address)
{
  byte val;

  if (address <= 0)
  {
    ospSettingsHelper::eepromRead(-address, val);
  }
  else
  {
    val = * (byte *)address;
  }

  serialPrint(hex(val >> 4));  
  serialPrintln(hex(val & 0xF));
}

static void cmdPoke(int address, byte val)
{
  if (address <= 0)
  {
    ospSettingsHelper::eepromWrite(-address, val);
  }
  else
  {
    *(byte *)address = val;
  }
}
#endif

static void cmdIdentify()
{
  serialPrint(F("Unit: "));
  serialPrint(reinterpret_cast<const __FlashStringHelper *>(PcontrollerName));
  serialPrint(F(" Version: "));
  serialPrintln(reinterpret_cast<const __FlashStringHelper *>(Pversion));
}

static void serialPrintProfileName(byte profileIndex)
{
  serialPrint(char('"'));
  for (byte i = 0; i < ospProfile::NAME_LENGTH; i++)
  {
    char ch = getProfileNameCharAt(profileIndex, i);
    if (ch == 0)
    {
      break;
    }
    Serial.write(ch);
  }
  serialPrint(char('"'));
}

static void query(char cmd)
{
  serialCommandBuffer[0] = cmd;
  serialCommandBuffer[1] = '?';
  serialCommandBuffer[2] = '\n';
  serialCommandLength = 3;
  processSerialCommand();        
}

static void cmdQuery()
{
  query('T');
  query('S');
  query('I');
  query('O');
  
  if (runningProfile)
  {
    //query('N');
    query('P');      
  }
  else
  {
    query('A');
  }
  serialCommandBuffer[0] = 'Q';
  serialCommandBuffer[1] = '\0';
  serialCommandLength = 1;
}

#if !defined (ATMEGA_32kB_FLASH)
static void cmdExamineSettings()
{
  unsigned int crc16;
  serialPrint(F("EEPROM checksum: "));
  ospSettingsHelper::eepromRead(0, crc16);
  serialPrintln(crc16);
  
  Serial.println();

  if (myPID.getMode() == PID::AUTOMATIC)
  {
    serialPrintln(F("PID mode"));
  }
  else
  {
    serialPrintln(F("Manual mode"));
  }

  if (myPID.getDirection() == PID::DIRECT)
  {
    serialPrintln(F("Direct action"));
  }
  else
  {
    serialPrintln(F("Reverse action"));
  }
    
  Serial.println();

  // write out the setpoints, with a '*' next to the active one
  for (byte i = 0; i < 4; i++)
  {
    if (i == setPointIndex)
    {
      serialPrint('*');
    }
    else
    {
      serialPrint(' ');
    }
    serialPrint(F("Sv"));
    serialPrint(char('1' + i));
    serialPrintFcolon();
    serialPrint(setPoints[i]);
    
#if !defined (UNITS_FAHRENHEIT)
    serialPrint(FdegCelsius());
#else
    serialPrint(FdegFahrenheit());
#endif

    if (i & 1)
    {
      Serial.println();
    }
    else
    {
      serialPrint('\t');
    }
  }

  Serial.println();

  serialPrint(F("Comm speed (bps): "));
  serialPrintln(baudrate);

  serialPrint(F("Power-on: "));
  switch (powerOnBehavior)
  {
  case POWERON_DISABLE:
    serialPrintln(F("go to manual"));
    break;
  case POWERON_RESUME_PROFILE:
    serialPrintln(F("resume profile"));
    break;
  case POWERON_CONTINUE_LOOP:
    serialPrintln(F("hold last setpoint"));
    break;
  }

  Serial.println();

  // auto-tuner settings
  serialPrintFAutotuner(); serialPrint(F("method: "));
  serialPrintln(aTuneMethod);
  serialPrintFAutotuner(); serialPrint(F("step size: "));
  serialPrintln(aTuneStep);
  serialPrintFAutotuner(); serialPrint(F("noise size: "));
  serialPrintln(aTuneNoise);
  serialPrintFAutotuner(); serialPrint(F("look back: "));
  serialPrintln(aTuneLookBack);

  Serial.println();

  // peripheral device settings
  serialPrint(F("In"));
  serialPrintDeviceFloatSettings(true);
  // same for integer settings, if any

  Serial.println();
  
  serialPrint(F("Out"));
  //serialPrintFCalibrationData();  
  serialPrintDeviceFloatSettings(false);
  // same for integer settings, if any
}
#endif // if !defined (ATMEGA_32kB_FLASH)

static void serialPrintProfileState(byte profileIndex, byte stepIndex)
{
  byte type;
  unsigned long duration;
  ospDecimalValue<1> endpoint;

  getProfileStepData(profileIndex, stepIndex, &type, &duration, &endpoint);

  if (type == ospProfile::STEP_INVALID)
  {
    return;
  }
  if (type & ospProfile::STEP_FLAG_BUZZER)
  {
    serialPrint(F(" *"));
  }
  else
  {
    serialPrint(F("  "));   
  }
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
  
  /*
  serialPrint(F("Checksum: "));
  serialPrintln(getProfileCrc(profileIndex));
  */
  
  for (byte i = 0; i < ospProfile::NR_STEPS; i++)
  {
    serialPrintProfileState(profileIndex, i);
  }
}

static bool __attribute__ ((noinline)) trySetGain(ospDecimalValue<3> *p, long val, byte decimals)
{
  ospDecimalValue<3> gain = makeDecimal<3>(val, decimals);

  if (((ospDecimalValue<3>){32767} < gain) || (gain < (ospDecimalValue<3>){0}))
  {
    return false;
  }

  *p = gain;
  return true;
}

static bool __attribute__ ((noinline)) trySetTemp(ospDecimalValue<1> *p, long val, byte decimals)
{
  ospDecimalValue<1> temp = makeDecimal<1>(val, decimals);

  if (((ospDecimalValue<1>){9999} < temp) || (temp < (ospDecimalValue<1>){-9999}))
  {
    return false;
  }

  *p = temp;
  return true;
}

 /*******************************************************************************
 *
 *
 *                               COMMAND  PARSER
 *
 *
 *******************************************************************************/

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
PROGMEM const SerialCommandParseData commandParseData[] = 
{
  { 'A', ARGS_ONE_NUMBER | ARGS_FLAG_FIRST_IS_01 | ARGS_FLAG_QUERYABLE },
  { 'B', ARGS_ONE_NUMBER | ARGS_FLAG_QUERYABLE },
  { 'C', ARGS_NONE },
  { 'D', ARGS_ONE_NUMBER | ARGS_FLAG_FIRST_IS_01 | ARGS_FLAG_QUERYABLE },
  
#if !defined (ATMEGA_32kB_FLASH)  
  { 'E', ARGS_STRING },
#endif  

  { 'I', ARGS_ONE_NUMBER | ARGS_FLAG_NONNEGATIVE | ARGS_FLAG_QUERYABLE },
  { 'J', ARGS_ONE_NUMBER | ARGS_FLAG_NONNEGATIVE | ARGS_FLAG_QUERYABLE },
  
#if !defined (ATMEGA_32kB_FLASH)
  { 'K', ARGS_ONE_NUMBER },
#endif

  { 'L', ARGS_ONE_NUMBER | ARGS_FLAG_FIRST_IS_01 | ARGS_FLAG_QUERYABLE },
  { 'M', ARGS_ONE_NUMBER | ARGS_FLAG_FIRST_IS_01 | ARGS_FLAG_QUERYABLE },
  { 'N', ARGS_STRING | ARGS_FLAG_QUERYABLE },
  { 'O', ARGS_ONE_NUMBER | ARGS_FLAG_NONNEGATIVE | ARGS_FLAG_QUERYABLE },
  { 'P', ARGS_THREE_NUMBERS | ARGS_FLAG_QUERYABLE },
  { 'Q', ARGS_NONE },
  { 'R', ARGS_ONE_NUMBER | ARGS_FLAG_PROFILE_NUMBER | ARGS_FLAG_QUERYABLE },
  { 'S', ARGS_ONE_NUMBER | ARGS_FLAG_QUERYABLE },
  { 'T', ARGS_NONE | ARGS_FLAG_QUERYABLE },
  { 'U', ARGS_STRING },
  { 'V', ARGS_ONE_NUMBER | ARGS_FLAG_PROFILE_NUMBER },
  { 'W', ARGS_ONE_NUMBER | ARGS_FLAG_NONNEGATIVE | ARGS_FLAG_QUERYABLE },
  { 'X', ARGS_NONE },
  { 'Y', ARGS_NONE },
  { 'a', ARGS_THREE_NUMBERS | ARGS_FLAG_QUERYABLE },
  { 'b', ARGS_THREE_NUMBERS | ARGS_FLAG_FIRST_IS_01 | ARGS_FLAG_QUERYABLE },
  { 'c', ARGS_FLAG_QUERYABLE },
  { 'd', ARGS_ONE_NUMBER | ARGS_FLAG_NONNEGATIVE | ARGS_FLAG_QUERYABLE },
  { 'i', ARGS_ONE_NUMBER | ARGS_FLAG_NONNEGATIVE | ARGS_FLAG_QUERYABLE },
  
#if !defined (ATMEGA_32kB_FLASH)
  { 'k', ARGS_TWO_NUMBERS },
#endif

  { 'l', ARGS_ONE_NUMBER | ARGS_FLAG_QUERYABLE },
  { 'm', ARGS_ONE_NUMBER | ARGS_FLAG_QUERYABLE },
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
      {
        return pgm_read_byte_near(&(commandParseData[start].args));
      }
      if (pgm_read_byte_near(&(commandParseData[end].mnemonic)) == mnemonic)
      {
        return pgm_read_byte_near(&(commandParseData[end].args));
      }
      return ARGS_NOT_FOUND;
    }

    byte pivot = (start + end) / 2;
    char m = pgm_read_byte_near(&(commandParseData[pivot].mnemonic));
    if (m < mnemonic)
    {
      start = pivot;
    }
    else if (mnemonic < m)
    {
      end = pivot;
    }
    else // found it!
    {
      return pgm_read_byte_near(&(commandParseData[pivot].args));
    }
  }
}
  
// this is the entry point to the serial command processor: it is called
// when '\n' or '\r' has been received over the serial connection, and therefore
// a full command is buffered in serialCommandBuffer, terminating in '\n'
void processSerialCommand()
{
  const char *p = &serialCommandBuffer[1], *p2;
  long i1, i2, i3;
  byte d1, d2, d3;
  byte argDescriptor;

  if (serialCommandBuffer[--serialCommandLength] != '\n')
  {
    goto out_EINV;
  }

  // first parse the arguments
  argDescriptor = argsForMnemonic(serialCommandBuffer[0]);
  if (argDescriptor == ARGS_NOT_FOUND)
  {
    goto out_EINV;
  }

  if ((argDescriptor & ARGS_FLAG_QUERYABLE) && (*p == '?'))
  {  
    // this is a query
    if (p[1] != '\n')
    {
      goto out_EINV;
    }

    switch (serialCommandBuffer[0])
    {
    case 'A':
      serialPrintln(myPID.isTuning);
      break;
    case 'a':
      serialPrint(aTuneStep);
      serialPrint(' ');
      serialPrint(aTuneNoise);
      serialPrint(' ');
      serialPrintln(aTuneLookBack);
      break;
    case 'B':
      serialPrintlnTemp(theInputDevice.getCalibration());
      break;
    case 'c':
      serialPrintln(baudrate);
      break;
    case 'D':
      serialPrintln(myPID.getDirection());
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
      serialPrintln(myPID.getMode());
      break;
    case 'm':
      serialPrintln(aTuneMethod);
      break;
    case 'N':
    /*
      if (!runningProfile)
      {
        goto out_EINV;
      }
      serialPrintProfileName(activeProfileIndex);
    */
      for (byte profileIndex = 0; profileIndex < 3; profileIndex++)
      {
        serialPrintProfileName(profileIndex);
        serialPrint(' ');
      }
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
      long elapsed;
      if (!runningProfile)
      {
        goto out_EINV;
      }
      serialPrint(activeProfileIndex);
      serialPrint(' ');   
      serialPrint(currentProfileStep);
      serialPrint(' ');
      serialPrint(profileState.stepType);
      serialPrint(' ');
      serialPrint(profileState.targetSetpoint);
      serialPrint(' ');
      serialPrint(profileState.stepDuration);
      serialPrint(' ');
      elapsed = now - profileState.stepEndMillis + profileState.stepDuration;
      serialPrint(elapsed);
      Serial.println();
      break;
    case 'p':
      serialPrintln(PGain);
      break;
    case 'R':
      if (runningProfile)
        serialPrintln(activeProfileIndex);
      else      
        serialPrintln(F("-1"));
      break;
    case 'S':
      serialPrintlnFloatTemp(activeSetPoint);
      break;
    case 's':
      serialPrintln((byte)theInputDevice.ioType);
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
      serialPrintln(F(" sec"));
      break;
    default:
      goto out_EINV;
    }
    goto out_OK;
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
    {
      serialCommandBuffer[--serialCommandLength] = '\0';
    }
    else
    {
      serialCommandBuffer[serialCommandLength] = '\0';
    }
    // p now points to the #String
    break;
  default:
  
#if !defined (ATMEGA_32kB_FLASH)
    BUGCHECK();
#else    
    ;
#endif

  }

  // perform bounds checking
  if (argDescriptor & (ARGS_FLAG_NONNEGATIVE | ARGS_FLAG_PROFILE_NUMBER | ARGS_FLAG_FIRST_IS_01))
  {
    if (i1 < 0)
    {
      goto out_EINV;
    }
  }

  if ((argDescriptor & ARGS_FLAG_PROFILE_NUMBER) && (i1 >= NR_PROFILES))
  {
    goto out_EINV;
  }

  if ((argDescriptor & ARGS_FLAG_FIRST_IS_01) && (i1 > 1))
  {
    goto out_EINV;
  }

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
    if ((myPID.isTuning ^ (byte) i1) == 0) // autotuner already on/off
    {
      goto out_ACK; // no EEPROM writeback needed
    }
    if (i1 == 0)
    {
      myPID.stopAutoTune();
    }
    else
    {
      myPID.startAutoTune(aTuneMethod, aTuneStep, aTuneNoise, aTuneLookBack);
    }
    break;

  case 'a': // set the auto-tune parameters
    aTuneStep = makeDecimal<1>(i3, d3);
    aTuneNoise = makeDecimal<3>(i2, d2);
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
    {
      stopProfile();
    }
    else
    {
      goto out_EMOD;
    }
    goto out_ACK; // no EEPROM writeback needed

/*
  // no longer implemented
  case 'c': // set the comm speed
    if (cmdSetSerialSpeed(i1)) // since this resets the interface, just return
      return;
    goto out_EINV;
*/

  case 'D': // set the controller action direction
    myPID.setControllerDirection(i1);
    break;

  case 'd': // set the D gain
    if (myPID.isTuning)
    {
      goto out_EMOD;
    }
    if (!trySetGain(&DGain, i1, d1))
    {
      goto out_EINV;
    }
    break;
    
#if !defined (ATMEGA_32kB_FLASH)
  case 'E': // execute a profile by name
    if (myPID.isTuning || runningProfile)
    {
      goto out_EMOD;
    }  
    if (!cmdStartProfile(p))
    {
      goto out_EINV;
    }
    myPID.setMode(PID::AUTOMATIC);
    goto out_ACK; // no EEPROM writeback needed
#endif    

  case 'e': // execute a profile by number
    if (myPID.isTuning || runningProfile || (myPID.getMode() != PID::AUTOMATIC))
    {
      goto out_EMOD;
    }
    activeProfileIndex = i1;
    startProfile();
    goto out_ACK; // no EEPROM writeback needed

  case 'I': // directly set the input command
    goto out_EMOD; // (I don't think so)

  case 'i': // set the I gain
    if (myPID.isTuning)
    {
      goto out_EMOD;
    }
    if (!trySetGain(&IGain, i1, d1))
    {
      goto out_EINV;
    }
    break;

  case 'J': // change the active setpoint
    if (i1 >= 4)
    {
      goto out_EINV;
    }
    setPointIndex = i1;
    updateActiveSetPoint();
    break;
    
#if !defined (ATMEGA_32kB_FLASH)  
  case 'K': // memory peek
    cmdPeek(i1);
    goto out_ACK; // no EEPROM writeback needed

  case 'k': // memory poke
    BOUNDS_CHECK(i1, 0, 255);
    cmdPoke(i2, i1);
    goto out_ACK; // no EEPROM writeback needed
#endif    

  case 'l': // set trip lower limit
    {
      if (!trySetTemp(&lowerTripLimit, i1, d1))
      {
        goto out_EINV;
      }
    }
    break;

  case 'L': // set limit trip enabled
    tripLimitsEnabled = i1;
    break;

  case 'M': // set the controller mode (PID or manual)
    myPID.setMode(i1);
    if (myPID.getMode() == PID::MANUAL)
    {
      setOutputToManualOutput();
    }
    break;

  case 'm': // select auto tune method
    // turn off auto tune
    if ((i1 < 0) || (i1 > PID::LAST_AUTOTUNE_METHOD))
    {
      goto out_EINV;
    }
    if (myPID.isTuning)
    {
      myPID.stopAutoTune();
    }
    aTuneMethod = i1;
    break;

  case 'N': // clear and name the profile buffer
    if (strlen(p) > ospProfile::NAME_LENGTH)
    {
      goto out_EINV;
    }
    profileBuffer.clear();
    memset(profileBuffer.name, 0, sizeof(profileBuffer.name));
    strcpy(profileBuffer.name, p);
    break;

  case 'O': // directly set the output command
    {
      ospDecimalValue<1> o = makeDecimal<1>(i1, d1);
      if ((ospDecimalValue<1>){1000} < o)
      {
        goto out_EINV;
      }
      if (myPID.isTuning || runningProfile || (myPID.getMode() == PID::AUTOMATIC))
      {
        goto out_EMOD;
      }
      manualOutput = o;
      output = double(manualOutput);
    }
    break;

  case 'o': // set power-on behavior
    if (i1 > 2)
    {
      goto out_EINV;
    }
    powerOnBehavior = i1;
    break;

  case 'P': // define a profile step
    if (!profileBuffer.addStep(i3, i2, makeDecimal<1>(i1, d1)))
    {
      goto out_EINV;
    }
    break;

  case 'p': // set the P gain
    if (myPID.isTuning)
    {
      goto out_EMOD;
    }
    if (!trySetGain(&PGain, i1, d1))
    {
      goto out_EINV;
    }
    break;

  case 'Q': // query current status
    cmdQuery();
    goto out_ACK; // no EEPROM writeback needed

  case 'R': // run a profile by number
    if (myPID.isTuning || runningProfile)
    {
      goto out_EMOD;
    }
    myPID.setMode(PID::AUTOMATIC);
    activeProfileIndex = i1;
    startProfile();
    goto out_ACK; // no EEPROM writeback needed

  case 'r': // reset memory
    if (i1 != -999)
    {
      goto out_EINV;
    }
    clearEEPROM();
    serialPrintln(F("Memory marked for reset, now restart."));
    goto out_ACK; // no EEPROM writeback needed or wanted!

  case 'S': // change the setpoint
    {
      if (myPID.isTuning)
      {
        goto out_EMOD;
      }
      if (!trySetTemp(&displaySetpoint, i1, d1)) 
      {     
        goto out_EINV;
      }
      setPoints[setPointIndex] = displaySetpoint;
      updateActiveSetPoint();
    }
    break;

  case 's': // set the inputType
    BOUNDS_CHECK(i1, 0, 2);
    theInputDevice.ioType = (byte) i1;
    break;

  case 'T': // clear a trip
    if (!tripped)
    {
      goto out_EMOD;
    }
    tripped = false;   

    goto out_ACK; // no EEPROM writeback needed

  case 't': // set trip auto-reset
    tripAutoReset = i1;
    break;

  case 'u': // set trip upper limit
    {
      if (!trySetTemp(&upperTripLimit, i1, d1))
      {
        goto out_EINV;
      }
    }
    break;

  case 'V': // save the profile buffer to EEPROM
    saveEEPROMProfile(i1);
    goto out_ACK; // no EEPROM writeback needed

  case 'W': // set the output window size in seconds
    ospDecimalValue<1> window;
    window = makeDecimal<1>(i1, d1);
    BOUNDS_CHECK(window, (ospDecimalValue<1>){10}, (ospDecimalValue<1>){9999});
    theOutputDevice.setOutputWindowSeconds(window);
    displayWindow = window;
    break;

  case 'X': // examine: dump the controller settings
  
#if !defined (ATMEGA_32kB_FLASH)
    cmdExamineSettings();
    goto out_ACK; // no EEPROM writeback needed
#else
    goto out_EINV;
#endif

  case 'x': // examine a profile: dump a description of the give profile
    cmdExamineProfile(i1);
    goto out_ACK; // no EEPROM writeback needed

  case 'Y': // identify
    cmdIdentify();
    goto out_ACK; // no EEPROM writeback needed

  default:
    goto out_EINV;
  }

#undef BOUNDS_CHECK

  // we wrote a setting of some sort: schedule an EEPROM writeback
  markSettingsDirty();
out_ACK:
  // acknowledge command by printing it back out
  serialCommandBuffer[serialCommandLength] = '\0';
  serialPrint(serialCommandBuffer);
  serialPrintln(F("::ACK"));
  return;
  
out_OK:  
  Serial.write(char(serialCommandBuffer[0]));
  Serial.write('?');
  serialPrintln(F("::OK"));
  return;

out_EINV:
  serialCommandBuffer[serialCommandLength] = '\0';
  serialPrint(serialCommandBuffer);
  serialPrintln(F("::EINV"));
  return;

out_EMOD:
  serialCommandBuffer[serialCommandLength] = '\0';
  serialPrint(serialCommandBuffer);
  serialPrintln(F("::EMOD"));
  return;
}

#endif
