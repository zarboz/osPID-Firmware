/* This file contains the implementation of the on-screen menu system of the controller */

#include <avr/io.h>
#include <avr/pgmspace.h>
#include "ospConfig.h"
#include "ospAssert.h"
#include "ospDecimalValue.h"

#undef BUGCHECK
#define BUGCHECK() ospBugCheck(PSTR("MENU"), __LINE__);

static const byte firstDigitPosition = 6;
static const byte lastDigitPosition  = firstDigitPosition + 4;

static const byte MENU_FLAG_2x2_FORMAT = 0x01;

/*
 * This class encapsulates the PROGMEM tables which define the menu system.
 */
struct MenuItem 
{
  byte pmemItemCount;
  byte pmemFlags;
  const byte *pmemItemPtr;

  byte itemCount() const 
  {
    return pgm_read_byte_near(&pmemItemCount);
  }

  byte itemAt(byte index) const 
  {
    const byte *itemPtr = (const byte *)pgm_read_word_near(&pmemItemPtr);
    return pgm_read_byte_near(&itemPtr[index]);
  }

  bool is2x2() const 
  {
    return (pgm_read_byte_near(&pmemFlags) & MENU_FLAG_2x2_FORMAT);
  }
};


// all of the items which might be displayed on the screen
enum 
{
  // all menus must be first
  ITEM_MAIN_MENU = 0,
  ITEM_DASHBOARD_MENU,
  ITEM_TUNING_MENU,
  ITEM_CONFIG_MENU,
  ITEM_PROFILE_MENU,
  ITEM_SETPOINT_MENU,
  ITEM_TRIP_MENU,
  
#if !defined (USE_SIMULATOR)
  ITEM_INPUT_MENU,
#endif

  ITEM_POWERON_MENU,
  ITEM_RESET_ROM_MENU,

  // then decimal items
  // NOTE: these must be the first N items in the decimalItemData[] array!
  FIRST_DECIMAL_ITEM,
  ITEM_SETPOINT = FIRST_DECIMAL_ITEM,
  ITEM_INPUT,
  ITEM_OUTPUT,
  ITEM_KP,
  ITEM_KI,
  ITEM_KD,
  ITEM_CALIBRATION,
  ITEM_WINDOW_LENGTH,
  ITEM_LOWER_TRIP_LIMIT,
  ITEM_UPPER_TRIP_LIMIT,

  // then generic/specialized items
  FIRST_ACTION_ITEM,
  ITEM_AUTOTUNE_CMD = FIRST_ACTION_ITEM,
  ITEM_PROFILE1,
  ITEM_PROFILE2,
  ITEM_PROFILE3,

  ITEM_SETPOINT1,
  ITEM_SETPOINT2,
  ITEM_SETPOINT3,
  ITEM_SETPOINT4,

  ITEM_PID_MODE,
  ITEM_PID_DIRECTION,

  ITEM_INPUT_THERMISTOR,
  ITEM_INPUT_ONEWIRE,
  ITEM_INPUT_THERMOCOUPLE,

  ITEM_POWERON_DISABLE,
  ITEM_POWERON_CONTINUE,
  ITEM_POWERON_RESUME_PROFILE,

  ITEM_TRIP_ENABLED,
  ITEM_TRIP_AUTORESET,

  ITEM_RESET_ROM_NO,
  ITEM_RESET_ROM_YES,

  ITEM_COUNT,
  MENU_COUNT = FIRST_DECIMAL_ITEM,
  DECIMAL_ITEM_COUNT = FIRST_ACTION_ITEM - FIRST_DECIMAL_ITEM
};

PROGMEM const byte mainMenuItems[4] = { ITEM_DASHBOARD_MENU, ITEM_TUNING_MENU, ITEM_CONFIG_MENU, ITEM_PROFILE_MENU };
PROGMEM const byte dashMenuItems[5] = { ITEM_SETPOINT, ITEM_INPUT, ITEM_OUTPUT, ITEM_PID_MODE, ITEM_TRIP_MENU };
PROGMEM const byte tuneMenuItems[5] = { ITEM_KP, ITEM_KI, ITEM_KD, ITEM_PID_DIRECTION, ITEM_AUTOTUNE_CMD };

#if !defined (USE_SIMULATOR)
PROGMEM const byte configMenuItems[5] = { ITEM_INPUT_MENU, ITEM_CALIBRATION, ITEM_WINDOW_LENGTH, ITEM_POWERON_MENU, ITEM_RESET_ROM_MENU };
#else
PROGMEM const byte configMenuItems[2] = { ITEM_POWERON_MENU, ITEM_RESET_ROM_MENU };
#endif 
  
PROGMEM const byte profileMenuItems[3] = { ITEM_PROFILE1, ITEM_PROFILE2, ITEM_PROFILE3 };
PROGMEM const byte setpointMenuItems[4] = { ITEM_SETPOINT1, ITEM_SETPOINT2, ITEM_SETPOINT3, ITEM_SETPOINT4 };

#if !defined (USE_SIMULATOR)
PROGMEM const byte inputMenuItems[3] = { ITEM_INPUT_THERMISTOR, ITEM_INPUT_ONEWIRE, ITEM_INPUT_THERMOCOUPLE };
#else
// only one simulation currently implemented so no need for menu
#endif

PROGMEM const byte poweronMenuItems[3]  = { ITEM_POWERON_DISABLE, ITEM_POWERON_CONTINUE, ITEM_POWERON_RESUME_PROFILE };
PROGMEM const byte tripMenuItems[4]     = { ITEM_TRIP_ENABLED, ITEM_LOWER_TRIP_LIMIT, ITEM_UPPER_TRIP_LIMIT, ITEM_TRIP_AUTORESET };
PROGMEM const byte resetRomMenuItems[2] = { ITEM_RESET_ROM_NO, ITEM_RESET_ROM_YES };
PROGMEM const byte temperatureItems[4]  = { ITEM_SETPOINT, ITEM_INPUT, ITEM_LOWER_TRIP_LIMIT, ITEM_UPPER_TRIP_LIMIT };

// This must be in the same order as the ITEM_*_MENU enumeration values
PROGMEM const MenuItem menuData[MENU_COUNT + 1] =
{ 
  { sizeof(mainMenuItems),     0,                    mainMenuItems       } ,
  { sizeof(dashMenuItems),     0,                    dashMenuItems       } ,
  { sizeof(tuneMenuItems),     0,                    tuneMenuItems       } ,
  { sizeof(configMenuItems),   0,                    configMenuItems     } ,
  { sizeof(profileMenuItems),  0,                    profileMenuItems    } ,
  { sizeof(setpointMenuItems), MENU_FLAG_2x2_FORMAT, setpointMenuItems   } ,
  { sizeof(tripMenuItems),     0,                    tripMenuItems       } ,
  
#if !defined (USE_SIMULATOR)
  { sizeof(inputMenuItems),    0,                    inputMenuItems      } ,
#endif

  { sizeof(poweronMenuItems),  0,                    poweronMenuItems    } ,
  { sizeof(resetRomMenuItems), 0,                    resetRomMenuItems   } 
};

/*
 * This class encapsulates the PROGMEM tables which describe how the various decimal
 * values are to be formatted.
 */
struct DecimalItem 
{
  char pmemIcon[3];
  byte pmemFlags;
  void *pmemValPtr;

  enum 
  {
    ONE_DECIMAL_PLACE = 0,
    TWO_DECIMAL_PLACES = 0x01,
    THREE_DECIMAL_PLACES = 0x02,
    RANGE_M9999_P9999 = 0,
    RANGE_0_1000 = 0x04,
    RANGE_0_32767 = 0x08,
    RANGE_10_32767 = 0x10,
    RANGE_M999_P999 = 0x20,
    NO_EDIT = 0x40,
    EDIT_MANUAL_ONLY = 0x80
  };

  byte flags() const 
  {
    return pgm_read_byte_near(&pmemFlags);
  }

  byte decimalPlaces() const
  {
    byte f = flags();
    if (f & TWO_DECIMAL_PLACES)
    {
      return 2;
    }
    if (f & THREE_DECIMAL_PLACES)
    {
      return 3;
    }
    return 1;
  }

  int minimumValue() const 
  {
    byte f = flags();
    if (f & RANGE_10_32767)
    {
      return 10;
    }
    if (f & (RANGE_0_1000 | RANGE_0_32767))
    {
      return 0;
    }
    if (f & RANGE_M999_P999)
    {
      return -999;
    }
    return -9999;
  }

  int maximumValue() const 
  {
    byte f = flags();
    if (f & RANGE_M999_P999)
    {
      return 999;
    }
    if (f & RANGE_0_1000)
    {
      return 1000;
    }
    if (f & (RANGE_0_32767 | RANGE_10_32767))
    {
      return 9999;
    }
    return 9999;
  }

  int currentValue() const 
  {
    int *p = (int *)pgm_read_word_near(&pmemValPtr);
    return *p;
  }
  
  void boundValue() const
  {
    int *p = (int *)pgm_read_word_near(&pmemValPtr);
    if (*p > this->maximumValue())
    {
      *p = this->maximumValue();
    }
    if (*p < this->minimumValue())
    {
      *p = this->minimumValue();
    }
  }

  int *valuePtr() const 
  {
    return (int *)pgm_read_word_near(&pmemValPtr);
  }
};

// This must be in the same order as the ITEM_* enumeration
PROGMEM DecimalItem decimalItemData[DECIMAL_ITEM_COUNT] =
{
  { { 'S', 'e', 't' }, DecimalItem::RANGE_M9999_P9999 | DecimalItem::ONE_DECIMAL_PLACE, &displaySetpoint },
  { { 'C', 'u', 'r' }, DecimalItem::RANGE_M9999_P9999 | DecimalItem::ONE_DECIMAL_PLACE | DecimalItem::NO_EDIT, &displayInput },
  { { 'O', 'u', 't' }, DecimalItem::RANGE_0_1000      | DecimalItem::ONE_DECIMAL_PLACE | DecimalItem::EDIT_MANUAL_ONLY, &manualOutput },
  { { 'P', ' ', ' ' }, DecimalItem::RANGE_0_32767     | DecimalItem::THREE_DECIMAL_PLACES, &PGain },
  { { 'I', ' ', ' ' }, DecimalItem::RANGE_0_32767     | DecimalItem::THREE_DECIMAL_PLACES, &IGain },
  { { 'D', ' ', ' ' }, DecimalItem::RANGE_0_32767     | DecimalItem::THREE_DECIMAL_PLACES, &DGain },
  { { 'C', 'a', 'l' }, DecimalItem::RANGE_M999_P999   | DecimalItem::ONE_DECIMAL_PLACE, &displayCalibration },
  { { 'C', 'y', 'c' }, DecimalItem::RANGE_0_32767     | DecimalItem::TWO_DECIMAL_PLACES, &displayWindow },
  { { 'M', 'i', 'n' }, DecimalItem::RANGE_M9999_P9999 | DecimalItem::ONE_DECIMAL_PLACE, &lowerTripLimit },
  { { 'M', 'a', 'x' }, DecimalItem::RANGE_M9999_P9999 | DecimalItem::ONE_DECIMAL_PLACE, &upperTripLimit }
};

struct MenuStateData 
{
  byte currentMenu;
  byte firstItemMenuIndex;
  byte highlightedItemMenuIndex;  
  byte editDepth;
  unsigned long editStartMillis;
  bool editing;
};

struct MenuStateData menuState;

int pow10(byte n)
{
  int result = 1;

  while (n--)
    result *= 10;

  return result;
}

// print n blanks to LCD
void __attribute__ ((noinline)) LCDspc(byte n)
{
  for (byte i = n; i > 0; i--)
    lcd.print(' ');
}

// print text from PROGMEM to LCD and fill in with blanks to the end of the line 
void __attribute__ ((noinline)) LCDprintln(const char* s)
{
  char c;
  byte i = 16;
  while ((c = (char) pgm_read_byte_near(s++)) && (i > 0))
  {
    lcd.print(c);
    i--;
  }
  LCDspc(i);
}

static void __attribute__ ((noinline)) LCDsetCursorTopLeft()
{
  lcd.setCursor(0, 0);
}

static void __attribute__ ((noinline)) LCDsetCursorBottomLeft()
{
  lcd.setCursor(0, 1);
}
//LED normalization code to prevent red from overpowering other colors
//testing code right now uncommenting would result in "swirl" of rgb colors
// the entire time LCD was "on"
void setBacklight(uint8_t r, uint8_t g, uint8_t b) {
  // normalize the red LED - its brighter than the rest!
  r = map(r, 0, 255, 0, 100);
  g = map(g, 0, 255, 0, 150);
 
  r = map(r, 0, 255, 0, brightness);
  g = map(g, 0, 255, 0, brightness);
  b = map(b, 0, 255, 0, brightness);
 
  // common anode so invert!
  r = map(r, 0, 255, 255, 0);
  g = map(g, 0, 255, 255, 0);
  b = map(b, 0, 255, 255, 0);
  Serial.print("R = "); Serial.print(r, DEC);
  Serial.print(" G = "); Serial.print(g, DEC);
  Serial.print(" B = "); Serial.println(b, DEC);
  analogWrite(lcdREDPin, r);
  analogWrite(lcdGRNPin, g);
  analogWrite(lcdBLUPin, b);
 }

// draw the initial startup banner
void drawStartupBanner()
{
  // display a startup message
  lcd.clear();
  LCDsetCursorTopLeft();
  LCDprintln(PcontrollerName);
  LCDsetCursorBottomLeft();
  LCDprintln(Pversion);

}

// draw a profile name at the current position
static void drawProfileName(byte profileIndex)
{
  for (byte i = 0; i < 15; i++)
  {
    char ch = getProfileNameCharAt(profileIndex, i);
    lcd.print(ch ? ch : ' ');
  }
}

// draw a banner reporting that we're resuming an interrupted profile
void drawResumeProfileBanner()
{
  LCDsetCursorTopLeft();
  LCDprintln(PSTR("Resuming"));
  LCDsetCursorBottomLeft();
  drawProfileName(activeProfileIndex);
  delay(1000);
}

// can a given item be edited
static bool __attribute__ ((noinline)) canEditDecimalItem(const byte index)
{
  byte flags = decimalItemData[index].flags();

  return !(flags & DecimalItem::NO_EDIT) &&
    !((flags & DecimalItem::EDIT_MANUAL_ONLY) && (myPID.getMode() == PID::AUTOMATIC));
}

static bool __attribute__ ((noinline)) canEditItem(byte item)
{
  bool canEdit = !myPID.isTuning;

  if (item < FIRST_DECIMAL_ITEM)
  {
    canEdit = true; // menus always get a '>' selector
  }
  else if (item < FIRST_ACTION_ITEM)
  {
    canEdit = canEdit && canEditDecimalItem(item - FIRST_DECIMAL_ITEM);
  }
  return canEdit;
}

static void __attribute__ ((noinline)) startEditing(byte item)
{
  menuState.editing = true;
  if (item < FIRST_ACTION_ITEM)
  {
    menuState.editDepth = firstDigitPosition;
  }
  else
  {
    menuState.editDepth = 1;
  }

  menuState.editStartMillis = millis();

  if (canEditItem(item))
  {
    lcd.cursor();
  }
}

static void __attribute__ ((noinline)) stopEditing()
{
  menuState.editing = false;
  lcd.noCursor();
}

// draw the selector character at the current position
static void __attribute__ ((noinline)) drawSelector(byte item, bool selected)
{
  if (!selected)
  {
    lcd.print(' ');
    return;
  }

  bool canEdit = canEditItem(item);

  if (menuState.editing && !canEdit && (millis() > menuState.editStartMillis + 1000))
  {
    // cancel the disallowed edit
    stopEditing();
  }

  if (menuState.editing)
  {
    lcd.print(canEdit ? char(126) : 'X'); // char(126) = arrow pointing right
  }
  else
  {
    lcd.print(canEdit ? '>' : '|');
  }
}

// draw an item which takes up half a row (4 characters),
// for 2x2 menu mode
static void drawHalfRowItem(byte row, byte col, bool selected, byte item)
{ 
  switch (item)
  {
  case ITEM_SETPOINT1:
  case ITEM_SETPOINT2:
  case ITEM_SETPOINT3:
  case ITEM_SETPOINT4: 
    lcd.setCursor(col, row);
    drawSelector(item, selected);
    lcd.print(F("Sv")); 
    lcd.print(char('1' + item - ITEM_SETPOINT1));
    LCDspc(col == 0 ? 1 : 7);
    break;
  default:
  
#if !defined (ATMEGA_32kB_FLASH)
    BUGCHECK();
#else    
    ;
#endif  

  }
}

// This function converts a decimal fixed-point integer to a string,
// using the requested number of decimals. The string value is
// right-justified in the buffer, and the return value is a pointer
// to the first character in the formatted value. Characters between
// the start of the buffer and the return value are left unchanged.
static char *formatDecimalValue(char buffer[7], int num, byte decimals)
{
  byte decimalPos = (decimals == 0) ? 255 : 5 - decimals;
  bool isNegative = (num < 0);
  num = abs(num);

  buffer[6] = '\0';
  for (byte i = 5; i >= 0; i--)
  {
    if (i == decimalPos)
    {
      buffer[i] = '.';
      continue;
    }
    if (num == 0)
    {
      if (i >= decimalPos - 1)
      {
        buffer[i] = '0';
      }
      else if (isNegative)
      {
        buffer[i] = '-';
        return &buffer[i];
      }
      else
        return &buffer[i+1];
    }
    byte digit = num % 10;
    num /= 10;
    buffer[i] = digit + '0';
  }
  return buffer;
}

// draw a floating-point item's value at the current position
static void drawDecimalValue(byte item)
{
  char buffer[lastDigitPosition + 1];
  memset(&buffer, ' ', lastDigitPosition + 1);
  byte itemIndex = item - FIRST_DECIMAL_ITEM;
  int num = decimalItemData[itemIndex].currentValue();
  const byte decimals = decimalItemData[itemIndex].decimalPlaces();

  const char *s = decimalItemData[itemIndex].pmemIcon;
  for (byte i = 0; i < 3; i++ )
  {
    buffer[i] = (char) pgm_read_byte_near(s++);
  }
  // flash "Trip" for the setpoint if the controller has hit a limit trip
  if (tripped && (item == ITEM_SETPOINT))
  {
    if (now & 0x400)
    {
      strcpy_P(&buffer[firstDigitPosition - 1], PSTR("Alarm"));
    }
  }
  else if ((num == -19999) && (item == ITEM_INPUT))
  {
    // display an error
    if (now & 0x400)
    {
      strcpy_P(&buffer[firstDigitPosition + 1], PSTR("Err"));
    }
  }
  else
  {
    formatDecimalValue(&buffer[firstDigitPosition - 2], num, decimals);
  }
  char c;
  for (byte i = 0; i < lastDigitPosition; i++ )
  {
    c = buffer[i];
    lcd.print((c == 0) ? ' ' : c);
  }
}

// draw an item occupying a full 8x1 display line
static void drawFullRowItem(byte row, bool selected, byte item)
{
  lcd.setCursor(0, row);

  // first draw the selector
  drawSelector(item, selected);

  // then draw the item
  if ((item >= FIRST_DECIMAL_ITEM) && (item < FIRST_ACTION_ITEM))
  {
    drawDecimalValue(item);
    byte spc = 13 - lastDigitPosition;
    switch (item)
    { 
    case ITEM_SETPOINT:
      if (tripped)
      {
        spc = 15 - lastDigitPosition;
        break;
      }
    case ITEM_INPUT:
    case ITEM_CALIBRATION:
    case ITEM_LOWER_TRIP_LIMIT:
    case ITEM_UPPER_TRIP_LIMIT:
#if !defined (UNITS_FAHRENHEIT)
      lcd.print(F(" \337C"));
#else
      lcd.print(F(" \337F"));
#endif
      spc--;
      break;
    case ITEM_WINDOW_LENGTH:
      lcd.print(F(" s"));
      break;
    case ITEM_OUTPUT:
      lcd.print(F(" %"));
    default:
      spc = 15 - lastDigitPosition;
    }
    LCDspc(spc);
  }
  else switch (item)
  {
  case ITEM_DASHBOARD_MENU:
    LCDprintln(PSTR("Dashboard"));
    break;
  case ITEM_TUNING_MENU:
    LCDprintln(PSTR("Tuning"));
    break;
  case ITEM_CONFIG_MENU:
    LCDprintln(PSTR("Config"));
    break;
  case ITEM_PROFILE_MENU:
    if (runningProfile)
    {
      LCDprintln(PSTR("Cancel"));
    }
    else
    {
      drawProfileName(activeProfileIndex);
    }
    break;
    // case ITEM_SETPOINT_MENU: should not happen
  case ITEM_POWERON_MENU:
    LCDprintln(PSTR("Power On"));
    break;
  case ITEM_TRIP_MENU:
    LCDprintln(PSTR("Alarm"));
    break;
 
#if !defined (USE_SIMULATOR)
  case ITEM_INPUT_MENU:
    LCDprintln(PSTR("Sensor"));
    break;
#endif
    
  case ITEM_RESET_ROM_MENU:
    LCDprintln(PSTR("Reset Memory"));
    break;
  case ITEM_AUTOTUNE_CMD:
    if (myPID.isTuning)
    {
      LCDprintln(PSTR("Cancel"));
    }
    else
    {
      LCDprintln(PSTR("Auto Tuning"));
    }
    break;
  case ITEM_PROFILE1:
  case ITEM_PROFILE2:
  case ITEM_PROFILE3:
    drawProfileName(item - ITEM_PROFILE1);
    break;
    //case ITEM_SETPOINT1:
    //case ITEM_SETPOINT2:
    //case ITEM_SETPOINT3:
    //case ITEM_SETPOINT4: should not happen
  case ITEM_PID_MODE:
    if (myPID.getMode() == PID::MANUAL)
    {
      LCDprintln(PSTR("Manual Control"));
    }
    else
    {
      LCDprintln(PSTR("PID Control"));
    }
    break;
  case ITEM_PID_DIRECTION:
    if (myPID.getDirection() == PID::DIRECT)
    {
      LCDprintln(PSTR("Direct Action"));
    }
    else
    {
      LCDprintln(PSTR("Reverse Action"));
    }
    break;
    
#if !defined (USE_SIMULATOR) 
  case ITEM_INPUT_THERMISTOR:
    LCDprintln(PSTR("Thermistor"));
    break;
  case ITEM_INPUT_ONEWIRE:   
    LCDprintln(PSTR("DS18B20+"));
    break;
  case ITEM_INPUT_THERMOCOUPLE:
    LCDprintln(PSTR("Thermocouple"));
    break;  
#endif    

  case ITEM_POWERON_DISABLE:
    LCDprintln(PSTR("Manual Control"));
    break;
  case ITEM_POWERON_CONTINUE:
    LCDprintln(PSTR("Hold Set Point"));
    break;
  case ITEM_POWERON_RESUME_PROFILE:
    LCDprintln(PSTR("Resume Profile"));
    break;
  case ITEM_TRIP_ENABLED:
    if (tripLimitsEnabled)
    {
      LCDprintln(PSTR("Alarm Enabled")); 
    }
    else
    {
      LCDprintln(PSTR("Alarm Disabled"));
    }
    break;
  case ITEM_TRIP_AUTORESET:
    if (tripAutoReset)
    {
      LCDprintln(PSTR("Auto Reset"));
    }
    else
    {
      LCDprintln(PSTR("Manual Reset"));
    }
    break;
  case ITEM_RESET_ROM_NO:
    LCDprintln(PSTR("No"));
    break;
  case ITEM_RESET_ROM_YES:
    LCDprintln(PSTR("Yes"));
    break;
  default:
  
#if !defined (ATMEGA_32kB_FLASH)
    BUGCHECK();
#else    
    ;
#endif

  }
}

// flash a status indicator if appropriate
static void drawStatusFlash()
{
  byte flashState = ((millis() & 0xC00) >> 10);

  char ch = 0;
  if (tripped && (flashState > 0))
  {   
    if ((flashState & 1) > 0) 
    {
      ch = '!';
    }
  }
  else if (myPID.isTuning && (flashState > 0))
  {
    if ((flashState & 1) > 0) 
    {
      ch = 'T';
    }
  }
  else if (runningProfile && (flashState > 1))
  {
    if (flashState == 2)
    {
      ch = 'P';
    }
    else
    {
      ch = currentProfileStep + 'A';
    }
  }
  else
    ch = 0;
  drawNotificationCursor(ch);
}

void drawMenu()
{
  byte menu = menuState.currentMenu;
  byte itemCount = menuData[menu].itemCount();
  if (menuData[menu].is2x2())
  {
    // NOTE: right now the code only supports one screen (<= 4 items) in
    // 2x2 menu mode
    
#if !defined (ATMEGA_32kB_FLASH)
    ospAssert(itemCount <= 4);
#endif

    for (byte i = 0; i < itemCount; i++) 
    {
      bool highlight = (i == menuState.highlightedItemMenuIndex);
      byte item = menuData[menu].itemAt(i);
      drawHalfRowItem(i / 2, 5 * (i & 1), highlight, item);
    }
  }
  else
  {
    // 2x1 format; supports an arbitrary number of items in the menu
    byte first = menuState.firstItemMenuIndex;
    bool highlightFirst = (menuState.highlightedItemMenuIndex == first);
    
#if !defined (ATMEGA_32kB_FLASH)
    ospAssert(first + 1 < itemCount);
#endif

    drawFullRowItem(0,  highlightFirst, menuData[menu].itemAt(first));
    drawFullRowItem(1, !highlightFirst, menuData[menu].itemAt(first + 1));
  }

  // certain ongoing states flash a notification in the cursor slot
  drawStatusFlash();
}

// draw a character at the current location of the selection indicator
// (it will be overwritten at the next screen redraw)
//
// if icon is '\0', then just set the cursor at the editable location
void __attribute__ ((noinline)) drawNotificationCursor(char icon)
{
  byte highlightedIndex = menuState.highlightedItemMenuIndex;
  byte row, col;
  if (menuData[menuState.currentMenu].is2x2())
  {
    
#if !defined ATMEGA_32kB_FLASH
    ospAssert(!menuState.editing);
#endif    

    if (!icon)
    {
      return;
    }

    row = highlightedIndex / 2;
    col = 5 * (highlightedIndex & 1);
  }
  else
  {
    row = (highlightedIndex == menuState.firstItemMenuIndex) ? 0 : 1;
    col = 0;
  }

  if (icon)
  {
    lcd.setCursor(col, row);
    lcd.print(icon);
  }

  if (menuState.editing)
  {
    lcd.setCursor(menuState.editDepth, row);
  }
}

void backKeyPress()
{
  byte menu = menuState.currentMenu;
  if (menuState.editing)
  {
    // step the cursor one place to the left and stop editing if required
    menuState.editDepth--;
    byte item = menuData[menu].itemAt(menuState.highlightedItemMenuIndex);
    if (item < FIRST_ACTION_ITEM)
    {
      // floating-point items have a decimal point, which we want to jump over
      if (menuState.editDepth == lastDigitPosition - decimalItemData[item - FIRST_DECIMAL_ITEM].decimalPlaces())
      {
        menuState.editDepth--;
      }
    }

    if (menuState.editDepth < firstDigitPosition)
    {
      stopEditing();   
    }

    return;
  }

  // go up a level in the menu hierarchy
  byte prevMenu = menu;
  switch (prevMenu)
  {
  case ITEM_MAIN_MENU:
    break;
  case ITEM_DASHBOARD_MENU:
  case ITEM_TUNING_MENU:
  case ITEM_CONFIG_MENU:
  case ITEM_PROFILE_MENU:
    menuState.currentMenu = ITEM_MAIN_MENU;
    menuState.highlightedItemMenuIndex = prevMenu - ITEM_DASHBOARD_MENU;
    menuState.firstItemMenuIndex = prevMenu - ITEM_DASHBOARD_MENU;
    break;
  case ITEM_SETPOINT_MENU:
    menuState.currentMenu = ITEM_DASHBOARD_MENU;
    menuState.highlightedItemMenuIndex = 0;
    menuState.firstItemMenuIndex = 0;
    break;
  case ITEM_TRIP_MENU:
    menuState.currentMenu = ITEM_DASHBOARD_MENU;
    menuState.highlightedItemMenuIndex = 4;
    menuState.firstItemMenuIndex = 3;
    break;
    
#if !defined (USE_SIMULATOR)
  case ITEM_INPUT_MENU:
    menuState.currentMenu = ITEM_CONFIG_MENU;
    menuState.highlightedItemMenuIndex = 0;
    menuState.firstItemMenuIndex = 0;
    break;
#endif 

  case ITEM_POWERON_MENU:
  case ITEM_RESET_ROM_MENU:
    menuState.currentMenu = ITEM_CONFIG_MENU;
    
#if !defined (USE_SIMULATOR)
    menuState.highlightedItemMenuIndex = prevMenu - ITEM_INPUT_MENU + 2;
    menuState.firstItemMenuIndex = menuState.highlightedItemMenuIndex - 1;
#else    
    menuState.highlightedItemMenuIndex = 0;
    menuState.firstItemMenuIndex = 0;
#endif

    break;
  default:
  
#if !defined (ATMEGA_32kB_FLASH)
    BUGCHECK();
#else    
    ;
#endif    

  }
}

void updownKeyPress(bool up)
{
  byte menu = menuState.currentMenu;
  if (!menuState.editing)
  {
    // menu navigation
    if (up)
    {
      if (menuState.highlightedItemMenuIndex == 0)
      {
        return;
      }
      if (menuData[menu].is2x2())
      {
        menuState.highlightedItemMenuIndex--;
        return;
      }
      if (menuState.highlightedItemMenuIndex == menuState.firstItemMenuIndex)
      {
        menuState.firstItemMenuIndex--;
      }
      menuState.highlightedItemMenuIndex = menuState.firstItemMenuIndex;
      return;
    }

    // down
    byte menuItemCount = menuData[menu].itemCount();
    if (menuState.highlightedItemMenuIndex == menuItemCount - 1)
    {
      return;
    }
    menuState.highlightedItemMenuIndex++;
    if (menuData[menu].is2x2())
    {
      return;
    }
    if (menuState.highlightedItemMenuIndex != menuState.firstItemMenuIndex + 1)
    {
      menuState.firstItemMenuIndex = menuState.highlightedItemMenuIndex - 1;
    }
    return;
  }

  // editing a number or a setting
  byte item = menuData[menu].itemAt(menuState.highlightedItemMenuIndex);

  if (!canEditItem(item))
  {
    return;
  }

  // _something_ is going to change, so the settings are now dirty
  markSettingsDirty();

  if (item >= FIRST_ACTION_ITEM)
  {
    switch (item)
    {
    case ITEM_PID_MODE:
      myPID.setMode(myPID.getMode() ^ 1);
      // use the manual output value
      if (myPID.getMode() == PID::MANUAL)
      {
        setOutputToManualOutput();
      }
      break;
    case ITEM_PID_DIRECTION:
      myPID.setControllerDirection(myPID.getDirection() ^ 1);
      break;
    case ITEM_TRIP_ENABLED:
      tripLimitsEnabled = !tripLimitsEnabled;
      break;
    case ITEM_TRIP_AUTORESET:
      tripAutoReset = !tripAutoReset;
      break;
    default:
    
#if !defined (ATMEGA_32kB_FLASH)
    BUGCHECK();
#else    
    ;
#endif

    }
    return;
  }

  // not a setting: must be a number

  // determine how much to increment or decrement
  const byte itemIndex = item - FIRST_DECIMAL_ITEM;
  byte decimalPointPosition = lastDigitPosition - decimalItemData[itemIndex].decimalPlaces();
  int increment = pow10(lastDigitPosition - menuState.editDepth - (menuState.editDepth < decimalPointPosition ? 1 : 0));

  if (!up)
  {
    increment = -increment;
  }

  // do the in/decrement and clamp it
  int val = decimalItemData[itemIndex].currentValue();
  int *valPtr = decimalItemData[itemIndex].valuePtr();
  *valPtr = val + increment;
  decimalItemData[itemIndex].boundValue();
  
  // capture changes
  if (item == ITEM_SETPOINT)
  {
    setPoints[setPointIndex] = displaySetpoint;
    updateActiveSetPoint();
  }
  
#if !defined (USE_SIMULATOR)
  if (item == ITEM_WINDOW_LENGTH)
  {
    theOutputDevice.setOutputWindowSeconds(displayWindow);
  }
  if (item == ITEM_CALIBRATION)
  {
    theInputDevice.setCalibration(displayCalibration);
  }
#endif    

  // capture any possible changes to the output value if we're in MANUAL mode
  if ((item == ITEM_OUTPUT) && (myPID.getMode() == PID::MANUAL) && !myPID.isTuning && !tripped)
  {
    setOutputToManualOutput();
  }
}

void okKeyPress()
{
  byte item = menuData[menuState.currentMenu].itemAt(menuState.highlightedItemMenuIndex);

  if (menuState.editing)
  {
    // step the cursor one digit to the right, or stop editing
    // if it has advanced off the screen
    menuState.editDepth++;
    if (item < FIRST_ACTION_ITEM)
    {
      // floating-point items have a decimal point, which we want to jump over
      if (menuState.editDepth == lastDigitPosition - decimalItemData[item - FIRST_DECIMAL_ITEM].decimalPlaces())
      {
        menuState.editDepth++;
      }
    }
    
    if ((menuState.editDepth > lastDigitPosition) || (item >= FIRST_ACTION_ITEM))
    {
      stopEditing();
    }
    return;
  }

  if (item < FIRST_DECIMAL_ITEM)
  {
    // the profile menu is special: a short-press on it triggers the
    // profile or cancels one that's in progress
    if (item == ITEM_PROFILE_MENU)
    {
      if (runningProfile)
      {
        stopProfile();
      }
      else if (!myPID.isTuning)
      {
        startProfile();
      }
      return;
    }

    // it's a menu: open that menu
    menuState.currentMenu = item;
    switch (item)
    {
    case ITEM_PROFILE_MENU:
      menuState.highlightedItemMenuIndex = activeProfileIndex;
      break;
    case ITEM_SETPOINT_MENU:
      menuState.highlightedItemMenuIndex = setPointIndex;
      break;
      
#if !defined (USE_SIMULATOR)
    case ITEM_INPUT_MENU:
      menuState.highlightedItemMenuIndex = theInputDevice.ioType;
      break;
#endif

    case ITEM_POWERON_MENU:
      menuState.highlightedItemMenuIndex = powerOnBehavior;
      break;
    default:
      menuState.highlightedItemMenuIndex = 0;
      break;
    }
    menuState.firstItemMenuIndex = min(menuState.highlightedItemMenuIndex, menuData[item].itemCount() - 2);
    return;
  }

  if (item < FIRST_ACTION_ITEM)
  {
    // the setpoint flashes "Trip" if the unit has tripped; OK clears the trip
    if (tripped && (item == ITEM_SETPOINT))
    {
      tripped = false;
      setOutputToManualOutput();

      return;
    }
    // it's a numeric value: mark that the user wants to edit it
    // (the cursor will change if they can't)
    startEditing(item);
    return;
  }

  // it's an action item: some of them can be edited; others trigger
  // an action
  switch (item)
  {
  case ITEM_AUTOTUNE_CMD:
    if (runningProfile)
    {
      break;
    }
    if (!myPID.isTuning)
    {
      myPID.startAutoTune(aTuneMethod, aTuneStep, aTuneNoise, aTuneLookBack);
    }
    else
    {
      myPID.stopAutoTune();
    }
    break;

  case ITEM_PROFILE1:
  case ITEM_PROFILE2:
  case ITEM_PROFILE3:
    activeProfileIndex = item - ITEM_PROFILE1;
    if (!myPID.isTuning)
    {
      startProfile();
    }
    markSettingsDirty();

    // return to the prior menu
    backKeyPress();
    break;

  case ITEM_SETPOINT1:
  case ITEM_SETPOINT2:
  case ITEM_SETPOINT3:
  case ITEM_SETPOINT4:
    setPointIndex = item - ITEM_SETPOINT1;
    updateActiveSetPoint();
    markSettingsDirty();

    // return to the prior menu
    backKeyPress();
    break;

  case ITEM_PID_MODE:
  case ITEM_PID_DIRECTION:
  case ITEM_TRIP_ENABLED:
  case ITEM_TRIP_AUTORESET:
    startEditing(item);
    break;
    
#if !defined (USE_SIMULATOR)
  case ITEM_INPUT_THERMISTOR:
  case ITEM_INPUT_ONEWIRE:
  case ITEM_INPUT_THERMOCOUPLE:
    // update inputType
    theInputDevice.ioType = (item - ITEM_INPUT_THERMISTOR);
#else
    // FIXME could maybe use this menu to choose between simulations
#endif  

    theInputDevice.initialize();
    markSettingsDirty();

    // return to prior menu
    backKeyPress();
    break;

  case ITEM_POWERON_DISABLE:
  case ITEM_POWERON_CONTINUE:
  case ITEM_POWERON_RESUME_PROFILE:
    powerOnBehavior = (item - ITEM_POWERON_DISABLE);
    markSettingsDirty();

    // return to the prior menu
    backKeyPress();
    break;

  case ITEM_RESET_ROM_NO:
    backKeyPress();
    break;
  case ITEM_RESET_ROM_YES:
    clearEEPROM();

    // perform a software reset by jumping to 0x0000, which is the start of the application code
    //
    // it would be better to use a Watchdog Reset
    typedef void (*VoidFn)(void);
    ((VoidFn) 0x0000)();
    break;
  default:
  
#if !defined (ATMEGA_32kB_FLASH)
    BUGCHECK();
#else    
    ;
#endif

  }
}

// returns true if there was a long-press action; false if a long press
// is the same as a short press
bool okKeyLongPress()
{
  byte item = menuData[menuState.currentMenu].itemAt(menuState.highlightedItemMenuIndex);

  // don't try to open menus while the user is editing a value
  if (menuState.editing)
  {
    return false;
  }

  // only two items respond to long presses: the setpoint and the profile menu
  if (item == ITEM_SETPOINT)
  {
    // open the setpoint menu
    menuState.currentMenu = ITEM_SETPOINT_MENU;
    menuState.firstItemMenuIndex = 0;
    menuState.highlightedItemMenuIndex = setPointIndex;
  }
  else if (item == ITEM_PROFILE_MENU)
  {
    // open the profile menu
    menuState.currentMenu = ITEM_PROFILE_MENU;
    menuState.highlightedItemMenuIndex = activeProfileIndex;
    menuState.firstItemMenuIndex = (activeProfileIndex == 0 ? 0 : 1);
  }
  else
    return false;

  return true;
}

// draw a banner reporting a bad EEPROM checksum
void drawBadCsum(byte profile)
{
  delay(500);
  LCDsetCursorTopLeft();
  if (profile == 0xFF)
  {
    LCDprintln(PSTR("Settings"));
  }
  else
  {
    LCDprintln(Pprofile);
    lcd.setCursor(8, 0);
    lcd.print(char(profile + '1'));
  }
  LCDsetCursorBottomLeft();
  LCDprintln(PSTR("Cleared"));
  delay(2000);
}
