
#ifndef ADAFRUIT_MAX31855_H
#define ADAFRUIT_MAX31855_H

#if (ARDUINO >= 100)
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

class Adafruit_MAX31855 {
 public:
  Adafruit_MAX31855(int8_t SCLK, int8_t CS, int8_t MISO);

  double readInternal(void);
  double readCelsius(void);
  double readFarenheit(void);
  uint8_t readError();

 private:
  int8_t sclk, miso, cs;
  uint32_t spiread32(void);
};

#endif
