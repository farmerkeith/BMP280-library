/*
  farmerkeith_BMP280.h - library for the Bosch Sensortec BMP280 pressure and temperature sensor
  created by Keith Hungerford, 15 september 2017
  Released into the public domain.
*/
#ifndef farmerkeith_BMP280_h
#define farmerkeith_BMP280_h

#include "Arduino.h"

class bmp280 {
  public:
  bmp280 (byte Addr, byte debug) ;         // constructor with 2 parameters
  bmp280 (byte Addr) ;                     // constructor with 1 parameter
  bmp280 () ;                              // constructor with no parameters
  void begin(byte osrs_t, byte osrs_p, byte mode, byte t_sb, byte filter, byte spi3W_en); 
  void begin();                // get calibration factors, set configuration
  double readTemperature ();   // function
  double readPressure (double &temperature); // function
  double readPressure (); // function, no parameter 
  
  long readRawPressure (long &rawTemperature); // function
  double calcPressure (long rawPressure, long rawTemperature, double &temperature);
  double calcTemperature (long rawTemperature); 
  float calcAltitude (double pressure, float seaLevelhPa);
  float calcAltitude (double pressure); // no parameter, standard seaLevelPressure assumed
  float calcNormalisedPressure (double pressure, float altitude);

  void getCalibratonData();              // function
  byte readRegister(byte reg);           // function
  byte readF4Sleep();           // function
  byte readF5Sleep();           // function
  void updateRegister(byte reg, byte value); // function
  void updateF4Control(byte osrs_t, byte osrs_p, byte mode); // function 
  void updateF4Control16xNormal();       // function
  void updateF4Control16xSleep();        // function
  void updateF4ControlSleep(byte value); // function
  void updateF5Config(byte t_sb, byte filter, byte spi3W_en);// function
  void updateF5Config1msIIR16I2C();      // function
  void updateF5ConfigSleep(byte value);  // function

  private:
  // variables
  // BMP280 I2C address is 0x76(108) when pin SDO is connected to GND
  // or 0x77(109) when pin SDO is connected to Vddio (+3.3V)
  const byte bmp280Addr = 0x76 ; // base address
  byte address;                  // base address + device index
  byte bmp280Debug=0;
  unsigned long dig_T1, dig_P1;        // temperature and pressure calibration
  long dig_T2, dig_T3;                 // temperature calibration
  long dig_P2, dig_P3, dig_P4, dig_P5; // pressure calibration
  long dig_P6, dig_P7, dig_P8, dig_P9; // pressure calibration

}; // end of class bmp280 

#endif // for #ifndef farmerkeith_BMP280_h
// end of file

