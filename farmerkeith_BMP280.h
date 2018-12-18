/*
  farmerkeith_BMP280.h - library for the Bosch Sensortec BMP280 pressure and temperature sensor
  created by Keith Hungerford, 15 september 2017
  Updated for bme280 an reorganising code 18 Dec 2018
  Released into the public domain for personal use and not-for-profit projects.
*/
#ifndef farmerkeith_BMP280_h
#define farmerkeith_BMP280_h

#include "Arduino.h"

class bmp280 { // for use with BMP280 sensors
  public:
    // constructors
    bmp280 (byte Addr, byte debug);
    bmp280 (byte Addr);
    bmp280 ();
    // begin functions
    void begin(byte osrs_t, byte osrs_p, byte mode, byte t_sb, byte filter, byte spi3W_en);
    void begin();                // get calibration factors, set configuration
    // temperature functions
    double readTemperature ();   // function
    double calcTemperature (long rawTemperature);
    double calcTemperature (long rawTemperature, double &t_fine);
    // pressure functions
    double readPressure (); // function, no parameter
    double readPressure (double &temperature); // function
    long readRawPressure (long &rawTemperature); // function
    double calcPressure (long rawPressure, double t_fine);
    // utility functions
    float calcAltitude (double pressure, float seaLevelhPa);
    float calcAltitude (double pressure); // no parameter, standard seaLevelPressure assumed
    float calcNormalisedPressure (double pressure, float altitude);
    // configuration controls
    byte readF4Sleep();           // function
    byte readF5Sleep();           // function
    byte updateF4Control(byte osrs_t, byte osrs_p, byte mode); // function
    byte updateF4Control16xNormal();       // function
    byte updateF4Control16xSleep();        // function
    byte updateF4ControlSleep(byte value); // function
    byte updateF5Config(byte t_sb, byte filter, byte spi3W_en);// function
    byte updateF5Config1msIIR16I2C();      // function
    byte updateF5ConfigSleep(byte value);  // function
    // calibration functions
    void getBmpCalibratonData();            // function
    // general tools
    byte readRegister(byte reg);           // function
    byte updateRegister(byte reg, byte value); // function
    byte readByteArray(byte reg, byte length, byte data[]); // function
    // public data and variables
    byte bmp280Debug = 0;
    // BMP280 I2C address is 0x76(108) when pin SDO is connected to GND
    // or 0x77(109) when pin SDO is connected to Vddio (+3.3V)
    const byte bmp280Addr = 0x76 ; // base address
    byte address;                  // base address + device index
  private:
    // private variables
    uint16_t dig_T1, dig_P1;                // temperature and pressure calibration
    int16_t dig_T2, dig_T3;                 // temperature calibration
    int16_t dig_P2, dig_P3, dig_P4, dig_P5; // pressure calibration
    int16_t dig_P6, dig_P7, dig_P8, dig_P9; // pressure calibration

}; // end of class bmp280

class bme280 : public bmp280 {             // for use with bme280 sensors
    // class bme280 inherits all the data and functions of bmp280
  public:
    // constructors
    bme280 (byte Addr, byte debug);
    bme280 (byte Addr);
    bme280 ();
    // begin functions
    void begin();                          // get calibration factors, set configuration
    void begin(byte osrs_t, byte osrs_p, byte mode, byte t_sb, byte filter, byte spi3W_en, byte osrs_h);
    // humidity functions
    double readHumidity ();                // function
    double readHumidity (double &temperature, double &pressure);  // function
    long readRawHumidity (long &rawTemperature, long &rawPressure); // reads raw humidity, temperature and pressure
    double calcHumidity(long rawHumidity, double t_fine);
    // configuration controls
    void updateF2Control(byte osrs_h);     // function

  private:
    // calibration functions
    void getBmeCalibratonData();           // function
    // variables
    uint8_t  dig_H1, dig_H3;           // humidity calibration
    int16_t  dig_H2, dig_H4, dig_H5;   // humidity calibration
    int8_t   dig_H6;                   // humidity calibration
}; // end of class bme280

#endif // for #ifndef farmerkeith_BMP280_h
// end of file

