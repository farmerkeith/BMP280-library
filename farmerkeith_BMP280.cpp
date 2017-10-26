/* 
  farmerkeith_BMP280.cpp - library for the Bosch Sensortec BMP280 pressure and temperature sensor
  created by Keith Hungerford, 15 september 2017
  Released into the public domain.
*/

#include "Arduino.h"
#include "farmerkeith_BMP280.h"
#include "Wire.h" // for I2C protocol
// *****************************************************
bmp280::bmp280 (byte Addr, byte debug){        // constructor for class bmp280
  address = bmp280Addr+Addr; 
  bmp280Debug = debug;
} // end of bmp280 constructor

bmp280::bmp280 (byte Addr){                     // constructor with 1 parameter
  address = bmp280Addr+Addr; 
  bmp280Debug = 0;
}
bmp280::bmp280 (){                              // constructor with no parameters
  address = bmp280Addr; 
  bmp280Debug = 0;
}

// *****************************************************

void bmp280::begin(){
  begin(7, 7, 3, 0, 0, 0); // defaults are 16x; Normal mode; 0.5ms, no filter, I2C
}

void bmp280::begin(byte osrs_t, byte osrs_p, byte mode, byte t_sb, byte filter, byte spi3W_en){
  // get individual calibration constants from bmp280
  // also set control and configuration registers
  // see update F4 and F5 for details of parameters
  if (bmp280Debug) {
    Serial.print("Initialising BMP280 object ");
    Serial.print (address - bmp280Addr);
    Serial.print (" at I2C address 0x");
    Serial.println(address,HEX);
  }
  getCalibratonData(); // includes conversion to integers and debug printing
  updateF4Control(osrs_t, osrs_p, mode);  // oversampling and mode
  updateF5Config(t_sb, filter, spi3W_en); // standby time, IIR filter, I2C select
} // end of void bmp280::begin(byte osrs_t, byte osrs_p, byte mode, byte t_sb, byte filter, byte spi3W_en)
// *****************************************************

double bmp280::readTemperature () {
  // function to read the temperature in BMP280, calibrate it and 
  // return the value in Celsius
  byte data[6]={0,0,0,0,0,0}; // array of 6 bytes used to store pressure and temperature data

  // get temperature and pressure data
    // Start I2C Transmission
    noInterrupts(); // disable interrupts
      Wire.beginTransmission(address);
      byte error = Wire.endTransmission();
      if (error){
        Serial.print ("I2C error with address ");
        Serial.print (address, HEX); 
        Serial.print (" error code= "); 
        Serial.print (error); 
        Serial.print (" millis= "); 
        Serial.println (millis()); 
      } // end of if (error)
    
    Wire.beginTransmission(address);
    // Select data register
    Wire.write(0xFA); // 0xFA hex base address of temperature data
    // Stop I2C Transmission
    Wire.endTransmission();
    // Request 3 bytes of data
    Wire.requestFrom(address, 3);
    // Read 3 bytes of data
    if (Wire.available() == 3)  {
      for (byte i=3; i<6; i++) {
        data[i] = Wire.read();
      } // end of for (i=3; i<6; i++) 
    } // end of if (Wire.available() == 3) 
    interrupts(); // enable interrupts
  // ***************************************

  // Convert temperature data bytes to 20-bits within 32 bit integer
  long adc_t = (((long)(data[3] & 0xFF) * 65536) + ((long)(data[4] & 0xFF) * 256) + (long)(data[5] & 0xF0)) / 16;

    if (bmp280Debug){
    Serial.print ("BMP280-");
    Serial.print (address - bmp280Addr); // index
    Serial.print (" raw temperature=");
    Serial.println (adc_t);
    }

  // Temperature offset calculations
  double var1 = (((double)adc_t) / 16384.0 - ((double)dig_T1) / 1024.0) * ((double)dig_T2);
  double var2 = ((((double)adc_t) / 131072.0 - ((double)dig_T1) / 8192.0) *
                 (((double)adc_t) / 131072.0 - ((double)dig_T1) / 8192.0)) * ((double)dig_T3);
  double t_fine = (long)(var1 + var2);

  return (var1 + var2) / 5120.0; // cTemp
   
} // end of double readTemperature ()
// *****************************************************

double bmp280::readPressure (double &temperature) {
  // function to read the pressure in BMP280, calibrate it and 
  // return the value in kPa
  // also returns temperature
  long adc_p, adc_t;
  adc_p = readRawPressure (adc_t);
  return calcPressure (adc_p, adc_t, temperature);
  
} // end of double readPressure (double &temperature)

double bmp280::readPressure () { // without temperature return variable
  double temperature;
  return readPressure(temperature); // temperature is discarded
} // end of double bmp280::readPressure

long bmp280::readRawPressure (long &adc_t){
  // function to read the pressure in BMP280 without calibrating 
  // return the value as a 32 bit integer
  // also returns raw temperature as a 32 bit integer
  
  byte data[6]={0,0,0,0,0,0}; // array of 6 bytes used to store pressure and temperature data
  // Start I2C Transmission
  noInterrupts(); // disable interrupts
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();
    if (error){
      Serial.print ("I2C error with address ");
      Serial.print (address, HEX); 
      Serial.print (" error code= "); 
      Serial.print (error); 
      Serial.print (" millis= "); 
      Serial.println (millis()); 
    } // end of if (error)
    
  Wire.beginTransmission(address);
  // Select data register
  Wire.write(247); // 0xF7 hex base address of pressure and temperature data
  // Stop I2C Transmission
  Wire.endTransmission();
  // Request 6 bytes of data
  Wire.requestFrom(address, 6);
  // Read 6 bytes of data
  if (Wire.available() == 6)  {
    for (byte i=0; i<6; i++) {
      data[i] = Wire.read();
    } // end of for (i=0; i<6; i++) 
  } // end of if (Wire.available() == 6) 
  interrupts(); // enable interrupts
  // Convert pressure and temperature data to 19-bits
  long adc_p = (((long)(data[0] & 0xFF) * 65536) + ((long)(data[1] & 0xFF) * 256) + (long)(data[2] & 0xF0)) / 16;
  adc_t = (((long)(data[3] & 0xFF) * 65536) + ((long)(data[4] & 0xFF) * 256) + (long)(data[5] & 0xF0)) / 16;

  if (bmp280Debug){
    Serial.print ("BMP280-");
    Serial.print (address - bmp280Addr); // index
    Serial.print (" raw pressure=");
    Serial.println (adc_p);
  }
  return adc_p;
} // end of long readRawPressure (long &adc_t)

double bmp280::calcPressure (long adc_p, long adc_t, double &temperature){
  // uses stored calibration factors to convert raw pressure and temperature into hPa and degrees C

  // Temperature offset calculations
  double var1 = (((double)adc_t) / 16384.0 - ((double)dig_T1) / 1024.0) * ((double)dig_T2);
  double var2 = ((((double)adc_t) / 131072.0 - ((double)dig_T1) / 8192.0) *
                 (((double)adc_t) / 131072.0 - ((double)dig_T1) / 8192.0)) * ((double)dig_T3);
  double t_fine = (long)(var1 + var2);
  temperature = (var1 + var2) / 5120.0;

  // Pressure offset calculations
  var1 = ((double)t_fine / 2.0) - 64000.0;
  var2 = var1 * var1 * ((double)dig_P6) / 32768.0;
  var2 = var2 + var1 * ((double)dig_P5) * 2.0;
  var2 = (var2 / 4.0) + (((double)dig_P4) * 65536.0);
  var1 = (((double) dig_P3) * var1 * var1 / 524288.0 + ((double) dig_P2) * var1) / 524288.0;
  var1 = (1.0 + var1 / 32768.0) * ((double)dig_P1);
  double p = 1048576.0 - (double)adc_p;
  p = (p - (var2 / 4096.0)) * 6250.0 / var1;
  var1 = ((double) dig_P9) * p * p / 2147483648.0;
  var2 = p * ((double) dig_P8) / 32768.0;
  return (p + (var1 + var2 + ((double)dig_P7)) / 16.0) / 100; // pressure in hPa
} // end of double bmp280::calcPressure (long rawPressure, long rawTemperature, double &temperature)

double bmp280::calcTemperature (long adc_t){
  // uses stored calibration factors to convert raw temperature into degrees C
  // Temperature offset calculations
  double var1 = (((double)adc_t) / 16384.0 - ((double)dig_T1) / 1024.0) * ((double)dig_T2);
  double var2 = ((((double)adc_t) / 131072.0 - ((double)dig_T1) / 8192.0) *
                 (((double)adc_t) / 131072.0 - ((double)dig_T1) / 8192.0)) * ((double)dig_T3);
  double t_fine = (long)(var1 + var2);
  return (double)(var1 + var2) / 5120.0;
} // end double bmp280::calcTemperature (long adc_t)

float bmp280::calcAltitude (double pressure, float seaLevelhPa){
  // returns altitude in metres based on pressure and seaLevelpressure, both in hPa
  // standard seaLevelhPa = 1013.25
    return (float)44330 * (1.0 - pow(pressure / seaLevelhPa, 0.19026));
} // end of float bmp280::calcAltitude (double pressure, float seaLevelhPa)

float bmp280::calcAltitude (double pressure){
  // returns altitude in metres based on pressure and standard seaLevelpressure, both in hPa
  // standard seaLevelhPa = 1013.25
    return (float)44330 * (1.0 - pow(pressure / 1013.25, 0.19026));
} // end of float bmp280::calcAltitude (double pressure, float seaLevelhPa)

float bmp280::calcNormalisedPressure (double pressure, float altitude){
  // returns normalised pressure at sea level based on pressure and current altitude in metres
  return pressure / pow((1-altitude*0.0000225577), 5.25588);
} // end of float bmp280::calcNormalisedPressure (double pressure, float altitude)

// ********************************************

void bmp280::getCalibratonData(){ // function
  // get calibration data
  byte b1[24]; // array of 24 integers used to store calibration bytes.
  // Start I2C Transmission
  noInterrupts(); // disable interrupts
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();
    if (error){
      Serial.print ("I2C error with address ");
      Serial.print (address, HEX); 
      Serial.print (" error code= "); 
      Serial.print (error); 
      Serial.print ("millis= "); 
      Serial.println (millis()); 
    } // end of if (error)
    
  Wire.beginTransmission(address);
  // Select data register
  Wire.write(136); // 136 decimal = 0x88 Hex, start address of calibration data
  // Stop I2C Transmission
  Wire.endTransmission();

  // Request 24 bytes of data
  Wire.requestFrom(address, 24);
  // Read 1 byte of data

  byte byteCount=0;
  switch (Wire.available()){
    case 24: // correct answer
      byteCount  = 24;  break;
    case 0: 
      byteCount  = 0;
      Wire.requestFrom(address, 24);
      if (bmp280Debug) {
        Serial.println ("repeating Wire.requestFrom ");
      } // end of if (bmp280Debug) 
      break;
    case 1: 
      byteCount  = 1;  break;
    default: // incorrect answer
      break;
  } // end of switch (Wire.available())
    
  if (Wire.available() == 24)   {
     byte byteCount  = 24; // correct answer
     if (bmp280Debug) {
       Serial.print ("getting ");
       Serial.print (byteCount);
       Serial.println (" bytes of calibration parameters");
     } // end of if (bmp280Debug) 
  } else {
     if (bmp280Debug) {
       Serial.print ("byteCount not 24 but is ");
       Serial.println (byteCount);
     } // end of if (bmp280Debug) 
  } // end of if (Wire.available() == 24)   
    
    for (int i = 0; i < 24; i++)  { // reads all 24 calibration bytes
      b1[i] = Wire.read();
    } // end of //  for (int i = 0; i < 24; i++)
  interrupts(); // enable interrupts

  // Convert the data (actually calibration coefficients)
  // temp coefficients
  dig_T1 = b1[0] + (b1[1] * 256); // unsigned

  dig_T2 = b1[2] + (b1[3] * 256);
  if (b1[3]>>7) dig_T2 = dig_T2 + 0xFFFF0000;

  dig_T3 = b1[4] + (b1[5] * 256);
  if (b1[5]>>7) dig_T3 = dig_T3 + 0xFFFF0000;

  // pressure coefficients
  dig_P1 = (b1[6] & 0xFF) + ((b1[7] & 0xFF) * 256); // bitwise AND

  dig_P2 = b1[8] + (b1[9] * 256);
  if (b1[9]>>7) dig_P2 = dig_P2 + 0xFFFF0000;

  dig_P3 = b1[10] + (b1[11] * 256);
  if (b1[11]>>7) dig_P3 = dig_P3 + 0xFFFF0000;
  
  dig_P4 = b1[12] + (b1[13] * 256);
  if (b1[13]>>7) dig_P4 = dig_P4 + 0xFFFF0000;
  
  dig_P5 = b1[14] + (b1[15] * 256);
  if (b1[15]>>7) dig_P5 = dig_P5 + 0xFFFF0000;
  
  dig_P6 = b1[16] + (b1[17] * 256);
  if (b1[17]>>7) dig_P6 = dig_P6 + 0xFFFF0000;
  
  dig_P7 = b1[18] + (b1[19] * 256);
  if (b1[19]>>7) dig_P7 = dig_P7 + 0xFFFF0000;
  
  dig_P8 = b1[20] + (b1[21] * 256);
  if (b1[21]>>7) dig_P8 = dig_P8 + 0xFFFF0000;
  
  dig_P9 = b1[22] + (b1[23] * 256);
  if (b1[23]>>7) dig_P9 = dig_P9 + 0xFFFF0000;

  if (bmp280Debug) {
    Serial.println ("Calibration parameters");
    Serial.print ("Temperature dig_T1=");
    Serial.print (dig_T1);
    Serial.print (" dig_T2=");
    Serial.print (dig_T2);
    Serial.print (" dig_T3=");
    Serial.println (dig_T3);
    Serial.print ("Pressure    dig_P1=");
    Serial.print (dig_P1);
    Serial.print (" dig_P2=");
    Serial.print (dig_P2);
    Serial.print (" dig_P3=");
    Serial.print (dig_P3);
    Serial.print (" dig_P4=");
    Serial.println (dig_P4);
    Serial.print (" dig_P5=");
    Serial.print (dig_P5);
    Serial.print (" dig_P6=");
    Serial.print (dig_P6);
    Serial.print (" dig_P7=");
    Serial.print (dig_P7);
    Serial.print (" dig_P8=");
    Serial.print (dig_P8);
    Serial.print (" dig_P9=");
    Serial.println (dig_P9);
    Serial.print ("End of calibration parameters for BMP280-");
    Serial.println (address-bmp280Addr);
  } // end of if (bmp280Debug)
  
} // end of void bmp280::getCalibratonData()
// ********************************************

byte bmp280::readRegister(byte reg){
  // general function to read control register
  noInterrupts(); // disable interrupts
  Wire.beginTransmission(address); // start I2C transmission
  Wire.write(reg); // address of register to control oversampling and power mode
  byte error = Wire.endTransmission();
  if (error){
    Serial.print ("I2C error with address ");
    Serial.print (address, HEX); 
    Serial.print (" error code= "); 
    Serial.print (error); 
    Serial.print (" millis= "); 
    Serial.println (millis()); 
  } // end of if (error)
  // Request 1 byte of data
  Wire.requestFrom(address, 1);
  byte data = Wire.read();
  interrupts(); // enable interrupts
  if (bmp280Debug){
    Serial.print ("BMP280-");
    Serial.print (address - bmp280Addr);
    Serial.print (" register ");
    Serial.print (reg, HEX);
    Serial.print (" value ");
    Serial.println(data);
  } // end of if (bmp280Debug)
  return (data);
}  // end of void readRegister(byte reg)
// ********************************************

void bmp280::updateRegister(byte reg, byte value){    // function
  noInterrupts(); // disable interrupts
  Wire.beginTransmission(address);
  // Select register
  Wire.write(reg); // address of register
  Wire.write(value); // 
  // Stop I2C Transmission
  Wire.endTransmission(); // end of 0xF5 write
  byte error = Wire.endTransmission(); // end of write
  interrupts(); // enable interrupts
  if (error){
    Serial.print ("I2C error with address ");
    Serial.print (address, HEX); 
    Serial.print (" error code= "); 
    Serial.print (error); 
    Serial.print (" millis= "); 
    Serial.println (millis()); 
  } // end of if (error)
  if (bmp280Debug){
    Serial.print ("BMP280-");
    Serial.print (address - bmp280Addr);
    Serial.print (" register ");
    Serial.print (reg, HEX);
    Serial.print (" updated to ");
    Serial.println(value);
  } // end of if (bmp280Debug)
} // end of void bmp280::updateRegister(byte reg, byte value)
// ********************************************

byte bmp280::readF4Sleep(){
  // returns 6-bit variable from bit[7:2] of register 0xF4
  // read this value before begin function
  return readRegister(0xF4)>>2;
} // end of byte bmp280::readF4Sleep()
// ********************************************

byte bmp280::readF5Sleep(){
  // returns 6-bit variable from bit[7:2] of register 0xF5
  // read this value before begin function
  return readRegister(0xF5)>>2;
} // end of byte bmp280::readF5Sleep()
// ********************************************

void bmp280::updateF4Control(byte osrs_t, byte osrs_p, byte mode){
  // general function to update control register at 0xF4  
  // details of register 0xF4
  // mode[1:0] F4bits[1:0] 00=sleep, 01,10=forced, 11=normal 
  // osrs_p[2:0] F4bits[4:2] 000=skipped, 001=16bit, 010=17bit, 011=18bit, 100=19bit, 101,110,111=20 bit
  // osrs_t[2:0] F4bits[7:5] 000=skipped, 001=16bit, 010=17bit, 011=18bit, 100=19bit, 101,110,111=20 bit
  noInterrupts(); // disable interrupts
  Wire.beginTransmission(address); // start I2C transmission
  Wire.write(0xF4); // address of register to control oversampling and power mode
  byte F4value = (osrs_t&0x7)<<5 | (osrs_p&0x7)<<2 | (mode&0x3); // construct bit pattern
  Wire.write(F4value); // 
  Wire.endTransmission(); // Stop I2C Transmission, end of 0xF4 write
  interrupts(); // enable interrupts
  if (bmp280Debug){
    Serial.print ("BMP280-");
    Serial.print (address - bmp280Addr);
    Serial.print (" F4 control register updated to ");
    Serial.println(F4value,BIN);
  } // end of if (bmp280Debug)
}  // end of void updateF4Control(byte osrs_t, byte osrs_p, byte mode)
// ********************************************

void bmp280::updateF4Control16xNormal(){    // function
  updateF4Control(7,7,3); // temperature 20 bit, pressure 20 bit, mode Normal (continuous)
} // end of  void bmp280::updateF4Control16xNormal()
// ********************************************

void bmp280::updateF4Control16xSleep(){    // function
  updateF4Control(7,7,0); // temperature 20 bit, pressure 20 bit, mode Sleep
} // end of  void bmp280::updateF4Control16xSleep()
// ********************************************

void bmp280::updateF4ControlSleep(byte value){
  // sets register 0xF4 bits [1:0] to 00 (sleep) and bits [7:2] to supplied value
  // suggested use: write this value just before deepSleep begins
  updateF4Control(value>>3&0x07,value&0x07,0); // Sleep mode and 6 bits of value
} // end of void bmp280::updateF4ControlSleep(byte value)
// ********************************************

void bmp280::updateF5Config(byte t_sb, byte filter, byte spi3W_en){    // function
  // this configures register F5 for standby time, IIR filter constant and SPI/I2C use
  // details of register 0xF5
  // spi3w_en F5 bit [0] 0 use I2C, 1 enable SPI
  // F5 bit [1] not used
  // filter[2:0] F5 bits[4:2] 000=1(OFF), 001=2, 010=4, 011=8, 1xx=16
  // t_sb[2:0] F5 bits [7:5] 000=0.5ms, 001=62.5ms, 010=125ms, 011=250ms, 100=500ms, 101=1000ms, 110=2000ms, 111=4000ms
  
  noInterrupts(); // disable interrupts
  Wire.beginTransmission(address);
  // Select config register
  Wire.write(0xF5); // address of register to control intersample delay in Normal mode; IIR filter constant; and SPI enable
  byte F5value = (t_sb&0x07)<<5 | (filter&0x07)<<2 | (spi3W_en&0x03); // construct bit pattern
  Wire.write(F5value); // 
  // Stop I2C Transmission
  Wire.endTransmission(); // end of 0xF5 write
  interrupts(); // enable interrupts
  if (bmp280Debug){
    Serial.print ("BMP280-");
    Serial.print (address - bmp280Addr);
    Serial.print (" F5 config  register updated to ");
    Serial.println(F5value,BIN);
  } // end of if (bmp280Debug)
} // end of void bmp280::updateF5Contfig(byte t_sb, byte filter, byte spi3W_en)
// ********************************************

void bmp280::updateF5Config1msIIR16I2C(){    // function
  updateF5Config(0,0,0); // standby time 0.5ms, filter Off, I2C

} // end of void bmp280::updateF5Contfig1msIIR16I2C()
// ********************************************

void bmp280::updateF5ConfigSleep(byte value){
  // sets register 0xF5 bits [1:0] to 00 (I2C) and bits [7:2] to supplied value
  // suitable for saving value while BMP280 is sleeping
  // write this value just before deepSleep begins
  updateF5Config(value>>3&0x07,value&0x07,0); // I2C and 6 bits of value
} // end of void bmp280::updateF5ConfigSleep(byte value)
// ********************************************


