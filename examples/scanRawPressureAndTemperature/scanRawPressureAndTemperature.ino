// example of using library farmerkeith_BMP280.h for pressure and temperature
// features: 
  // BMP280 sleeps between measurements 
  // BMP280 is used in single shot mode ("forced mode")
  // measurement read command is delayed, first by the calculated typical measurement time
  // and then by repeatedly checking the "measuring" bit of status register (0xF3) until ready
  // raw temperature and raw pressure values are stepped across a +- range to show how
  // calcPressure function is affected by small changes in the raw values
  
const bool bmp280Debug=0; // controls serial printing for bmp280 transactions
 // set to 1 to enable printing

#include <farmerkeith_BMP280.h>
#include <Wire.h>

unsigned long eventCounter=0; // to count measurement events
const long measurementInterval = 10000;  // measurement interval in ms
unsigned long timerMillis = 0;  // variable for duration calculation
unsigned long bmp280Millis = 0;  // variable for measurement delay
byte osrs_t=1; // initial setting for temperature oversampling 
 // No. of samples = 2 ^ (osrs_t-1) or 0 for osrs_t==0
byte osrs_p=1; // initial setting for pressure oversampling 
 // No. of samples = 2 ^ (osrs_p-1) or 0 for osrs_p==0
byte temperatureSamples = 0, pressureSamples = 0;

bmp280 bmp0 (0,bmp280Debug) ; // creates object bmp of type bmp280, base address
// turn on debugging printing to show configuration and data transactions

void setup() {
  Serial.begin(115200);
//  Serial.begin(9600); // use this if you get errors with the faster rate
  Serial.println("\nStart of scanRawPressureAndTemperature sketch");
//  Serial.println("Printing pressure and temperature at 5 second intervals");
  Serial.print (" Time at start of setup=");
  Serial.println((float)millis()/1000,3);
  Wire.begin(); // initialise I2C protocol

  temperatureSamples=pow(2,osrs_t-1);
  pressureSamples = pow(2,osrs_p-1);
  Serial.print ("Temperature samples=");
  Serial.print (temperatureSamples);
  Serial.print (" Pressure samples=");
  Serial.println (pressureSamples);
  
  
//  bmp0.begin(); // initialise BMP280 for continuous measurements
  bmp0.begin(osrs_t, osrs_p, 1, 0, 0, 0); 
  // parameters are (osrs_t, osrs_p, mode, t_sb, filter, spi3W_en) 
  // see BMP280 data sheet for definitions
  // this is single shot mode with no filtering (filtering set to 2)
  while (millis()>timerMillis) timerMillis += measurementInterval ; // set next time to measure
  bmp280Millis = timerMillis + measurementInterval;
  
  Serial.print("Set timerMillis to ");
  Serial.println(timerMillis);
  
//  bmp280Millis = 1 + 2 * (temperatureSamples + pressureSamples); 
   // time to run measurementEvent 
   // typical delay is (1 + 2*samples)ms

} // end of void setup() 

void loop() {
  
  if (millis() - timerMillis >= measurementInterval){
    startToMeasure();
    timerMillis = millis();
    bmp280Millis = millis() + 1 + 2 * (temperatureSamples + pressureSamples); 
  } // end of if (millis() - timerMillis >= measurementInterval)

  if ((long)(millis() - bmp280Millis) >=0){
    measurementEvent();
    eventCounter ++;
    bmp280Millis += measurementInterval;
  } // end of if ((long)(millis() - bmp280Millis) >=0)

//  saveCounter(eventCounter); // this also puts bmp0 to sleep
//  goToSleep();

} // end of void loop() 

