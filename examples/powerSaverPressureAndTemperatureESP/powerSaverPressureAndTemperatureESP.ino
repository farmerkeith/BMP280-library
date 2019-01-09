// example of using library farmerkeith_BMP280.h for pressure and temperature
// using sleep mode to reduce energy consumed
// features:
// main microcontroller (ESP8266) and BMP280 both sleep between measurements
// BMP280 is used to store an event counter while sleeping
// BMP280 is used in single shot mode ("forced mode")
// measurement read command is delayed, first by the calculated typical measurement time
// and then by repeatedly checking the "measuring" bit of status register (0xF3) until ready

const bool bmp280Debug = 1; // controls serial printing for bmp280 transactions
// set to 1 to enable printing

#include <farmerkeith_BMP280.h>
#include <Wire.h>

bmp280 bmp0 (0, bmp280Debug) ; // creates object bmp of type bmp280, base address
// turn on debugging printing to show configuration and data transactions

unsigned long eventCounter = 0; // to count measurement events
const long measurementInterval = 5000;  // measurement interval in ms
unsigned long timerMillis = 0;  // variable for duration calculation
byte osrs_t = 2; // setting for temperature oversampling
// No. of samples = 2 ^ (osrs_t-1) or 0 for osrs_t==0
byte osrs_p = 5; // setting for pressure oversampling
// No. of samples = 2 ^ (osrs_p-1) or 0 for osrs_p==0
byte temperatureSamples = 0, pressureSamples = 0;
long baseEventTime = 0 ; // to measure time within event
#include "functions.h" // tab file

void setup() {
  Serial.begin(115200);
  //  Serial.begin(9600); // use this if you get errors with the faster rate
  Serial.println("\nStart of powerSaverPressureAndTemperature sketch");
  //  Serial.println("Printing pressure and temperature at 5 second intervals");
  baseEventTime = millis();
/*  Serial.print (" Time waking up=");
  Serial.print((float)millis() / 1000, 3);
  Serial.print (" Event elapsed Time=");
  Serial.println((float)(millis() - baseEventTime) / 1000, 3);*/
  Wire.begin(); // initialise I2C protocol
  Wire.setClock(400000L); // usual speed is 100,000

  eventCounter = recoverCounter();
  temperatureSamples = pow(2, osrs_t - 1);
  pressureSamples = pow(2, osrs_p - 1);
  Serial.print ("Temperature samples=");
  Serial.println (temperatureSamples);
  Serial.print ("Pressure samples=");
  Serial.println (pressureSamples);


  //  bmp0.begin(); // initialise BMP280 for continuous measurements
  bmp0.begin(osrs_t, osrs_p, 1, 0, 0, 0);
  // parameters are (osrs_t, osrs_p, mode, t_sb, filter, spi3W_en)
  // see BMP280 data sheet for definitions
  // this is single shot mode with no filtering
  timerMillis = 1 + 2 * (temperatureSamples + pressureSamples);
  // typical delay is (1 + 2*samples)ms
  measurementEvent();
  eventCounter ++;
  timerMillis = millis();
  saveCounter(eventCounter); // this also puts bmp0 to sleep
  goToSleep();

} // end of void setup()

void loop() {
} // end of void loop()

