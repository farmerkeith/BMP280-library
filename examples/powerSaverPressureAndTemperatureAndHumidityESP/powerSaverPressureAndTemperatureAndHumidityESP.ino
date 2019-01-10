// example of using library farmerkeith_BMP280.h for pressure and temperature and humidity
// using sleep mode to reduce energy consumed
// features:
// main microcontroller (ESP8266) and BME280 both sleep between measurements
// BME280 is used to store an event counter while sleeping
// BME280 is used in single shot mode ("forced mode")
// measurement read command is delayed, first by the calculated typical measurement time
// and then by repeatedly checking the "measuring" bit of status register (0xF3) until ready

const bool bme280Debug = 0; // controls serial printing for bmp280 transactions
// set to 1 to enable printing

#include <farmerkeith_BMP280.h>
#include <Wire.h>

bme280 bme0 (0, bme280Debug) ; // creates object bme0 of type bme280, base address
// turn on debugging printing to show configuration and data transactions

unsigned long eventCounter = 0; // to count measurement events
const long measurementInterval = 10000;  // measurement interval in ms
unsigned long timerMillis = 0;  // variable for duration calculation
byte osrs_t = 2; // setting for temperature oversampling
// No. of samples = 2 ^ (osrs_t-1) or 0 for osrs_t==0
byte osrs_p = 5; // setting for pressure oversampling
// No. of samples = 2 ^ (osrs_p-1) or 0 for osrs_p==0
byte osrs_h = 5; // setting for humidity oversampling
// No. of samples = 2 ^ (osrs_h-1) or 0 for osrs_h==0
byte temperatureSamples = 0, pressureSamples = 0, humiditySamples = 0;
long baseEventTime = 0 ; // to measure time within event

#include "PTHsleep.h" // tab file

void setup() {
  Serial.begin(115200);
  //  Serial.begin(9600); // use this if you get errors with the faster rate
  Serial.println("\nStart of powerSaverPressureAndTemperatureAndHumidity sketch");
  //  Serial.println("Printing pressure and temperature and humidity at 10 second intervals");
  baseEventTime = millis();
/*  Serial.print (" Time waking up=");
  Serial.print((float)millis() / 1000, 3);
  Serial.print (" Event elapsed Time=");
  Serial.println((float)(millis() - baseEventTime) / 1000, 3);*/
  Wire.begin(); // initialise I2C protocol
  Wire.setClock(400000L); // usual speed is 100,000

  eventCounter = recoverCounter();         // comment out if counter is not required
  temperatureSamples = pow(2, osrs_t - 1);
  pressureSamples = pow(2, osrs_p - 1);
  humiditySamples = pow(2, osrs_h -1);
  Serial.print ("Temperature samples=");
  Serial.println (temperatureSamples);
  Serial.print ("Pressure samples=");
  Serial.println (pressureSamples);
  Serial.print ("Humidity samples=");
  Serial.println (humiditySamples);


  //  bme0.begin(); // initialise BME280 for continuous measurements
  bme0.begin(osrs_t, osrs_p, 1, 0, 0, 0, osrs_h);
  // parameters are (osrs_t, osrs_p, mode, t_sb, filter, spi3W_en, osrs_h)
  // see BME280 data sheet for definitions
  // this is single shot mode with no filtering
  timerMillis = 1 + 2 * (temperatureSamples + pressureSamples);
  // typical delay is (1 + 2*samples)ms
  measurementEvent();
  eventCounter ++;
  timerMillis = millis();
  saveCounter(eventCounter);         // this also puts bme0 to sleep
  // bme0.updateF4Control16xSleep(); // use instead of saveCounter if counter is not required
  goToSleep();

} // end of void setup()

void loop() {
} // end of void loop()



