// example of using library farmerkeith_BMP280.h for humidity, temperature and pressure
// Requires a BME280 sensor to work.

#include <farmerkeith_BMP280.h>
#include <Wire.h>

bme280 bme0 ; // creates object bme0 of type bme280, base address

void setup() {
  Serial.begin(115200);
//  Serial.begin(9600); // use this if you get errors with the faster rate
  Serial.println("\nStart of basicHumidityAndTemperatureAndPressure sketch");
  Serial.println("Printing humidity, temperature and pressure at 5 second intervals");
  Wire.begin(); // initialise I2C protocol
  bme0.begin(); // initialise BME280 for continuous measurements
} // end of void setup() 

void loop() {
  double temperature, pressure;
  double humidity=bme0.readHumidity (temperature, pressure); // measure temperature and pressure
  Serial.print("Humidity = ");
  Serial.print(humidity,4); // print with 4 decimal places
  Serial.print("%RH. Atmospheric pressure = ");
  Serial.print(pressure,4); // print with 4 decimal places
  Serial.print(" hPa. Temperature = ");
  Serial.print(temperature,2); // print with 2 decimal places
  Serial.println( " degrees Celsius");
  delay(5000);
} // end of void loop() 

