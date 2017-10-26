// example of using library farmerkeith_BMP280.h for pressure and temperature

#include <farmerkeith_BMP280.h>
#include <Wire.h>

bmp280 bmp0 ; // creates object bmp of type bmp280, base address

void setup() {
  Serial.begin(115200);
//  Serial.begin(9600); // use this if you get errors with the faster rate
  Serial.println("\nStart of basicPressureAndTemperature sketch");
  Serial.println("Printing pressure and temperature at 5 second intervals");
  Wire.begin(); // initialise I2C protocol
  bmp0.begin(); // initialise BMP280 for continuous measurements
} // end of void setup() 

void loop() {
  double temperature;
  double pressure=bmp0.readPressure (temperature); // measure pressure and temperature
  Serial.print("Atmospheric pressure = ");
  Serial.print(pressure,4); // print with 4 decimal places
  Serial.print(" hPa. Temperature = ");
  Serial.print(temperature,2); // print with 2 decimal places
  Serial.println( " degrees Celsius");
  delay(5000);
} // end of void loop() 

