// example of using library farmerkeith_BMP280.h for temperature only

#include <farmerkeith_BMP280.h>
#include <Wire.h>

bmp280 bmp0 ; // creates object bmp of type bmp280, base address

void setup() {
  Serial.begin(115200);
//  Serial.begin(9600); // use this if you get errors with the faster rate
  Serial.println("\nStart of basicTemperature sketch");
  Wire.begin();
  bmp0.begin();
} // end of void setup() 

void loop() {
  double temperature=bmp0.readTemperature (); // measure temperature
  Serial.print("Temperature = ");
  Serial.print(temperature,3); // print with 3 decimal places
  Serial.println( " degrees Celsius");
  delay(5000);
} // end of void loop() 

