// example of using library farmerkeith_BMP280.h for pressure only

#include <farmerkeith_BMP280.h>
#include <Wire.h>

bmp280 bmp0 ; // creates object bmp of type bmp280, base address

void setup() {
  Serial.begin(115200);
//  Serial.begin(9600); // use this if you get errors with the faster rate
  Serial.println("\nStart of basicPressure sketch");
  Wire.begin();
  bmp0.begin();
} // end of void setup() 

void loop() {
  double pressure=bmp0.readPressure (); // measure pressure
  Serial.print("Atmospheric pressure = ");
  Serial.print(pressure,4); // print with 4 decimal places
  Serial.println( " hPa");
  delay(5000);
} // end of void loop() 

