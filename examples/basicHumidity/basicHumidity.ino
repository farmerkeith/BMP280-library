// example of using library farmerkeith_BMP280.h for humidity only

#include <farmerkeith_BMP280.h>
#include <Wire.h>

bme280 bme0(0,1) ; // creates object bme0 of type bme280, base address

void setup() {
  Serial.begin(115200);
//  Serial.begin(9600); // use this if you get errors with the faster rate
  Serial.println("\nStart of basicHumidity sketch");
  Wire.begin();
  bme0.begin();
} // end of void setup() 

void loop() {
  double humidity =bme0.readHumidity(); // measure humidity
  Serial.print("Humidity = ");
  Serial.print(humidity,4); // print with 4 decimal places
  Serial.println( " %Relative Humidity");
  delay(5000);
} // end of void loop() 

