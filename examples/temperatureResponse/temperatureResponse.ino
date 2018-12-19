// Response of BMP280 temperature to intensity of measurements

#include <farmerkeith_BMP280.h>
#include <Wire.h>

bmp280 bmp0 ; // creates object bmp of type bmp280, base address
uint32_t cycleMillis=0; // variable to track time for cycles
const uint32_t cyclePeriod = 1000; // ms, period of cycle
byte osrs_p =1; // variable to control no. of pressure measurements
int reportCycle = 10;

void setup() {
  Serial.begin(115200);
  Serial.println("\nStart of temperatureResponse sketch");
  Serial.println("Printing temperature sequence for multiple measurements with varying pressure measurements");
  Wire.begin(); // initialise I2C protocol
  Wire.setClock(400000L); // usual speed is 100,000
  bmp0.begin(1,0,3,0,0,0); // initialise BMP280 for continuous measurements
} // end of void setup() 

void loop() {
  if (millis() - cycleMillis >= cyclePeriod) { // true if time has expired
    runReport();
    cycleMillis = millis(); // set current time
  }
} // end of void loop() 

void runReport(){ 
  
  if (reportCycle <23) reportCycle++; else reportCycle = 0; // cycles 0 through 59
  osrs_p = reportCycle/4;
//  if (reportCycle <59) reportCycle++; else reportCycle = 0; // cycles 0 through 59
//  osrs_p = reportCycle/10;
//  if (osrs_p <5) osrs_p++; else osrs_p = 0; // cycles 0 through 5
  Serial.print (" reportCycle =");
  Serial.print (reportCycle);
  Serial.print ("   ");
  Serial.println (reportCycle);
  Serial.print (" osrs_p= ");
  Serial.print (osrs_p);
  Serial.print ("   ");
  Serial.println (osrs_p);
  bmp0.updateF4Control(1, osrs_p, 3); //  1 temperature, osrs_p pressure, continuous
//  bmp0.updateF5Config(0,0,0); // 0.5ms standby, IIR filter OFF, I2C

  delay(50); // to allow new F4 to take effect
  double temperature;
  int checkCounter = 0;
  uint32_t delayTime = 0;
  
  for (int i=0; i<5; i++){
    delayTime = micros();
    checkCounter = 0;
    while (bmp0.readRegister(0xF3)>>3){ // loop until F3bit 3 ==0
      checkCounter ++;
//      delay(1); 
    }
    delayTime = micros()-delayTime;
    double pressure=bmp0.readPressure (temperature); // measure pressure and temperature
    Serial.print("time ");
    Serial.print((float)millis()/1000,3);
    Serial.print(" Pressure ");
    Serial.print(pressure,4); // print with 4 decimal places
    Serial.print(" hPa. Temperature ");
    Serial.print(temperature,4); // print with 4 decimal places
    Serial.print( " degrees Celsius");
    Serial.print(" checkCounter ");
    Serial.print (checkCounter);
    Serial.print(" delayTime ");
    Serial.print ((float)delayTime/1000);
    Serial.print(" osrs_p ");
    Serial.println (osrs_p);
  }
}

