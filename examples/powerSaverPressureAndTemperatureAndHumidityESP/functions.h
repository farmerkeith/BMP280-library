// tab file for functions used by powerSaverPressureAndTemperatureAndHumidity

void measurementEvent(){
/*  Serial.print (" Time start F3 check=");
  Serial.print((float)millis()/1000,3);
  Serial.print (" Event elapsed Time=");
  Serial.println((float)(millis()-baseEventTime)/1000,3);*/
  while (bme0.readRegister(0xF3)>>3); // loop until F3bit 3 ==0
  // ToDo check that measurement is finished before reading
/*  Serial.print (" Time end F3 check=");
  Serial.print ((float)millis()/1000,3);
  Serial.print (" Event elapsed Time=");
  Serial.println((float)(millis()-baseEventTime)/1000,3);
*/  
  double temperature, pressure;
  double humidity=bme0.readHumidity (temperature, pressure); // measure pressure, temperature and humidity
  float altitude = bme0.calcAltitude (pressure);
/*  Serial.print (" Time end measurement=");
  Serial.print ((float)millis()/1000,3);
  Serial.print (" Event elapsed Time=");
  Serial.println((float)(millis()-baseEventTime)/1000,3);*/
  Serial.print("Atm press = ");
  Serial.print(pressure,2); // print with 2 decimal places
  Serial.print(" hPa. Temperature = ");
  Serial.print(temperature,2); // print with 2 decimal places
  Serial.print( " deg C. Humidity = ");
  Serial.print(humidity,2); // print with 2 decimal places
  Serial.print( " %RH. Altitude = ");
  Serial.print(altitude,2); // print with 2 decimal places
  Serial.print (" m. Counter= ");
  Serial.println (eventCounter);

} // end of void measurementEvent()

// ----------recover Event counter----------------------------
int recoverCounter(){
  // read value of counter back from bmp0
  byte bme0F4value= bme0.readF4Sleep(); // 0 to 63
  byte bme0F5value= bme0.readF5Sleep(); // 0 to 63
  return bme0F5value*64+bme0F4value; // 0 to 4095
  if (bme280Debug){
    Serial.print ("bme0F4,F5values from bme0 Registers 0xF4,F5 =");
    Serial.print (bme0F4value);
    Serial.print (" ");
    Serial.println (bme0F5value);
  } // end of if (bme280Debug) 
} // end of void recoverCounter()

void saveCounter(int counter){
  // write value of counter into bme0
  bme0.updateF4ControlSleep(counter&0x3F); // store eventCounter
  bme0.updateF5ConfigSleep((counter/64)&0x3F); // store eventCounter
  // this also puts bme0 to sleep
  if (bme280Debug){
    Serial.print ("saved counter to bme0F4,F5 as ");
    Serial.print (counter&0x3F);
    Serial.print (", ");
    Serial.println ((counter/64)&0x3F);
  } // end of if (bme280Debug) 
} // end of void saveCounter()

void goToSleep(){
  // calculate required sleep time and go to sleep
  long sleepTime = measurementInterval - millis(); // in milliseconds
  if (sleepTime <100) sleepTime = 100; // set minimum sleep of 0.1 second
  
//  Serial.print ("This is report cycle No. ");
//  Serial.println(eventCounter);
  Serial.print ("Going to sleep now for ");
  Serial.print((float)sleepTime/1000, 3);
  Serial.println (" seconds.");
  Serial.print (" Time going to sleep=");
  Serial.print ((float)millis()/1000,3);
  Serial.print (" Event elapsed Time=");
  Serial.println((float)(millis()-baseEventTime)/1000,3);
  Serial.println();

ESP.deepSleep(sleepTime * 1000); // convert to microseconds 

} // end of void goToSleep()


