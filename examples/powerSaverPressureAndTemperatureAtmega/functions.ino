// tab for functions used by powerSaverPressureAndTemperature

void measurementEvent(){
  Serial.print (" Time start F3 check=");
  Serial.print((float)millis()/1000,3);
  Serial.print (" Event elapsed Time=");
  Serial.println((float)(millis()-baseEventTime)/1000,3);
  while (bmp0.readRegister(0xF3)>>3); // loop untl F3bit 3 ==0
  // ToDo check that measurement is finished before reading
  Serial.print (" Time end F3 check=");
  Serial.print ((float)millis()/1000,3);
  Serial.print (" Event elapsed Time=");
  Serial.println((float)(millis()-baseEventTime)/1000,3);
  
  double temperature;
  double pressure=bmp0.readPressure (temperature); // measure pressure and temperature
  Serial.print (" Time end measurement=");
  Serial.print ((float)millis()/1000,3);
  Serial.print (" Event elapsed Time=");
  Serial.println((float)(millis()-baseEventTime)/1000,3);
  Serial.print("Atm press = ");
  Serial.print(pressure,4); // print with 4 decimal places
  Serial.print(" hPa. Temperature = ");
  Serial.print(temperature,2); // print with 2 decimal places
  Serial.print( " deg C");
  Serial.print (" Counter= ");
  Serial.println (eventCounter);

} // end of void measurementEvent()

// ----------recover Event counter----------------------------
int recoverCounter(){
  // read value of counter back from bmp1
  byte bmp0F4value= bmp0.readF4Sleep(); // 0 to 63
  byte bmp0F5value= bmp0.readF5Sleep(); // 0 to 63
  return bmp0F5value*64+bmp0F4value; // 0 to 4095
  if (bmp280Debug){
    Serial.print ("bmp0F4,F5values from bmp0 Registers 0xF4,F5 =");
    Serial.print (bmp0F4value);
    Serial.print (" ");
    Serial.println (bmp0F5value);
  } // end of if (bmp280Debug) 
} // end of void recoverCounter()

void saveCounter(int counter){
  // write value of counter into bmp0
  bmp0.updateF4ControlSleep(counter&0x3F); // store eventCounter
  bmp0.updateF5ConfigSleep((counter/64)&0x3F); // store eventCounter
  // this also puts bmp0 to sleep
  if (bmp280Debug){
    Serial.print ("saved counter to bmp0F4,F5 as ");
    Serial.print (counter&0x3F);
    Serial.print (", ");
    Serial.println ((counter/64)&0x3F);
  } // end of if (bmp280Debug) 
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


