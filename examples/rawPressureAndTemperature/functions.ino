// tab for functions used by rawPressureAndTemperature

void startToMeasure() {
  // Serial.print ("\n Time startToMeasure=");
  // Serial.println((float)millis() / 1000, 3);
  static byte startCounter = osrs_t*10; // set initial value
  startCounter++;
  if (startCounter == 60) startCounter = 10; // cycle 0 to 59
  // 10 measurements at each level of oversampling
  osrs_t = startCounter / 10; // cycle 0 to 5
  osrs_p = osrs_t;
  // Serial.print (" osrs_p,t=");
  // Serial.println(osrs_p);
  temperatureSamples = pow(2, osrs_t - 1);
  pressureSamples = pow(2, osrs_p - 1);
  bmp0.updateF4Control(osrs_t, osrs_p, 1); // commands BMP280 to start a measurement

  if (startCounter % 10 == 0) {
    Serial.print ("\nTemperature samples=");
    Serial.print (temperatureSamples);
    Serial.print (" Pressure samples=");
    Serial.println (pressureSamples);
  } // end of if (startCounter%10==0)

} // end of void startToMeasure()

void measurementEvent() {
  // Serial.print (" Time start measurementEvent=");
  // Serial.println((float)millis() / 1000, 3);
  while (bmp0.readRegister(0xF3) >> 3); // loop untl F3bit 3 ==0
  // ToDo check that measurement is finished before reading
  //  Serial.print (" Time end F3 check=");
  //  Serial.println((float)millis()/1000,3);

  long rawTemperature;
  long rawPressure = bmp0.readRawPressure (rawTemperature); // measure raw pressure and temperature
  double t_fine;
  double temperature = bmp0.calcTemperature(rawTemperature, t_fine);
  double pressure = bmp0.calcPressure (rawPressure, t_fine);
  //  Serial.print (" Time end measurement=");
  //  Serial.println((float)millis()/1000,3);
  Serial.print("Atm press, raw= ");
  Serial.print(rawPressure);
  //  Serial.print((rawPressure & 0xF0) >> 4, BIN);
  //  Serial.print(" ");
  //  Serial.print(rawPressure & 0xF, BIN);
  Serial.print(" hPa= ");
  Serial.print(pressure);
  Serial.print(" Temperature, raw= ");
  Serial.print(rawTemperature);
  //  Serial.print((rawTemperature & 0xF0) >> 4, BIN);
  //  Serial.print(" ");
  //  Serial.print(rawTemperature & 0xF, BIN);
  Serial.print(" degC= ");
  Serial.print(temperature);
  Serial.print (" Counter= ");
  Serial.print (eventCounter);
  Serial.print (" Time= ");
  Serial.println ((float)millis() / 1000, 3);

} // end of void measurementEvent()


