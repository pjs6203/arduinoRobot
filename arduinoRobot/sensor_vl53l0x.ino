
void distanceSensor_init()
{
  //reset
  pinMode(distanceSensorFront_XSHUT, OUTPUT);
  pinMode(distanceSensorRight_XSHUT, OUTPUT);

  digitalWrite(distanceSensorFront_XSHUT, LOW);
  digitalWrite(distanceSensorRight_XSHUT, LOW);

  Serial.println(F("XSHUT LOW"));
  Serial.println(F("VL53L0X Distance Sensor Starting..."));


  //starting front sensor
  delay(500);
  digitalWrite(distanceSensorFront_XSHUT, HIGH);
  digitalWrite(distanceSensorRight_XSHUT, LOW);
  Serial.println(F("Front Sensor XSHUT HIGH"));
  delay(500);


  Serial.print(F("Front Sensor Address : "));
  Serial.println(distanceSensorFront.getAddress());

  

  if (!distanceSensorFront.init())//try to initilise the sensor
  {
      Serial.println("distanceSensorFront is not responding."); //Sensor does not respond within the timeout time
      Serial.print(F("Front Sensor Address : "));
      Serial.println(distanceSensorFront.getAddress());
  }
  else
  {
    //distanceSensorFront.setAddress(distanceSensorFront_ADDRESS);
    distanceSensorFront.setMeasurementTimingBudget(100000);
    distanceSensorFront.setSignalRateLimit(0.1);
    distanceSensorFront.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
    distanceSensorFront.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
    distanceSensorFront.startContinuous(100);

    Serial.print(F("Front Sensor Address : "));
    Serial.println(distanceSensorFront.getAddress());
  }  



/*
  //starting right sensor
  delay(500);
  digitalWrite(distanceSensorFront_XSHUT, HIGH);
  digitalWrite(distanceSensorRight_XSHUT, HIGH);
  Serial.println(F("Right Sensor XSHUT HIGH"));
  delay(500);


  Serial.print(F("Right Sensor Address : "));
  Serial.println(distanceSensorRight.getAddress());


  if (!distanceSensorRight.init())//try to initilise the sensor
  {
      Serial.println("distanceSensorRight is not responding."); //Sensor does not respond within the timeout time
      Serial.print(F("Right Sensor Address : "));
      Serial.println(distanceSensorRight.getAddress());
  }
  else
  {
    distanceSensorRight.setAddress(distanceSensorRight_ADDRESS);
    distanceSensorRight.setMeasurementTimingBudget(100000);
    distanceSensorRight.setSignalRateLimit(0.1);
    distanceSensorRight.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
    distanceSensorRight.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
    distanceSensorRight.startContinuous(100);

    Serial.print(F("Right Sensor Address : "));
    Serial.println(distanceSensorRight.getAddress());
  }  

  digitalWrite(distanceSensorFront_XSHUT, HIGH);
  digitalWrite(distanceSensorRight_XSHUT, HIGH);

   delay(500);
*/
}


//read VL53L0X Distance Sensor
int16_t readDistanceSensorFront()
{
  if (!distanceSensorFront.timeoutOccurred()) 
  {
    return distanceSensorFront.readRangeContinuousMillimeters();
  }
  else
  {
    return 0;
  }
}


int16_t readDistanceSensorRight()
{
  if (!distanceSensorRight.timeoutOccurred()) 
  {
    return distanceSensorRight.readRangeContinuousMillimeters();
  }
  else
  {
    return 0;
  }
}


