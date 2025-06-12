
void distanceSensor_init()
{

  Serial.println(F("VL53L0X Distance Sensor Starting..."));

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
