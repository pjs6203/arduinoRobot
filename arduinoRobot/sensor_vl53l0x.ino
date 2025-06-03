//VL53L0X
#define distanceSensorFront_ADDRESS 0x30
#define distanceSensorRight_ADDRESS 0x31
#define distanceSensorFront_XSHUT 6
#define distanceSensorRight_XSHUT 7



void distanceSensor_init()
{
  //reset
  pinMode(distanceSensorFront_XSHUT, OUTPUT);
  pinMode(distanceSensorRight_XSHUT, OUTPUT);
  digitalWrite(distanceSensorFront_XSHUT, LOW);
  digitalWrite(distanceSensorRight_XSHUT, LOW);

  Serial.println(F("VL53L0X Distance Sensor Starting..."));

  //starting front sensor
  delay(50);
  digitalWrite(distanceSensorFront_XSHUT, HIGH);
  delay(50);

  if (!distanceSensorFront.init())//try to initilise the sensor
  {
      Serial.println("distanceSensorFront is not responding."); //Sensor does not respond within the timeout time
  }
  else
  {
    distanceSensorFront.setAddress(distanceSensorFront_ADDRESS);
    distanceSensorFront.setMeasurementTimingBudget(200000); //200ms
    distanceSensorFront.setSignalRateLimit(0.1);
    distanceSensorFront.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
    distanceSensorFront.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
  }  

  //starting right sensor
  delay(50);
  digitalWrite(distanceSensorRight_XSHUT, HIGH);
  delay(50);
  if (!distanceSensorRight.init())//try to initilise the sensor
  {
      Serial.println("distanceSensorRight is not responding."); //Sensor does not respond within the timeout time
  }
  else
  {
    distanceSensorRight.setAddress(distanceSensorRight_ADDRESS);
    distanceSensorRight.setMeasurementTimingBudget(200000); //200ms
    distanceSensorRight.setSignalRateLimit(0.1);
    distanceSensorRight.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
    distanceSensorRight.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
  }  
}


//read VL53L0X Distance Sensor
int16_t readDistanceSensorFront()
{
  if (!distanceSensorFront.timeoutOccurred()) 
  {
    return distanceSensorFront.readRangeSingleMillimeters();
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
    return distanceSensorRight.readRangeSingleMillimeters();
  }
  else
  {
    return 0;
  }
}


