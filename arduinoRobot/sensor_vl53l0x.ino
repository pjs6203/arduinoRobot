//VL53L0X


void distanceSensor_init(){
  pinMode(distanceSensorFront_XSHUT, OUTPUT);
  pinMode(distanceSensorRight_XSHUT, OUTPUT);
  digitalWrite(distanceSensorFront_XSHUT, LOW);
  digitalWrite(distanceSensorRight_XSHUT, LOW);

  Serial.println(F("Both in reset mode...(pins are low)")); 
  Serial.println(F("VL53L0X Distance Sensor Starting..."));

  delay(50);
  digitalWrite(distanceSensorFront_XSHUT, HIGH);
  delay(50);

  if (!distanceSensorFront.init())//try to initilise the sensor
  {
      //Sensor does not respond within the timeout time
      Serial.println("distanceSensorFront is not responding.");
  }
  else
  {
    distanceSensorFront.setAddress(distanceSensorFront_ADDRESS);
    distanceSensorFront.setMeasurementTimingBudget(33000);
  }  

  delay(50);
  digitalWrite(distanceSensorRight_XSHUT, HIGH);
  delay(50);
  if (!distanceSensorRight.init())//try to initilise the sensor
  {
      //Sensor does not respond within the timeout time
      Serial.println("distanceSensorRight is not responding.");
  }
  else
  {
    distanceSensorRight.setAddress(distanceSensorRight_ADDRESS);
    distanceSensorRight.setMeasurementTimingBudget(33000);
  }  
}


//read VL53L0X Distance Sensor 
//2000[mm] 이상은 Out Of Range임
int16_t readDistanceSensorFront(){
  return distanceSensorFront.readRangeSingleMillimeters();
}

int16_t readDistanceSensorRight(){
  return distanceSensorRight.readRangeSingleMillimeters();
}


