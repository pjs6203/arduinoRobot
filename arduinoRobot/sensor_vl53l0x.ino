#define SHARP_RIGHT_PIN   A1 //A1

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
        return distanceSensorFront.readRangeContinuousMillimeters() - 240; //보정값 추가
    else
        return 8190;          // 오류·타임아웃 = 무한대 거리 취급
}

/* ── GP2Y0A21YK(10-80 cm) → 거리 [mm] 변환 ───────────────── */
int16_t readDistanceSensorRight()
{
    /* 1) ADC 읽기 */
    int raw = analogRead(SHARP_RIGHT_PIN);   // 0-1023

    /* 3) 원시값 → 전압[V] */
    float v = raw * (5.0f / 1023.0f);

    /* 4) 전압 ↔ 거리 회귀식  (GP2Y0A21 데이터시트)  
          d[cm] = 27.728 × V^-1.2045  */
    float dist_cm = 27.728f * powf(v, -1.2045f);

    /* 6) mm 단위로 반환 */
    return (int16_t)(dist_cm * 10.0f + 0.5f) - 150;   // 반올림, 보정값 추가
}