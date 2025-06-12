// Huskylens Pro
// 팔레트 인식도 가능할까 ? ..

void printResult(HUSKYLENSResult result);

int16_t readQRdata() //인식 불가 시 0 리턴
{
  int data = 0;
  if(!huskylens.request()) return 0;
  if(!huskylens.isLearned()) return 0; 
  if(!huskylens.available()) return 0;

  HUSKYLENSResult result = huskylens.read();

  if (result.ID >= 1 && result.ID <= 6) return result.ID;
  return 0;
}


/*-----------------------------------------------------------
   QR / 팔레트 ID를 찾을 때까지 최장 timeout_ms 만큼 반복 스캔
   성공 → 감지된 ID(1~6)  │   실패(타임아웃·오류) → 0
  -----------------------------------------------------------*/
int16_t readQRdataTimeout(uint32_t timeout_ms)
{
    const uint32_t tStart = millis();          // 시작 시각

    while (millis() - tStart < timeout_ms)     // 타임아웃 전까지 반복
    {
        /* ① 요청 → 응답 대기                         */
        if (!huskylens.request())      continue;   // I²C 송신 실패
        if (!huskylens.isLearned())    continue;   // 학습 데이터 없음
        if (!huskylens.available())    continue;   // 프레임에 아무것도 안 보임

        /* ② 결과 읽기                               */
        HUSKYLENSResult r = huskylens.read();
        if (r.ID >= 1 && r.ID <= 6)
            return r.ID;                           // ★ 성공: 즉시 반환
    }

    return 0;  // ★ 타임아웃: 아무 것도 못 찾음
}


