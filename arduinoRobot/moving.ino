/**
 * 앞으로 일정 거리(startDist)까지 주행 ↔
 * 주행 도중 앞 ToF가 thresh(mm) 이하로 떨어지면 즉시 정지 ↔
 * 정지 후 리프트를 올리고 끝.
 *
 * 모든 과정을 내부 while 루프에서 처리하므로
 * 호출자는 이 함수 한 줄만 쓰면 됩니다.
 */
void driveUntilFrontLE(float startDist_mm,
                     float thresh_mm)
{
  /* 1) 주행 시작 -------------------------------------- */
  startForward(startDist_mm);          // 거리만 주고 전진

  /* 2) 주행·센서 감시 --------------------------------- */
  while (motion.active) {              // 모션이 끝날 때까지
    controller.run();                  // odom thread
    motionUpdate();      

    float d = readDistanceSensorFront();
    if (d > 0 && d <= thresh_mm) {     // 센서 조건 만족
      prizm.setMotorSpeeds(0, 0);      // 즉시 정지
      motion.active = false;           // 루프 탈출
    }
  }

}


void driveUntilRightLE(float startDist_mm,
                     float thresh_mm)
{
  /* 1) 주행 시작 ────────────────── */
  startSide(startDist_mm);            // 좌우 이동

  /* 2) 주행·센서 감시 ───────────── */
  while (motion.active) {
    controller.run();                 // odom thread
    motionUpdate();                   // 가감속·엔코더

    float d = readDistanceSensorRight();
    if (d > 0 && d <= thresh_mm) {    // 센서 조건 만족
      prizm.setMotorSpeeds(0, 0);     // 즉시 정지
      motion.active = false;          // 루프 탈출
    }
  }
}


void driveUntilFrontGE(float startDist_mm,
                     float thresh_mm)
{
  /* 1) 주행 시작 -------------------------------------- */
  startForward(startDist_mm);          // 거리만 주고 전진

  /* 2) 주행·센서 감시 --------------------------------- */
  while (motion.active) {              // 모션이 끝날 때까지
    controller.run();                  // odom thread
    motionUpdate();      

    float d = readDistanceSensorFront();
    if (d > 0 && d >= thresh_mm) {     // 센서 조건 만족
      prizm.setMotorSpeeds(0, 0);      // 즉시 정지
      motion.active = false;           // 루프 탈출
    }
  }

}


void driveUntilRightGE(float startDist_mm,
                     float thresh_mm)
{
  /* 1) 주행 시작 ────────────────── */
  startSide(startDist_mm);            // 좌우 이동

  /* 2) 주행·센서 감시 ───────────── */
  while (motion.active) {
    controller.run();                 // odom thread
    motionUpdate();                   // 가감속·엔코더

    float d = readDistanceSensorRight();
    if (d > 0 && d >= thresh_mm) {    // 센서 조건 만족
      prizm.setMotorSpeeds(0, 0);     // 즉시 정지
      motion.active = false;          // 루프 탈출
    }
  }
}



void forward(int mm)             // 매개변수도 없음!
{
    startForward(mm);           // ① 주행 시작

    while (motion.active) {          // ② 끝날 때까지 대기
        controller.run();            // 쓰레드(오도메트리)
        motionUpdate();              // 가감속·엔코더 체크
                // PRIZM 내부 루프(필요시)
    }
}

void side(float mm)        // mm=±1000 만 넘기면 됨
{
    startSide(mm);                   // ① 주행 시작

    while (motion.active) {          // ② 끝날 때까지 대기
        controller.run();
        motionUpdate();

    }
}