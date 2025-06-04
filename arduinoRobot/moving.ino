//이동 관련

/*
updateOdom()
  실시간으로 로봇 위치 추적

moveStraight(float mm, int spdDeg = 360)
  spdDeg(-720 ~ 720)의 속도로 거리 mm 만큼 이동

turnInPlace(float deg, int spdDeg = 300)
  spdDeg(-720 ~ 720)의 속도로 각도 deg 만큼 회전

moveArc(float R_mm, float theta_deg, int spdDeg = 300)
  spdDeg(-720 ~ 720)의 속도로 반지름 R_mm, theta_deg 만큼 회전

moveStraightUntil(int16_t speedDeg, ValueFn curValFn, CmdOp op, float refVal)
  speedDeg(-720 ~ 720)의 속도로 이동 중 특정 조건이 되면 멈춤

  ex:
    moveStraightUntil(720, readDistanceSensorFront(), LE, 100); // readDistanceSensorFront <= 100이 참이될 때까지 720 deg/s의 속도로 전진
    moveStraightUntil(720, [](){ return x_mm; }, LE, 3150.0); // X <= 3150 일 때까지 전진


  >= : GE(Greater than or Equal)
  <= : LE(Less than or Equal)
  > : GT(Greater Than)
  < : LT(Less Than)
  == : EQ(Equal)

setFirstPose()
  로봇의 초기 위치를 변수에 저장함
*/

//x, y, theta 업데이트
void updateOdom()
{
  long curL = prizm.readEncoderCount(1);
  long curR = prizm.readEncoderCount(2);
  long dnL  = curL - prevL;
  long dnR  = curR - prevR;
  prevL = curL;  prevR = curR;

  /* ① 그대로 rad 단위로 계산 */
  float dsL   = (PI*WHEEL_D) * dnL / TICKS_REV;
  float dsR   = (PI*WHEEL_D) * dnR / TICKS_REV;
  float ds    = 0.5f * (dsR + dsL);
  float dth_r = (dsR - dsL) / WHEEL_BASE;          // Δθ [rad]

  /* ② 삼각함수용 중간각(rad) */
  float midTh_r = (th_deg * PI/180.0f) + 0.5f * dth_r;

  /* ③ 누적 */
  noInterrupts();
  x_mm  += ds * cosf(midTh_r);
  y_mm  += ds * sinf(midTh_r);
  th_deg += dth_r * 180.0f / PI;                    // rad→deg

  if      (th_deg >  180) th_deg -= 360;
  else if (th_deg < -180) th_deg += 360;
  interrupts();
}


//거리, 각도를 목표로 하는 이동함수용
struct MotionTask 
{
  TaskMode mode = IDLE;
  long  tgtL = 0, tgtR = 0;       // 목표 엔코더 tick
  int16_t spL = 0,  spR = 0;      // 모터 속도 [deg/s]
} task;

/* mm·deg → 엔코더 tick 변환 */
inline long mm2tick(float mm)   { return lround(mm / MM_PER_TICK); }
inline long deg2tick(float deg) { return mm2tick( (PI*WHEEL_BASE)*deg/360.0 ); }

/*
//직선이동
void moveStraight(float mm, int spdDeg = 360)
{
  prizm.resetEncoders();   prevL = prevR = 0;

  long ticks  = mm2tick(mm);
  task.tgtL   = task.tgtR = ticks;
  task.spL    = task.spR  = (mm >= 0) ?  spdDeg : -spdDeg;
  task.mode   = STRAIGHT;

  prizm.setMotorSpeeds(task.spL, task.spR);   // 좌·우 동시에
}
*/

//지정한 거리(mm)를 이동, 목표에 도달할 때까지 함수 내에서 대기
void moveStraight(float mm, int spdDeg = 360) {
  // 1) 엔코더 값 리셋 및 상태 초기화
  prizm.resetEncoders();
  prevL = prevR = 0;

  // 2) 목표 틱 수 계산
  long ticks = mm2tick(mm);
  task.tgtL  = task.tgtR = ticks;
  task.spL   = task.spR  = (mm >= 0) ?  spdDeg : -spdDeg;
  task.mode  = STRAIGHT;

  // 3) 모터 속도 설정 → 이동 시작
  prizm.setMotorSpeeds(task.spL, task.spR);

  // 4) 이동이 완료될 때까지 루프를 돌며 확인
  while (true) {
    // 오도메트리(엔코더) 업데이트
    controller.run();

    // 현재 엔코더 카운트 읽기
    long curL = prizm.readEncoderCount(1);
    long curR = prizm.readEncoderCount(2);

    // 목표 틱에 도달했는지 확인
    bool doneL = (task.tgtL >= 0) ? (curL >= task.tgtL) : (curL <= task.tgtL);
    bool doneR = (task.tgtR >= 0) ? (curR >= task.tgtR) : (curR <= task.tgtR);

    if (doneL && doneR) {
      // 도달했으면 모터 정지, 상태 IDLE로 변경 후 반복 종료
      prizm.setMotorSpeeds(0, 0);
      task.mode = IDLE;
      break;
    }

    // 너무 빠르게 반복되지 않도록 약간의 delay
    delay(5);
  }
}


/*
//회전
void turnInPlace(float deg, int spdDeg = 300)
{
  prizm.resetEncoders();   prevL = prevR = 0;

  long t      = deg2tick(deg);       // +CCW
  task.tgtL   = -t;                  // L−, R+
  task.tgtR   =  t;
  task.spL    = (deg >= 0) ? -spdDeg :  spdDeg;
  task.spR    = -task.spL;
  task.mode   = TURN;

  prizm.setMotorSpeeds(task.spL, task.spR);
}
*/


//지정한 각도(deg)만큼 회전, 목표에 도달할 때까지 대기
void turnInPlace(float deg, int spdDeg = 300) {
  // 1) 엔코더 리셋 및 상태 초기화
  prizm.resetEncoders();
  prevL = prevR = 0;

  // 2) 목표 틱 수 계산 (+CCW 방향)
  long t       = deg2tick(deg);  // 엔코더 틱으로 변환
  task.tgtL    = -t;             // 왼쪽 바퀴는 반대 방향
  task.tgtR    =  t;
  task.spL     = (deg >= 0) ? -spdDeg : spdDeg;  // CCW면 왼쪽 음수, CW면 양수
  task.spR     = -task.spL;
  task.mode    = TURN;

  // 3) 모터 속도 설정 → 회전 시작
  prizm.setMotorSpeeds(task.spL, task.spR);

  // 4) 회전이 완료될 때까지 대기
  while (true) {
    controller.run();  // 오도메트리 및 스레드 업데이트

    // 현재 엔코더 값 읽기
    long curL = prizm.readEncoderCount(1);
    long curR = prizm.readEncoderCount(2);

    // 목표 틱에 도달했는지 확인
    bool doneL = (task.tgtL >= 0) ? (curL >= task.tgtL) : (curL <= task.tgtL);
    bool doneR = (task.tgtR >= 0) ? (curR >= task.tgtR) : (curR <= task.tgtR);

    if (doneL && doneR) {
      // 도달했으면 모터 정지, 상태 IDLE로 변경 후 반복 중단
      prizm.setMotorSpeeds(0, 0);
      task.mode = IDLE;
      break;
    }

    // 너무 빠르게 루프가 도는 것을 방지
    delay(5);
  }
}


/*
// 원호 주행
void moveArc(float R_mm, float theta_deg, int spdDeg = 300)
{
  prizm.resetEncoders();  prevL = prevR = 0;

  float arcIn_mm  = PI * fabs(R_mm) * fabs(theta_deg) / 180.0;
  long  tickIn    = mm2tick(arcIn_mm);
  long  tickOut   = lround(tickIn * (fabs(R_mm)+WHEEL_BASE) / fabs(R_mm));

  if (R_mm > 0) { task.tgtL =  tickIn;  task.tgtR =  tickOut; }
  else          { task.tgtL = -tickOut; task.tgtR = -tickIn;  }

  //속도 비례
  float ratio  = (float)abs(task.tgtR) / abs(task.tgtL);
  task.spL = (task.tgtL >= 0) ?  spdDeg : -spdDeg;
  task.spR = task.spL * ratio;
  if (task.spR >  720) task.spR =  720;
  if (task.spR < -720) task.spR = -720;

  task.mode = ARC;
  prizm.setMotorSpeeds(task.spL, task.spR);
}
*/


//지정한 반지름 R_mm, 각도 theta_deg만큼 원호 주행 후 대기
void moveArc(float R_mm, float theta_deg, int spdDeg = 300) {
  // 1) 엔코더 리셋 및 상태 초기화
  prizm.resetEncoders();
  prevL = prevR = 0;

  // 2) 원호 길이(arcIn_mm)와 대응 엔코더 틱 계산
  float arcIn_mm = PI * fabs(R_mm) * fabs(theta_deg) / 180.0f;
  long  tickIn   = mm2tick(arcIn_mm);
  long  tickOut  = lround(tickIn * (fabs(R_mm) + WHEEL_BASE) / fabs(R_mm));

  // 3) 좌, 우 바퀴 목표 틱과 속도 설정
  if (R_mm > 0) {
    // 좌회전: 왼쪽 바퀴가 안쪽 원, 오른쪽 바퀴가 바깥쪽 원
    task.tgtL =  tickIn;
    task.tgtR =  tickOut;
  } else {
    // 우회전: 각 반대
    task.tgtL = -tickOut;
    task.tgtR = -tickIn;
  }

  // 속도 비례로 계산
  float ratio = (float)abs(task.tgtR) / abs(task.tgtL);
  task.spL = (task.tgtL >= 0) ? spdDeg : -spdDeg;
  task.spR = lround(task.spL * ratio);
  if (task.spR >  720) task.spR =  720;
  if (task.spR < -720) task.spR = -720;

  task.mode = ARC;

  // 4) 모터 속도 설정 → 원호 주행 시작
  prizm.setMotorSpeeds(task.spL, task.spR);

  // 5) 목표 틱에 도달할 때까지 대기
  while (true) {
    controller.run();  // 오도메트리 및 스레드 업데이트

    long curL = prizm.readEncoderCount(1);
    long curR = prizm.readEncoderCount(2);

    // 좌/우 바퀴가 목표 틱에 도달했는지 확인
    bool doneL = (task.tgtL >= 0) ? (curL >= task.tgtL) : (curL <= task.tgtL);
    bool doneR = (task.tgtR >= 0) ? (curR >= task.tgtR) : (curR <= task.tgtR);

    if (doneL && doneR) {
      // 도달 시 모터 정지, 상태 IDLE로 변경 후 반복 종료
      prizm.setMotorSpeeds(0, 0);
      task.mode = IDLE;
      break;
    }

    delay(5);  // 너무 빠르게 반복되지 않도록 약간 지연
  }
}





void moveVelocity(float v_mm_s, float w_deg_s) {
  // w를 rad/s로 변환
  float w_rad_s = w_deg_s * PI / 180.0f;
  // 바퀴 반발 거리: 로봇 반지름 = WHEEL_BASE / 2
  float R = WHEEL_BASE / 2.0f;
  // 좌/우 바퀴의 선형 속도 [mm/s]
  float vL_mm_s = v_mm_s - w_rad_s * R;
  float vR_mm_s = v_mm_s + w_rad_s * R;
  // 바퀴 원주 [mm/rev]
  float wheelCirc = WHEEL_C;  // = PI * WHEEL_D
  // 바퀴 회전 속도 [rev/s]
  float revsL = vL_mm_s / wheelCirc;
  float revsR = vR_mm_s / wheelCirc;
  // 모터 속도 단위: deg/s (1 rev = 360 deg)
  int16_t spL = lround(revsL * 360.0f);
  int16_t spR = lround(revsR * 360.0f);
  // 엔코더 카운터 리셋 및 모터 제어 모드 업데이트
  prizm.resetEncoders();
  prevL = prevR = 0;
  task.mode = STRAIGHT;  // 이동 모드(값 상관없이 속도 제어만 함)
  prizm.setMotorSpeeds(spL, spR);
}


// 조건부 전/후진 함수
void moveStraightUntil(int16_t speedDeg, ValueFn curValFn, CmdOp op, float refVal)
{
    prizm.setMotorSpeeds(speedDeg, speedDeg);

    while (true) 
    {
        controller.run(); // 오도메트리, 센서 유지

        float cur = curValFn(); //매 주기 현재값 읽기
        bool stop = false;
        switch(op)
        {
          case GE: 
            stop = (cur >= refVal); 
            break;
          case LE: 
            stop = (cur <= refVal); 
            break;
          case GT: 
            stop = (cur > refVal); 
            break;
          case LT: 
            stop = (cur < refVal); 
            break;
          case EQ: 
            stop = (fabs(cur - refVal) < 1e-3); 
            break;
        }
        if (stop) break;
    }
    prizm.setMotorSpeeds(0,0);
}



void setFirstPose() // 로봇의 초기 각도는 90도에 맞추어야 함
{
  th_deg = 90;
  x_mm = world_X - (robotWidth / 2) - readDistanceSensorRight();
  y_mm = world_Y - (robotHeight / 2) - readDistanceSensorFront();
}

