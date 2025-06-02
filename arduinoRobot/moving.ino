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
  speedDeg(-720 ~ 720)의 속도로 특정 조건에 도달 시 멈춤

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
struct MotionTask {
  TaskMode mode = IDLE;
  long  tgtL = 0, tgtR = 0;       // 목표 엔코더 tick
  int16_t spL = 0,  spR = 0;      // 모터 속도 [deg/s]
} task;

/* mm·deg → 엔코더 tick 변환 */
inline long mm2tick(float mm)   { return lround(mm / MM_PER_TICK); }
inline long deg2tick(float deg) { return mm2tick( (PI*WHEEL_BASE)*deg/360.0 ); }


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


// 원호 주행
void moveArc(float R_mm, float theta_deg, int spdDeg = 300)
/* R_mm>0 : 좌회전,  R_mm<0 : 우회전 */
{
  prizm.resetEncoders();  prevL = prevR = 0;

  float arcIn_mm  = PI * fabs(R_mm) * fabs(theta_deg) / 180.0;
  long  tickIn    = mm2tick(arcIn_mm);
  long  tickOut   = lround(tickIn * (fabs(R_mm)+WHEEL_BASE) / fabs(R_mm));

  if (R_mm > 0) { task.tgtL =  tickIn;  task.tgtR =  tickOut; }
  else          { task.tgtL = -tickOut; task.tgtR = -tickIn;  }

  /* 속도 비례(간단) */
  float ratio  = (float)abs(task.tgtR) / abs(task.tgtL);
  task.spL = (task.tgtL >= 0) ?  spdDeg : -spdDeg;
  task.spR = task.spL * ratio;
  if (task.spR >  720) task.spR =  720;
  if (task.spR < -720) task.spR = -720;

  task.mode = ARC;
  prizm.setMotorSpeeds(task.spL, task.spR);
}


// 조건부 전/후진 함수
void moveStraightUntil(int16_t speedDeg, ValueFn curValFn, CmdOp op, float refVal)
{
    prizm.setMotorSpeeds(speedDeg, speedDeg);

    while (true) {
        controller.run(); // 오도메트리, 센서 유지

        float cur = curValFn(); //매 주기 현재값 읽기
        bool stop = false;
        switch(op){
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


void setFirstPose(){ // 초기 theta값은 90도로 가정
  th_deg = 90;
  x_mm = world_X - readDistanceSensorRight();
  y_mm = world_Y - readDistanceSensorFront();
}

