/*
[moving.ino]

updateOdom() //로봇 위치 추적
moveStraight(float mm, int spdDeg = 360) //전후진
turnInPlace(float deg, int spdDeg = 300) //회전
moveArc(float R_mm, float theta_deg, int spdDeg = 300) //원호주행
moveStraightUntil(int16_t speedDeg, ValueFn curValFn, CmdOp op, float refVal) //전후진 중 특정 조건 도달 시 정지
setFirstPose() //로봇의 초기 위치 설정
*/

/*
[sensor_vl53l0x.ino]

distanceSensor_init() //vl53l0x I2C 주소 재설정 및 파라미터 설정
readDistanceSensorFront() //전방 거리센서값 읽기
readDistanceSensorRight() //우측 거리센서값 읽기
*/

/*
[sensor_huskylens.ino]

readQRdata() //QR인식
*/

/*
[I2C Address]
PRIZM : 0x01 ~ 0x06 (prizm programming guide 134P 참고)
frontToF (VL53L0X) : 0x29 -> 0x30
rightToF (VL53L0X) : 0x29 -> 0x31
HUSKYLENS : 0x32
*/

/*
1구역 : 350, 1200
2구역 : 1050, 1200
3구역 : 350, 200
4구역 : 1050, 200
5구역 : 2600, 1200
6구역 : 3050, 1200
시작/종료 구역 : 3150, 250
*/

/*
1. Huskylens QR, 팔레트 학습시키기 (참고 : https://blog.naver.com/icbanq/223079420765)
2. setMotorInvert 하면 엔코더 값도 음수로 들어오는지 확인해야함 >> 음수로 들어옴
3. 휠 간 거리 측정
4. 센서 거리 측정
5. prizm.PrizmEnd 주석풀기
6. 거리 오차 테스트
7. 가속도 ?
8. ToF XSHUT 핀들 프리즘에 맞게 다시 써야함
*/

//moveStraightUntil
typedef float (*ValueFn)(); // 예: readFrontToF(), [](){ return x_mm; }

//moveStraight, turnInPlace, moveArc, moveStraightUntil
enum TaskMode { IDLE, STRAIGHT, TURN, ARC };
enum CmdOp { GE, LE, GT, LT, EQ };
/*
  GE : >=
  LE : <=
  GT : >
  LT : <
  EQ : ==
*/

#include <VL53L0X.h> //pololu
#include <HUSKYLENS.h>
#include <PRIZM.h>
#include <Wire.h>
#include <Thread.h> //ivanseidel
#include <ThreadController.h> //ivanseidel





//VL53L0X
#define distanceSensorFront_ADDRESS 0x30
#define distanceSensorRight_ADDRESS 0x31
#define distanceSensorFront_XSHUT 9 //D2-3
#define distanceSensorRight_XSHUT 2 //D2-4

// objects
PRIZM prizm;
EXPANSION exc;
ThreadController controller;
Thread odomThread;

HUSKYLENS huskylens;


VL53L0X distanceSensorFront;
VL53L0X distanceSensorRight;

//state
volatile float x_mm = 0, y_mm = 0, th_deg = 0;
long prevL=0, prevR=0;

/* ─── 하드웨어 파라미터 ───────────────────── */
const float WHEEL_D   = 100;               // 휠 지름 [mm]
const float WHEEL_C   = PI * WHEEL_D;        // 휠 거리 [mm]
const float WHEEL_BASE= 100.0;               // 휠 간 거리 [mm] 확인 후 바꿔야함
const int   TICKS_REV = 1440;                // 1회전 당 엔코더 틱 [ticks/rev]
const float MM_PER_TICK = WHEEL_C / TICKS_REV;
const int   world_X = 3400;
const int   world_Y = 1400;
const int   robotHeight = 0; //세로길이, 바꿔야함
const int   robotWidth = 0; //가로길이, 바꿔야함



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

void moveStraight(float mm,
                  int   spdDeg    = 360,
                  float rampRatio = 0.20f)   // «가감속 강도» 사용자가 조정
{
  /* ── 0) 파라미터 보정 ───────────────────────────────────────── */
  rampRatio = constrain(rampRatio, 0.0f, 0.45f);   // 과도값 방지
  const int MIN_SPD = 60;                          // 멈추지 않을 최소 속도

  /* ── 1) 엔코더 리셋   및 상태 초기화 ───────────────────────── */
  prizm.resetEncoders();
  delay(10);
  prevL = prevR = 0;

  /* ── 2) 목표 틱 계산 ───────────────────────────────────────── */
  long tgtTicks  = mm2tick(mm);          // +앞 / –뒤
  long absTicks  = labs(tgtTicks);
  long rampTicks = (long)(absTicks * rampRatio);
  int  dir       = (tgtTicks >= 0) ? +1 : -1;

  task.tgtL = task.tgtR = tgtTicks;
  task.mode = STRAIGHT;

  /* ── 3) 초기 속도 설정(정지 출발) ──────────────────────────── */
  prizm.setMotorSpeeds(0, 0);

  /* ── 4) 메인 루프 : 가감속 제어 ───────────────────────────── */
  while (true) {
    controller.run();                       // 오도메트리 갱신
    long curL =  prizm.readEncoderCount(1);
    long curR = -prizm.readEncoderCount(2); // 모터 2는 반대 방향

    long traveled   = (labs(curL) + labs(curR)) >> 1; // 평균 이동 틱
    long remaining  = absTicks - traveled;

    /* 구간별 목표 속도 계산 – 선형 보간 */
    int targetSpd;

    if (traveled < rampTicks) {                                   // ▶ 가속
      float k = (float)traveled / (float)rampTicks;               // 0→1
      targetSpd = MIN_SPD + (int)((spdDeg - MIN_SPD) * k);
    }
    else if (remaining < rampTicks) {                             // ▶ 감속
      float k = (float)remaining / (float)rampTicks;              // 1→0
      targetSpd = MIN_SPD + (int)((spdDeg - MIN_SPD) * k);
    }
    else {                                                        // ▶ 정속
      targetSpd = spdDeg;
    }

    prizm.setMotorSpeeds(dir * targetSpd, dir * targetSpd);

    /* 디버그 출력(선택) */
    Serial.print("curL:");  Serial.print(curL);
    Serial.print(" tgtL:"); Serial.print(task.tgtL);
    Serial.print(" curR:"); Serial.print(curR);
    Serial.print(" tgtR:"); Serial.println(task.tgtR);

    /* 목표 도달 판정 */
    bool doneL = (tgtTicks >= 0) ? (curL >= tgtTicks) : (curL <= tgtTicks);
    bool doneR = (tgtTicks >= 0) ? (curR >= tgtTicks) : (curR <= tgtTicks);
    if (doneL && doneR) break;

    delay(5);   // 루프 주기 ≈ 5 ms (200 Hz)
  }

  /* ── 5) 정지 및 모드 리셋 ─────────────────────────────────── */
  prizm.setMotorSpeeds(0, 0);
  task.mode = IDLE;
}

// 지정한 각도(deg)만큼 회전(가감속 프로파일 적용), 목표에 도달할 때까지 블로킹
void turnInPlace(float deg,
                         int   spdDeg    = 300,
                         float rampRatio = 0.20f) 
{
  // ── 0) 파라미터 보정 ─────────────────────────────────────────
  rampRatio = constrain(rampRatio, 0.0f, 0.45f);   // 과도값 방지
  const int MIN_SPD = 60;                          // 멈추지 않을 최소 속도

  // ── 1) 엔코더 리셋 및 상태 초기화 ───────────────────────────
  prizm.resetEncoders();
  delay(10);
  prevL = prevR = 0;

  // ── 2) 목표 틱 계산 (deg → tick) ─────────────────────────────
  long tgtTicks  = deg2tick(deg);          // +CCW / –CW
  long absTicks  = labs(tgtTicks);
  long rampTicks = (long)(absTicks * rampRatio);
  int  dir       = (tgtTicks >= 0) ? +1 : -1;   // +1 = CCW, -1 = CW

  task.tgtL = -tgtTicks;  // 왼쪽 엔코더는 deg 신호의 부호 반대
  task.tgtR =  tgtTicks;
  task.mode = TURN;

  // ── 3) 초기 속도 설정(정지 출발) ─────────────────────────────
  prizm.setMotorSpeeds(0, 0);

  // ── 4) 메인 루프: 가감속 제어 ────────────────────────────────
  while (true) {
    controller.run();  // 오도메트리 갱신

    long curL = prizm.readEncoderCount(1);
    long curR = -prizm.readEncoderCount(2);

    // 회전 시 이동 틱 계산: 두 바퀴 틱 절댓값의 평균
    long traveled  = (labs(curL) + labs(curR)) >> 1;
    long remaining = absTicks - traveled;

    // 각 구간별 목표 속도 계산 (선형 보간)
    int targetSpd;
    if (traveled < rampTicks) {  // ▶ 가속 구간
      float k = (float)traveled / (float)rampTicks;  // 0 → 1
      targetSpd = MIN_SPD + (int)((spdDeg - MIN_SPD) * k);
    }
    else if (remaining < rampTicks) {  // ▶ 감속 구간
      float k = (float)remaining / (float)rampTicks;  // 1 → 0
      targetSpd = MIN_SPD + (int)((spdDeg - MIN_SPD) * k);
    }
    else {  // ▶ 정속 구간
      targetSpd = spdDeg;
    }

    // 방향에 따라 모터 속도 설정
    // CCW 회전: 왼쪽 바퀴는 음수, 오른쪽 바퀴는 양수
    int spL = -dir * targetSpd;
    int spR =  dir * targetSpd;
    prizm.setMotorSpeeds(spL, spR);

    /* (선택) 디버그 출력
    Serial.print("curL:");  Serial.print(curL);
    Serial.print(" tgtL:"); Serial.print(task.tgtL);
    Serial.print(" curR:"); Serial.print(curR);
    Serial.print(" tgtR:"); Serial.println(task.tgtR);
    */

    // 목표 틱 도달 판정
    bool doneL = (tgtTicks >= 0) ? (curL <= task.tgtL) : (curL >= task.tgtL);
    bool doneR = (tgtTicks >= 0) ? (curR >= task.tgtR) : (curR <= task.tgtR);
    // ────── 주의: 왼쪽 엔코더 목표(tgtL)는 -tgtTicks이므로 비교 부호 반대 ──────

    if (doneL && doneR) {
      // 도달 시 모터 정지, 상태 IDLE로 변경 후 루프 탈출
      prizm.setMotorSpeeds(0, 0);
      task.mode = IDLE;
      break;
    }

    delay(5);  // 루프 주기 약 5 ms (200 Hz)
  }
}


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



void moveStraightUntil(int16_t maxSpdDeg, ValueFn curValFn, CmdOp op, float refVal, float rampRatio = 0.20f)
{
  // 0) 파라미터 보정
  rampRatio   = constrain(rampRatio, 0.0f, 0.45f);
  const int MIN_SPD = 60; // 최소 속도(멈지 않도록)

  // 1) 엔코더 리셋
  prizm.resetEncoders();
  prevL = prevR = 0;

  // 2) 모터 가감속 프로파일 준비
  prizm.setMotorSpeeds(0, 0); 
  delay(10);

  // 3) 메인 루프: 조건 검사 + 가감속 제어
  while (true)
  {
    controller.run(); // 오도메트리 등 업데이트

    // 3-1) 센서값 읽어서 조건 검사
    float cur = curValFn();
    bool stop = false;
    switch(op)
    {
      case GE: stop = (cur >=  refVal); break;
      case LE: stop = (cur <=  refVal); break;
      case GT: stop = (cur >   refVal); break;
      case LT: stop = (cur <   refVal); break;
      case EQ: stop = (fabs(cur - refVal) < 1e-3f); break;
    }
    if (stop)
      break;

    // 3-2) 엔코더 틱으로부터 운동량 계산 (가감속을 위해)
    long curL = prizm.readEncoderCount(1);
    long curR = -prizm.readEncoderCount(2);
    long traveledTicks = (labs(curL) + labs(curR)) >> 1;

    // (참고) 전체 이동 거리가 명확치 않으므로, 예를 들어
    // refVal이 “거리”라면 dist2ticks(refVal)로 환산 후 absTicks를 계산해야 합니다.
    // 여기서는 편의상 absTicks 대신 traveledTicks를 사용해, 움직인 거리가 rampTicks 이상일 때
    // 감속 구간으로 진입하는 구조만 보여드립니다.

    long absTicks      = traveledTicks;      // 실제로는 “목표 거리(tgtDist)를 틱으로 환산”해야 함
    long rampTicks     = (long)(absTicks * rampRatio);
    long remaining     = absTicks - traveledTicks;

    // 3-3) 속도 목표 계산(선형 보간)
    int targetSpd;
    if (traveledTicks < rampTicks) {
      // ▶ 가속 구간
      float k = (float)traveledTicks / (float)rampTicks; // 0→1
      targetSpd = MIN_SPD + (int)((maxSpdDeg - MIN_SPD) * k);
    }
    else if (remaining < rampTicks) {
      // ▶ 감속 구간
      float k = (float)remaining / (float)rampTicks;    // 1→0
      targetSpd = MIN_SPD + (int)((maxSpdDeg - MIN_SPD) * k);
    }
    else {
      // ▶ 정속 구간
      targetSpd = maxSpdDeg;
    }

    // 모터에 속도 명령
    prizm.setMotorSpeeds(targetSpd, targetSpd);

    delay(5); // 200 Hz 정도로 루프 동작
  }

  // 4) 조건 만족 시 정지
  prizm.setMotorSpeeds(0, 0);
}




void setFirstPose() // 로봇의 초기 각도는 90도에 맞추어야 함
{
  th_deg = 90;
  //x_mm = world_X - (robotWidth / 2) - readDistanceSensorRight();
  y_mm = world_Y - (robotHeight / 2) - readDistanceSensorFront();
}



void setup() 
{
  //Serial Port
  Serial.begin(115200);

  //PRIZM Board
  prizm.PrizmBegin();
  prizm.setMotorInvert(2,1); //2번 모터를 반

  //prizm.resetEncoders();

  //I2C
  Wire.begin();

  //Huskylens
  huskylens.begin(Wire);

  //VL53L0X
  distanceSensor_init(); 

        



  //odometry thread
  odomThread.onRun(updateOdom);
  odomThread.setInterval(20);   // 50 Hz
  controller.add(&odomThread);
}

void loop() 
{
  controller.run(); //odometry update

  while(true) //거리센서 테스트용
  {
    Serial.print(F("Front Distance [mm] : "));
    Serial.print(readDistanceSensorFront());
    delay(5);
    Serial.println(F(" "));
    //Serial.print(F("Right Distance [mm] : "));
    //Serial.println(readDistanceSensorRight()); 
    delay(50);
  }
  

  // 테스트 동작을 해보아요
  // moveStraight(mm, speed(-720 ~ 720))
  // turnInPlace(deg, deg/s(-720 ~ 720))
  
  moveStraight(1000, 360, 0.35);
  moveStraightUntil(360, readDistanceSensorFront(), LE, 500, 0.2);

/*
  // 1) 엔코더 값 리셋 및 상태 초기화

  delay(10); // 리셋이 실제로 반영되도록 약간 대기
  prevL = prevR = 0;


Serial.print(prizm.readEncoderCount(1));
 Serial.println(prizm.readEncoderCount(2));
 prizm.setMotorSpeeds(720, 720);
 


 delay(5000);


 
Serial.print(prizm.readEncoderCount(1));
 Serial.println(prizm.readEncoderCount(2));


  prizm.setMotorSpeeds(0, 0);


 delay(5000);

Serial.print(prizm.readEncoderCount(1));
 Serial.println(prizm.readEncoderCount(2));

*/


  /*
  moveStraight(-1000, 720);

  moveStraight(1000, 360);
  moveStraight(-1000, 360);



  //네모 그리기
  moveStraight(1000, 360);
  turnInPlace(90, 360);

  moveStraight(1000, 360);
  turnInPlace(90, 720);

  moveStraight(1000, 360);
  turnInPlace(90, 720);

  moveStraight(1000, 360);
  turnInPlace(90, 720);


  //빠르게 네모 그리기
  moveStraight(1000, 720);
  turnInPlace(90, 720);

  moveStraight(1000, 720);
  turnInPlace(90, 720);

  moveStraight(1000, 720);
  turnInPlace(90, 720);

  moveStraight(1000, 720);
  turnInPlace(90, 720);

*/
  //prizm.PrizmEnd();  //나중에 활성화하기

}