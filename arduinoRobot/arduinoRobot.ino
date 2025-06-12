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
#include "SoftwareSerial.h"


//VL53L0X
#define distanceSensorFront_ADDRESS 0x30
#define distanceSensorRight_ADDRESS 0x31
//#define distanceSensorFront_XSHUT 9 //D2-3
//#define distanceSensorRight_XSHUT 2 //D2-4

// objects
PRIZM prizm;
EXPANSION exc;
ThreadController controller;
Thread odomThread;

HUSKYLENS huskylens;

VL53L0X distanceSensorFront;
VL53L0X distanceSensorRight;
SoftwareSerial mySerial(2, 9); // RX, TX



/* ─────── 하드웨어 파라미터 ─────── */
#define WHEEL_D_FWD   105.0f      // 앞뒤용 휠 지름 [mm]
#define WHEEL_D_SIDE  105.0f      // 좌우용 휠 지름 [mm]

/* ────────── 가감속 공통 파라미터 ────────── */
#define TARGET_SPD_DEG  360      // 목표 속도(°/s)
#define MIN_SPD_DEG     60      // 정지 방지 최소 속도
#define ACCEL_DPS2      720      // 선형 가속·감속 (°/s²)
#define LOOP_DT_MS        5      // 제어 주기(ms)  →   dt = 0.005 s

/* 모터 2의 물리적 설치 방향 보정 (+1 또는 -1) */
#define SIGN_M2      1          // 오른쪽 이동이 +tick 이 되도록 맞춰라

/* ─────── 전역 상태 ─────── */
volatile float x_mm = 0.0f;      // +x : 전방
volatile float y_mm = 0.0f;      // +y : 우측   (좌표계는 필요에 따라 교체)
static   long  prevF = 0, prevS = 0;   // 이전 인코더 값

/* ─── 하드웨어 파라미터 ───────────────────── */
const float WHEEL_D   = 100;               // 휠 지름 [mm]
const float WHEEL_C   = PI * WHEEL_D;        // 휠 거리 [mm]
const float WHEEL_BASE= 315;               // 휠 간 거리 [mm] 확인 후 바꿔야함
const int   TICKS_REV = 1440;                // 1회전 당 엔코더 틱 [ticks/rev]
const float MM_PER_TICK = WHEEL_C / TICKS_REV;
const int   world_X = 3400;
const int   world_Y = 1400;
const int   robotHeight = 0; //세로길이, 바꿔야함
const int   robotWidth = 0; //가로길이, 바꿔야함



/* mm·deg → 엔코더 tick 변환 */
inline float deg2tick(float deg) { return (deg * TICKS_REV) / 360.0f; }
inline float tick2deg(float tick){ return (tick * 360.0f) / (float)TICKS_REV; }

/* mm·deg → 엔코더 tick 변환 */
inline long mm2tick(float mm)   { return lround(mm / MM_PER_TICK); }


struct MotionState {
    bool   active     = false;   // 실행 중인가?
    uint8_t motIdx    = 0;       // 1=전진축, 2=측면축
    long   tgtTicks   = 0;       // 목표 tick (부호 포함)
    long   originTick = 0;       // 시작 시점의 엔코더 값
    int    dir        = 1;       // +1 / –1
    float  spdDeg     = 0.0f;    // 현재 목표 속도(°/s)
    unsigned long tPrev = 0;     // 직전 업데이트 시각(ms)
} motion;



/* --- 엔코더와 내부 변수 동시 리셋 --- */
inline void syncEncoders()
{
  noInterrupts();          // 오도메트리 쓰레드와 충돌 방지
  prizm.resetEncoders();
  prevF = prevS = 0;
  interrupts();
}




//x, y 업데이트
void updateOdom()
{
  /* ① 현재 인코더 읽기 */
  long curF =  prizm.readEncoderCount(1);
  long curS =  prizm.readEncoderCount(2);

  /* ② 증분 계산(tick) */
  long dnF  = curF - prevF;
  long dnS  = curS - prevS;
  prevF = curF;
  prevS = curS;

  /* ③ tick → mm 변환 */
  const float mmPerTickF = (PI * WHEEL_D_FWD ) / (float)TICKS_REV;
  const float mmPerTickS = (PI * WHEEL_D_SIDE) / (float)TICKS_REV;
  float dx = dnF * mmPerTickF;   // + 전진 / - 후진
  float dy = dnS * mmPerTickS;   // + 우측  / - 좌측

  /* ④ 적분 (임계 구역 보호) */
  noInterrupts();
  x_mm += dx;
  y_mm += dy;
  interrupts();

  Serial.print(F("x mm : "));
  Serial.print(x_mm);
  Serial.print(F(" y mm : "));
  Serial.println(y_mm);

  Serial.print("QR : ");
  Serial.println(readQRdata());

  Serial.print("DistanceSensorFront [mm] : ");
  Serial.println(readDistanceSensorFront());

  long raw = prizm.readEncoderCount(2);
  Serial.println(raw);
  //delay(50);


}

static void runProfile(int motIdx, long tgtTicks)
{

  const int dir = (tgtTicks >= 0) ? +1 : -1;
  long  absTot  = tgtTicks;
  long  prevEnc = prizm.readEncoderCount(motIdx);

  float spdDeg  = 0.0f;                     // [°/s] 현재 목표 속도
  unsigned long tPrev = millis();

  while (true)
  {
    /* ----- Δ시간 · 엔코더 갱신 ----- */
    unsigned long tNow = millis();
    float dt = (tNow - tPrev) / 1000.0f;    // s
    if (dt < LOOP_DT_MS/1000.0f) continue;  // 주기 유지
    tPrev = tNow;

    long encNow = prizm.readEncoderCount(motIdx);
    if (motIdx == 2) encNow = -encNow;      // 모터2 반전 보정
    long moved  = encNow;             // 누적 tick
    long remain = absTot - moved;           // 잔여 tick

    /* ----- 제동 필요 여부 판단 ----- */
    float dStop = deg2tick( (spdDeg * spdDeg) / (2.0f * ACCEL_DPS2) );
    bool  braking = (remain <= dStop);

    /* ----- 속도 업데이트 ----- */
    if (!braking)  spdDeg = min(spdDeg + ACCEL_DPS2 * dt, (float)TARGET_SPD_DEG);
    else           spdDeg = max(spdDeg - ACCEL_DPS2 * dt, (float)MIN_SPD_DEG);

    /* ----- 모터 구동 ----- */
    int cmdSpd = (int)spdDeg;               // 프리즘은 정수 deg/s
    prizm.setMotorSpeeds(
        (motIdx==1 ? dir*cmdSpd : 0),
        (motIdx==2 ? dir*cmdSpd : 0) );

    /* ----- 종료 조건 ----- */
    bool done = (tgtTicks >= 0) ? (encNow >= tgtTicks) : (encNow <= tgtTicks);
    if (done) break;

    controller.run();  
  }

  prizm.setMotorSpeeds(0,0);
}


void motionUpdate()
{
    if (!motion.active) return;               // 할 일 없음

    unsigned long tNow = millis();
    float dt = (tNow - motion.tPrev) * 0.001f;
    if (dt < LOOP_DT_MS * 0.001f) return;     // 아직 주기가 안 됨
    motion.tPrev = tNow;

    /* ── 현재 이동량 계산 ─────────────────── */
  // (motionUpdate 내부)  ────── Δ tick 읽기
  long encNow = prizm.readEncoderCount(motion.motIdx);
  long progress = (encNow - motion.originTick) * motion.dir;
  if (progress < 0) progress = 0;                 // 뒤로 밀린 경우 0으로 클램프

  long moved  = labs(encNow - motion.originTick);
  long absTot = labs(motion.tgtTicks);
/* 잔여 tick (음수면 0으로 클램프) */
// 남은 tick
long remain = labs(motion.tgtTicks) - progress;
if (remain < 0) remain = 0;

float dStop  = deg2tick((motion.spdDeg * motion.spdDeg) / (2.0f * ACCEL_DPS2));
bool  braking = (remain <= dStop);

    if (!braking)
        motion.spdDeg = min(motion.spdDeg + ACCEL_DPS2 * dt,
                            (float)TARGET_SPD_DEG);
    else
        motion.spdDeg = max(motion.spdDeg - ACCEL_DPS2 * dt,
                            (float)MIN_SPD_DEG);

    int cmd = (int)motion.spdDeg * motion.dir;
    prizm.setMotorSpeeds(
        (motion.motIdx == 1 ? cmd : 0),
        (motion.motIdx == 2 ? cmd : 0) );

    /* ── 완료 판단 ───────────────────────── */
bool done = (progress >= labs(motion.tgtTicks));
    if (done) {
        prizm.setMotorSpeeds(0, 0);
        motion.active = false;
    }
}




void moveForward(float mm)
{
  syncEncoders();          // ← 변경
  runProfile(1, mm2tick(mm));
}

void moveSide(float mm)
{
  syncEncoders();          // ← 변경
  runProfile(2, mm2tick(mm));
}




void startForward(float mm)
{
    motion.active   = true;
    motion.motIdx   = 1;
    motion.tgtTicks = mm2tick(mm);
    motion.dir      = (motion.tgtTicks >= 0) ? +1 : -1;
    motion.originTick = prizm.readEncoderCount(1); // 시작 기준
    motion.spdDeg   = 0.0f;
    motion.tPrev    = millis();
}

void startSide(float mm)
{
    motion.active   = true;
    motion.motIdx   = 2;
    motion.tgtTicks = mm2tick(mm);
    motion.dir      = (motion.tgtTicks >= 0) ? +1 : -1;
    motion.originTick = prizm.readEncoderCount(2);
    motion.spdDeg   = 0.0f;
    motion.tPrev    = millis();
}




void setup() 
{
  //Serial Port
  Serial.begin(115200);

  //PRIZM Board
  prizm.PrizmBegin();
  //prizm.setMotorInvert(2,1); //2번 모터를 반전
  prizm.resetEncoders();

  //I2C
  Wire.begin();

  //Huskylens
  mySerial.begin(9600);
  while (!huskylens.begin(mySerial))
  {
      Serial.println(F("Begin failed!"));
        Serial.println(F("1.Please recheck the \"Protocol Type\" in HUSKYLENS (General Settings>>Protocol Type>>Serial 9600)"));
        Serial.println(F("2.Please recheck the connection."));
        delay(100);
    }

  //VL53L0X
  distanceSensor_init(); 

  //odometry thread
  odomThread.onRun(updateOdom);
  odomThread.setInterval(20);   // 50 Hz
  controller.add(&odomThread);
}

byte step = 0;             // ← loop() 밖, 전역에 선언

void loop()
{
    controller.run();
    motionUpdate();


    /*
    switch (step)
    {
        case 0:
            if (!motion.active) {
                startForward(1000);        // +1 m
                step = 1;
            }
            break;

        case 1:
            if (!motion.active) {
                startForward(-1000);       // -1 m (복귀)
                step = 2;
            }
            break;

        case 2:
            if (!motion.active) {
                startSide(1000);           // +1 m 측면 이동
                step = 3;
            }
            break;

        case 3:
            if (!motion.active) {
                startSide(-1000);          // -1 m 복귀
                step = 4;                  // ▸ 끝
            }
            break;

        case 4:
                if (!motion.active) {
                //startSide(0);          // -1 m 복귀
                step = 5;                  // ▸ 끝
            }
            break;


                    case 5:
            break;
    }
    */


    //prizm.PrizmEnd();
}