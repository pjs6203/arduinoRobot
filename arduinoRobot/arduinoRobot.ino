/*
[I2C Address]
PRIZM : 0x01 ~ 0x06 (prizm programming guide 134P 참고)
frontToF (VL53L0X) : 0x29
HUSKYLENS : 0x32
*/



/*
[할 것]
1. 리프트 delay 없이 하기
2. 어떠한 배열을 입력하면, 순서를 리턴하는 함수 ?..
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






typedef float (*ValueFn)(); // 예: readFrontToF(), [](){ return x_mm; }
enum CmdOp { GE, LE, GT, LT, EQ };
inline bool cmp(CmdOp op, float a, float b)
{
    switch(op){
      case GE: return a >= b;
      case LE: return a <= b;
      case GT: return a >  b;
      case LT: return a <  b;
      case EQ: return fabsf(a-b) < 0.001f;
      default: return false;
    }
}

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
#include <SoftwareSerial.h>
#include <math.h>  // fabs

//객체 생성
PRIZM prizm; //tetrix prizm
EXPANSION exc; //tetrix dc motor expansion controller
ThreadController controller; //odometry update
Thread odomThread; //odometry update
HUSKYLENS huskylens;
SoftwareSerial mySerial(2, 9); //huskylens 
VL53L0X distanceSensorFront;

//dc motor expansion controller ID
#define EXP_ID 2

//상세 정보를 시리얼 모니터로 출력
#define VERBOSE 1

//가감속도 설정
#define TARGET_SPD_DEG 720 //목표 속도, 0 ~ 720 [deg/s] 500
#define MIN_SPD_DEG 60 //이동 시 최소 속도를 설정합니다. [deg/s] 60
#define ACCEL_DPS2 360 //가감속도 [deg/s^2] 100
#define LOOP_DT_MS 5 //적분 주기를 설정합니다. [ms]

//바퀴 지름 설정
#define WHEEL_D_FWD   100.0f // 앞뒤용 휠 지름 [mm]
#define WHEEL_D_SIDE  100.0f // 좌우용 휠 지름 [mm]

//모터 2번 반전여부, -1 or +1
#define SIGN_M2 1

#define LINE_PIN 4

/* ─────── 전역 상태 ─────── */
volatile float x_mm = 0.0f;      // +x : 전방
volatile float y_mm = 0.0f;      // +y : 우측   (좌표계는 필요에 따라 교체)
volatile uint8_t lastQR = 0;   // 0 = 아직 없음 / 1~6 = 최근 인식된 ID
static   long  prevF = 0, prevS = 0;   // 이전 인코더 값

/* ─── 하드웨어 파라미터 ───────────────────── */
const float WHEEL_D   = 100; //
const float WHEEL_C   = PI * WHEEL_D;        // 휠 거리 [mm]
const float WHEEL_BASE= 315;               // 휠 간 거리 [mm] 확인 후 바꿔야함
const int   TICKS_REV = 1440;                // 1회전 당 엔코더 틱 [ticks/rev]
const float MM_PER_TICK = WHEEL_C / TICKS_REV;
const int   world_X = 3400;
const int   world_Y = 1400;
const int   robotHeight = 288; //세로길이, 바꿔야함
const int   robotWidth = 288; //가로길이, 바꿔야함

//틱 당 mm [tick/mm]
#define MM_PER_TICK_FWD   ( (PI * WHEEL_D_FWD ) / TICKS_REV )
#define MM_PER_TICK_SIDE  ( (PI * WHEEL_D_SIDE) / TICKS_REV )
inline long mm2tickFWD (float mm) { return lround(mm / MM_PER_TICK_FWD ); } //mm -> tick
inline long mm2tickSIDE(float mm) { return lround(mm / MM_PER_TICK_SIDE); } //mm -> tick

/* mm·deg → 엔코더 tick 변환 */
inline float deg2tick(float deg) { return (deg * TICKS_REV) / 360.0f; }
inline float tick2deg(float tick){ return (tick * 360.0f) / (float)TICKS_REV; }

int palletDest[7] = {0,0,0,0,0,0,0}; //구역 별 팔레트 목적지, 0번 인덱스는 아마 사용 안 할듯


//x, y를 업데이트 합니다.
void updateOdom()
{
  /* ① 현재 인코더 읽기 */
  long curF =  -prizm.readEncoderCount(2);
  long curS =  prizm.readEncoderCount(1);

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

  /* 이미 있는 출력 블록 바로 위, 또는 아래 아무 곳 */
  uint8_t q = readQRdata();   // 한 번 읽기
  if (q != 0) lastQR = q;     // 0이 아니면 덮어쓴다

  /* ④ 적분 (임계 구역 보호) */
  noInterrupts();
  x_mm += dx;
  y_mm += dy;
  interrupts();

  if(VERBOSE == 1){
    Serial.print("BatteryVoltage : ");
    Serial.print(prizm.readBatteryVoltage());
    Serial.print(F(" Motor 1 Encoder : "));
    Serial.print(prizm.readEncoderCount(1));
    Serial.print(F(" Motor 2 Encoder : "));
    Serial.println(prizm.readEncoderCount(2));

    Serial.print(F("y mm : "));
    Serial.print(y_mm);
    Serial.print(F(" x mm : "));
    Serial.print(x_mm);
    Serial.print(" DistanceSensorFront [mm] : ");
    Serial.print(readDistanceSensorFront());
    Serial.print(" DistanceSensorRight [mm] : ");
    Serial.println(readDistanceSensorRight());

    Serial.print("lastQR : ");
    Serial.print(lastQR);
    Serial.print(" QR : ");
    Serial.println(readQRdata());

    Serial.print("palletDest[] ");

    for(int i = 0; i < 7; i++)
    {
        Serial.print(i);
        Serial.print(" : ");
        Serial.print(palletDest[i]);
        Serial.print(", ");
    }
    Serial.println();
  }

    Serial.print(" line : ");
    Serial.println(digitalRead(4));
}

struct MotionState {
    bool   active   = false;
    uint8_t motIdx  = 0;
    long   tgtTicks = 0;
    long   originTick = 0;
    int    dir      = 1;
    float  spdDeg   = 0.0f;
    unsigned long tPrev = 0;

    /* ★ 조건 중단 파라미터 */
    bool      useCond   = false;
    ValueFn   condFn    = nullptr;
    CmdOp     condOp    = GE;
    float     condRef   = 0;
} motion;



inline float clampX(float X)  // 로봇이 벽을 뚫지 않도록
{
    float minX = robotWidth  * 0.5f;
    float maxX = world_X - minX;
    return constrain(X, minX, maxX);
}
inline float clampY(float Y)
{
    float minY = robotHeight * 0.5f;
    float maxY = world_Y - minY;
    return constrain(Y, minY, maxY);
}

void motionUpdate()
{
    if (!motion.active) return;

    unsigned long tNow = millis();
    float dt = (tNow - motion.tPrev) * 0.001f;
    if (dt < LOOP_DT_MS * 0.001f) return;
    motion.tPrev = tNow;

    /* ── 진행량 계산 ───────────────────── */
    long encNow = - prizm.readEncoderCount(motion.motIdx);
    if (motion.motIdx == 1 ){
        encNow  = prizm.readEncoderCount(motion.motIdx);
    }
    long progress = (encNow - motion.originTick) * motion.dir;
    if (progress < 0) progress = 0;

    long remain = labs(motion.tgtTicks) - progress;
    if (remain < 0) remain = 0;

    /* ── 속도 업데이트 ─────────────────── */
    float dStop  = deg2tick((motion.spdDeg * motion.spdDeg) /
                            (2.0f * ACCEL_DPS2));
    bool braking = (remain <= dStop);

    if (!braking)
        motion.spdDeg = min(motion.spdDeg + ACCEL_DPS2 * dt,
                            (float)TARGET_SPD_DEG);
    else
        motion.spdDeg = max(motion.spdDeg - ACCEL_DPS2 * dt,
                            (float)MIN_SPD_DEG);

    int cmd = (int)motion.spdDeg * motion.dir;
    prizm.setMotorSpeeds(
        (motion.motIdx == 1 ? cmd : 0),
        (motion.motIdx == 2 ? cmd : 0));

    /* ── 완료 판단 ─────────────────────── */
    bool distanceDone = (progress >= labs(motion.tgtTicks));      // 원래 조건

    /* ★ 추가: 센서 조건 */

/* motionUpdate() 완료 판단 부분만 고쳐 줍니다 */
bool condDone = false;
if (motion.useCond && motion.condFn)
{
    float v = motion.condFn();          // 센서값
    if (v > 0)                          // 0 → 아직 값 없음/오류 → 무시
        condDone = cmp(motion.condOp, v, motion.condRef);
}


    if (distanceDone || condDone) {
        prizm.setMotorSpeeds(0, 0);
        motion.active = false;
    }
}


void startForward(float mm)
{
    motion.active   = true;
    motion.motIdx   = 1;
    motion.tgtTicks = mm2tickFWD(mm);
    motion.dir      = (motion.tgtTicks >= 0) ? +1 : -1;
    motion.originTick = prizm.readEncoderCount(1); // 시작 기준
    motion.spdDeg   = 0.0f;
    motion.tPrev    = millis();
}

void startSide(float mm)
{
    motion.active   = true;
    motion.motIdx   = 2;
    motion.tgtTicks = mm2tickSIDE(mm);
    motion.dir      = (motion.tgtTicks >= 0) ? +1 : -1;
    motion.originTick = -prizm.readEncoderCount(2);
    motion.spdDeg   = 0.0f;
    motion.tPrev    = millis();
}


void startForwardUntil(float mm,
                       ValueFn fn, CmdOp op, float ref)
{
    motion.active   = true;
    motion.motIdx   = 1;
    motion.tgtTicks = mm2tickFWD(mm);
    motion.dir      = (motion.tgtTicks >= 0) ? +1 : -1;
    motion.originTick = prizm.readEncoderCount(1);
    motion.spdDeg   = 0;
    motion.tPrev    = millis();

    motion.useCond  = true;
    motion.condFn   = fn;
    motion.condOp   = op;
    motion.condRef  = ref;
}

void startSideUntil(float mm,
                    ValueFn fn, CmdOp op, float ref)
{
    motion.active   = true;
    motion.motIdx   = 2;
    motion.tgtTicks = mm2tickSIDE(mm);
    motion.dir      = (motion.tgtTicks >= 0) ? +1 : -1;
    motion.originTick = -prizm.readEncoderCount(2);
    motion.spdDeg   = 0;
    motion.tPrev    = millis();

    motion.useCond  = true;
    motion.condFn   = fn;
    motion.condOp   = op;
    motion.condRef  = ref;
}







void setPoseFirst()
{
    y_mm = world_Y - (355 * 0.5) - readDistanceSensorFront();
    Serial.print(F("y_mm : "));
    Serial.println(y_mm);

    x_mm = world_X - 230 - readDistanceSensorRight();
    Serial.print(F("x_mm : "));
    Serial.println(x_mm);

    Serial.println(F("location set complete."));
}

void setPoseY(int mm = -1)
{
    if(mm > 0){
        y_mm = mm;
    }else{
    y_mm = world_Y - (355 * 0.5) - readDistanceSensorFront();
    Serial.print(F("y_mm : "));
    Serial.println(y_mm);
    Serial.println(F("Y set complete"));
    }
}

void setPoseX(int mm = -1)
{
    if(mm > 0){
        y_mm = mm;
    }else{
    x_mm = world_X - (355 * 0.5) - readDistanceSensorRight();
    Serial.print(F("x_mm : "));
    Serial.println(x_mm);
    Serial.println(F("X set complete"));
    }
}




/* --------------------------------------------------------------------
   1. gotoWorldY  ―  목표 Y(mm)까지 직진
      · targetY_mm : 절대 좌표 (월드 기준)
      · tol_mm     : 오차 허용(기본 1mm). 1mm 이내면 이동 생략.
   -------------------------------------------------------------------- */
void gotoWorldY(float targetY_mm, float tol_mm = 1.0f)
{
    float tgt = clampY(targetY_mm);        // 벽 넘어가지 않도록
    float dy  = tgt - (world_Y - (355 * 0.5) - readDistanceSensorFront());                // +앞 / –뒤
    if (fabsf(dy) < tol_mm) return;        // 거의 제자리면 끝

    startForward(dy);                      // ① 주행 시작

    while (motion.active) {                // ② 완료까지 대기
        controller.run();                  // 오도메트리 쓰레드
        motionUpdate();                    // 가감속·엔코더
    }
}

/* --------------------------------------------------------------------
   2. gotoWorldX  ―  목표 X(mm)까지 좌·우 이동
      · targetX_mm : 절대 좌표
      · tol_mm     : 오차 허용(1mm 기본)
   -------------------------------------------------------------------- */
void gotoWorldX(float targetX_mm, float tol_mm = 1.0f)
{
    float tgt = clampX(targetX_mm);
    float dx  = tgt - x_mm;                // +우 / –좌
    if (fabsf(dx) < tol_mm) return;

    startSide(dx);                         // ① 주행 시작

    while (motion.active) {                // ② 완료까지 대기
        controller.run();
        motionUpdate();

    }
}




const int REGION_X[7] = { 0,350,1050,350,1050,2350,3050 };


/* 1) 지금 바로 옮길 수 있는 팔레트가 있는 가장 가까운 구역 */
int findNearestMovableRegion(const int dest[7], double robotX) {
    int nearestIdx = 0;
    double minDist = 1e9;

    for (int i = 1; i <= 6; ++i) {
        if (dest[i] == 0) continue;
        int target = dest[i];
        if (dest[target] != 0) continue;  // 목적지에 이미 팔레트가 있으면 스킵

        double d = fabs(REGION_X[i] - robotX);  // <-- fabs 사용
        if (d < minDist) {
            minDist = d;
            nearestIdx = i;
        }
    }
    return nearestIdx;
}

/* 2) 팔레트가 “어디든” 있는 곳 중 가장 가까운 구역 (목적지 막힘 무시) */
int findNearestLoadedRegion(const int dest[7], double robotX)
{
    int idx = 0; double minD = 1e9;

    for (int i = 1; i <= 6; ++i) {
        if (dest[i] == 0) continue;             // 비어 있음

        double d = fabs(REGION_X[i] - robotX);
        if (d < minD) { minD = d; idx = i; }
    }
    return idx;     // 없으면 0 (모든 구역이 비어 있으면)
}


/* 3) 비어 있는 구역(팔레트 없음) 중 가장 가까운 곳 */
int findNearestEmptyRegion(const int dest[7], double robotX)
{
    int idx = 0; double minD = 1e9;

    for (int i = 1; i <= 6; ++i) {
        if (dest[i] != 0) continue;             // 팔레트 있으면 skip

        double d = fabs(REGION_X[i] - robotX);
        if (d < minD) { minD = d; idx = i; }
    }
    return idx;     // 없으면 0 (빈 칸이 하나도 없으면)
}






//const int REGION_X[7] = { 0,350,1050,350,1050,2600,3050 };


//어떠한 한 구역에 가서 QR을 스캔한 후 스캔 성공 여부를 리턴한다.
bool scanOneZone(uint8_t zoneIdx)
{
    lastQR = 0;                        // 1) QR 초기화
    gotoWorldY(600);                   // 2) Y 중앙선
    gotoWorldX(REGION_X[zoneIdx]);     // 3) X 정렬

    if(zoneIdx == 3 || zoneIdx == 4 ){
        driveUntilFrontGE(-10000, 1250); 
    }else{
        driveUntilFrontLE(10000, 150); 
    }

    if (lastQR != 0) {                 // 5) QR이 실제로 읽혔을 때만
        palletDest[zoneIdx] = lastQR;  //    배열에 기록
        setPoseY();                    //    Y 보정
        return true;                   //    성공
    }
    return false;                      // QR 없음(0) → 실패
}


//어떠한 한 구역에 간다.
void goOneZone(uint8_t zoneIdx)
{
    lastQR = 0;                        // 1) QR 초기화
    gotoWorldY(700);                   // 2) Y 중앙선
    gotoWorldX(REGION_X[zoneIdx]);     // 3) X 정렬
    if(zoneIdx == 3 || zoneIdx == 4){
        driveUntilFrontGE(-1000, 1300);
    }else{
        driveUntilFrontLE(1000, 100);
    }
}

//집으로 돌아간다.
void goHome()
{
    gotoWorldY(700);                   // 2) Y 중앙선
    gotoWorldX(3150);     // 3) X 정렬
}




//어떠한 한 구역에 가서 리프트를 올린 후, 타겟 구역에 놓는다.
void pickupAndDrop(uint8_t zoneIdx, uint8_t zoneIdxTarget)
{
    lastQR = 0;                        // 1) QR 초기화
    gotoWorldY(700);                   // 2) Y 중앙선
    gotoWorldX(REGION_X[zoneIdx]);     // 3) X 정렬
    if(zoneIdx == 3 || zoneIdx == 4){
        driveUntilFrontGE(-1000, 1300);
        liftUp();
    }else{
        driveUntilFrontLE(1000, 100);
        liftUp();
    }

    gotoWorldY(700);                   // 2) Y 중앙선
    gotoWorldX(REGION_X[zoneIdxTarget]);     // 3) X 정렬
    if(zoneIdxTarget == 3 || zoneIdxTarget == 4){
        driveUntilFrontGE(-1000, 1300);
        liftDown();
    }else{
        driveUntilFrontLE(1000, 100);
        liftDown();
    }
    palletDest[zoneIdxTarget] = zoneIdx;
    palletDest[zoneIdx] = 0;
}


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






/*  true  →  더 이상 옮길 팔레트 없음
    false →  아직 옮겨야 할 팔레트 있음                      */
bool isAllPalletsDone(const int dest[7])
{
    for (int i = 1; i <= 6; ++i) {
        if (dest[i] != 0 && dest[i] != i)
            return false;          // 제자리도 아니고 비어 있지도 않음
    }
    return true;                   // 모두 자리 완료
}


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
        return distanceSensorFront.readRangeContinuousMillimeters() - 246; //보정값 추가
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
    return (int16_t)(dist_cm * 10.0f + 0.5f) - 195;   // 반올림, 보정값 추가
}



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


/* ① 전방(앞/뒤) 이동 + 라인센서 정지 조건 ---------------------- */
void driveUntilFrontLine(float startDist_mm)
{
    /* a) 주행 시작 -------------------------------------------- */
    startForward(startDist_mm);

    /* b) “절반 거리”를 tick 단위로 계산 ------------------------- */
    long halfTicks = labs(mm2tickFWD(startDist_mm)) / 2;
    long encStart  = prizm.readEncoderCount(1);   // 시작 시 tick

    /* c) 루프 -------------------------------------------------- */
    while (motion.active) {
        controller.run();
        motionUpdate();

        /* 주행 경과 tick 계산 */
        long encNow   = -prizm.readEncoderCount(1);
        long traveled = labs(encNow - encStart);

        bool halfPassed = (traveled >= halfTicks);
        bool lineOK     = (digitalRead(LINE_PIN) == HIGH);

        if (halfPassed && lineOK) {             // 둘 다 만족할 때만 정지
            prizm.setMotorSpeeds(0, 0);
            motion.active = false;
        }
    }
}

/* ② 좌·우 이동 + 라인센서 정지 조건 (startDist_mm: +우 / –좌) -- */
void driveUntilRightLine(float startDist_mm)
{
    startSide(startDist_mm);

    long halfTicks = labs(mm2tickSIDE(startDist_mm)) / 2;
    long encStart  = -prizm.readEncoderCount(2);   // 모터 2번

    while (motion.active) {
        controller.run();
        motionUpdate();

        long encNow   = -prizm.readEncoderCount(2);
        long traveled = labs(encNow - encStart);

        bool halfPassed = (traveled >= halfTicks);
        bool lineOK     = (digitalRead(LINE_PIN) == HIGH);

        if (halfPassed && lineOK) {
            prizm.setMotorSpeeds(0, 0);
            motion.active = false;
        }
    }
}


/* ② 좌·우 이동 + 라인센서 정지 조건 (startDist_mm: +우 / –좌) -- */
void driveUntilRightLineForce(float startDist_mm)
{
    startSide(startDist_mm);

    while (motion.active) {
        controller.run();
        motionUpdate();

        bool lineOK = (digitalRead(LINE_PIN) == HIGH);

        if (lineOK) {
            prizm.setMotorSpeeds(0, 0);
            motion.active = false;
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




void liftUp()
{
  exc.resetEncoders(EXP_ID);
  exc.setMotorTarget(EXP_ID, 1, 270, -1440*1.5);
  delay(1000);
}


void liftDown()
{
  exc.resetEncoders(EXP_ID);
  exc.setMotorTarget(EXP_ID, 1, 270, 1440*1.5);
  delay(1000);
}



void setup() 
{
  //Serial Port
  Serial.begin(115200);

  //Huskylens & SWSerial
  mySerial.begin(9600);
  while (!huskylens.begin(mySerial))
  {
    Serial.println(F("Begin failed!"));
    Serial.println(F("1.Please recheck the \"Protocol Type\" in HUSKYLENS (General Settings>>Protocol Type>>Serial 9600)"));
    Serial.println(F("2.Please recheck the connection."));
    delay(100);
  }

  //PRIZM Board
  prizm.PrizmBegin();
  prizm.resetEncoders();
  prizm.setMotorInvert(1,0);
  prizm.setMotorInvert(2,1);

  //I2C
  Wire.begin();

  //VL53L0X
  distanceSensor_init(); 

  //odometry thread
  odomThread.onRun(updateOdom);
  odomThread.setInterval(20);   // 50 Hz
  controller.add(&odomThread);

  //line sensor
  pinMode(LINE_PIN, INPUT);

  setPoseFirst(); //처음 위치를 설정합니다. 
}



/*
1구역 : 350, 1200
2구역 : 1050, 1200
3구역 : 350, 200
4구역 : 1050, 200
5구역 : 2600, 1200
6구역 : 3050, 1200
시작/종료 구역 : 3150, 250
*/







void loop()
{
    controller.run(); //thread를 이용해 다른 함수를 실행하더라도 x, y를 업데이트합니다.

    motionUpdate();


    //forward(1000);
    //forward(-1000);
    //side(1000);
    //side(-1000);
    //driveUntilFront(1000, 500);



    //1 ~ 6구간 모두 스캔 후 palletdest 변수에 저장합니다.
    //만약 QR스캔이 제대로 안 돼서 변수에 2-3개만 저장된다면 다시 스캔해야하는데, 어떻게 구현해야할까
    for(int i = 1; i < 7; i++){ 
        int tmp = 0;
        int ind[7] = {0, 6, 5, 2, 4, 1, 3};
        tmp = tmp + scanOneZone(ind[i]);
        if (tmp == 4) break;
    }
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

    // 6번
    lastQR = 0;                        // 1) QR 초기화
    gotoWorldY(700);
    driveUntilRightLineForce(-500);
    setPoseX(3050);
    driveUntilFrontLE(1000, 100);     
   
    if (lastQR != 0) {                 // 5) QR이 실제로 읽혔을 때만
        palletDest[6] = lastQR;  //    배열에 기록
        setPoseY();                    //    Y 보정
    }

    // 5번
    lastQR = 0;                        // 1) QR 초기화
    gotoWorldY(700);
    driveUntilRightLine(-500);
    setPoseX(2600);
    driveUntilFrontLE(1000, 100);        
    if (lastQR != 0) {                 // 5) QR이 실제로 읽혔을 때만
        palletDest[5] = lastQR;  //    배열에 기록
        setPoseY();                    //    Y 보정
    }

     // 2번
    lastQR = 0;                        // 1) QR 초기화
    gotoWorldY(700);
    driveUntilRightLine(-1500);
    setPoseX(1050);
    driveUntilFrontLE(1000, 100);        
    if (lastQR != 0) {                 // 5) QR이 실제로 읽혔을 때만
        palletDest[2] = lastQR;  //    배열에 기록
        setPoseY();                    //    Y 보정
    }

    //4번
    lastQR = 0;                        // 1) QR 초기화
    gotoWorldY(700);
    driveUntilFrontGE(-1000, 1300);        
    if (lastQR != 0) {                 // 5) QR이 실제로 읽혔을 때만
        palletDest[4] = lastQR;  //    배열에 기록
        setPoseY();                    //    Y 보정
    }


     //1번
    lastQR = 0;                        // 1) QR 초기화
    gotoWorldY(700);
    setPoseX(350);
    driveUntilRightLine(-1500);
    driveUntilFrontLE(1000, 100);        
    if (lastQR != 0) {                 // 5) QR이 실제로 읽혔을 때만
        palletDest[1] = lastQR;  //    배열에 기록
        setPoseY();                    //    Y 보정
    }

    //3번
    lastQR = 0;                        // 1) QR 초기화
    gotoWorldY(700);
    driveUntilFrontGE(-1000, 1300);        
    if (lastQR != 0) {                 // 5) QR이 실제로 읽혔을 때만
        palletDest[3] = lastQR;  //    배열에 기록
        setPoseY();                    //    Y 보정
    }



*/

    while(true){ //palletdest의 인덱스와 데이터가 같다면 끝냅니다.

        int flag = 0;

        if (isAllPalletsDone(palletDest)) //모든 팔레트가 제자리에 있다면, 
        {
            break;                       // while 탈출
        }

        //만약 옮길 수 있는 팔레트가 없다면, 
        if(findNearestMovableRegion(palletDest, x_mm) == 0)
        {
            flag = 1; //옮길 수 없다.
        }
        else //있다면,
        {
            flag = 0; //옮길 수 있다.
        }

        switch(flag)
        {
            case 0: //옮길 수 있을 때 그것을 옮긴다.
                pickupAndDrop(findNearestMovableRegion(palletDest, x_mm), palletDest[findNearestMovableRegion(palletDest, x_mm)]);
                break;

            case 1: //옮길 수 없을 때 그냥 가장 가까운 팔레트를 아무 곳에 둔다.
                int loadedregion = findNearestLoadedRegion(palletDest, x_mm);
                int emptyregion = findNearestEmptyRegion(palletDest, x_mm);
                pickupAndDrop(loadedregion, emptyregion);
                break;
        }
    }

    
    //집으로 돌아가요.
    Serial.println("집으로 돌아갑니다.");
    goHome();



   // prizm.PrizmEnd();
}