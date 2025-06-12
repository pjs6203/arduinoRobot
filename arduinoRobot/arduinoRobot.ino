/*
[I2C Address]
PRIZM : 0x01 ~ 0x06 (prizm programming guide 134P 참고)
frontToF (VL53L0X) : 0x29
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

//객체 생성
PRIZM prizm; //tetrix prizm
EXPANSION exc; //tetrix dc motor expansion controller
ThreadController controller; //odometry update
Thread odomThread; //odometry update
HUSKYLENS huskylens;
SoftwareSerial mySerial(2, 9); //huskylens 
VL53L0X distanceSensorFront;

//상세 정보를 시리얼 모니터로 출력
#define VERBOSE 1
#define VERBOSE_DELAY 0

//가감속도 설정
#define TARGET_SPD_DEG 360 //목표 속도, 0 ~ 720 [deg/s]
#define MIN_SPD_DEG 60 //이동 시 최소 속도를 설정합니다. [deg/s]
#define ACCEL_DPS2 720 //가감속 파라미터. [deg/s^2]
#define LOOP_DT_MS 5 //적분 주기를 설정합니다. [ms]





//바퀴 지름 설정
#define WHEEL_D_FWD   105.0f // 앞뒤용 휠 지름 [mm]
#define WHEEL_D_SIDE  105.0f // 좌우용 휠 지름 [mm]

//모터 2번 반전여부, -1 or +1
#define SIGN_M2 1

/* ─────── 전역 상태 ─────── */
volatile float x_mm = 0.0f;      // +x : 전방
volatile float y_mm = 0.0f;      // +y : 우측   (좌표계는 필요에 따라 교체)
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



//x, y를 업데이트 합니다.
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

  if(VERBOSE == 1){
    Serial.print(F("x mm : "));
    Serial.print(x_mm);
    Serial.print(F(" y mm : "));
    Serial.print(y_mm);

    Serial.print("DistanceSensorFront [mm] : ");
    Serial.println(readDistanceSensorFront());

    Serial.print(F("Motor 1 Encoder : "));
    Serial.print(prizm.readEncoderCount(1));
    Serial.print(F("Motor 2 Encoder : "));
    Serial.println(prizm.readEncoderCount(2));

    Serial.print(" QR : ");
    Serial.println(readQRdata());

    long raw = prizm.readEncoderCount(2);
    Serial.println(raw);

    delay(VERBOSE_DELAY);
  }

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
    long encNow   = prizm.readEncoderCount(motion.motIdx);
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
    bool condDone = false;
    if (motion.useCond && motion.condFn)
        condDone = cmp(motion.condOp, motion.condFn(), motion.condRef);

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
    motion.originTick = prizm.readEncoderCount(2);
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
    motion.originTick = prizm.readEncoderCount(2);
    motion.spdDeg   = 0;
    motion.tPrev    = millis();

    motion.useCond  = true;
    motion.condFn   = fn;
    motion.condOp   = op;
    motion.condRef  = ref;
}





//특정 Y 좌표까지 이동하기
void gotoWorldY(float targetY_mm)
{
    float tgt = clampY(targetY_mm);
    float dy  = tgt - y_mm;           // +앞 / –뒤
    if (fabsf(dy) >= 1.0f)            // 1 mm 이하는 무시
        startForward(dy);
}

//특정 X 좌표까지 이동하기
void gotoWorldX(float targetX_mm)
{
    float tgt = clampX(targetX_mm);
    float dx  = tgt - x_mm;           // +우 / –좌
    if (fabsf(dx) >= 1.0f)
        startSide(dx);
}




void setPoseFirst()
{
    y_mm = world_Y - (robotHeight / 2) - readDistanceSensorFront();
    Serial.print(F("y_mm : "));
    Serial.println(y_mm);

    x_mm = world_X - (robotWidth / 2) - readDistanceSensorRight();
    Serial.print(F("x_mm : "));
    Serial.println(x_mm);

    Serial.println(F("location set complete."));
}

void setPoseY()
{
    y_mm = world_Y - (robotHeight / 2) - readDistanceSensorFront();
    Serial.print(F("y_mm : "));
    Serial.println(y_mm);

    Serial.println(F("Y set complete"));
}

void setPoseX()
{
    x_mm = world_X - (robotWidth / 2) - readDistanceSensorRight();
    Serial.print(F("x_mm : "));
    Serial.println(x_mm);

    Serial.println(F("X set complete"));
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

  //I2C
  Wire.begin();

  //VL53L0X
  distanceSensor_init(); 

  //odometry thread
  odomThread.onRun(updateOdom);
  odomThread.setInterval(20);   // 50 Hz
  controller.add(&odomThread);

  setPoseFirst(); //처음 위치를 설정합니다. 
}

byte test = 0;
byte step1_scanPalette = 0; //팔레트 스캔
byte step2 = 0;
byte step3 = 0;
byte step4 = 0;
byte step5 = 0; 
int palletDest[7] = {0,0,0,0,0,0,0}; //구역 별 팔레트 목적지, 0번 인덱스는 아마 사용 안 할듯



void loop()
{
    controller.run(); //thread를 이용해 다른 함수를 실행하더라도 x, y를 업데이트합니다.

    motionUpdate();

    switch (test) //테스트용
    {
        case 0: //앞으로 1미터
            if (!motion.active) {
                startForward(1000);
                test = 1; 
            }
            break;

        case 1: //뒤로 1미터
            if (!motion.active) {
                startForward(-1000);
                test = 2;
            }
            break;

        case 2: //오른쪽으로 1미터
            if (!motion.active) {
                startSide(1000);
                test = 3;
            }
            break;

        case 3: //왼쪽으로 1미터
            if (!motion.active) {
                startSide(-1000);
                test = 4;
            }
            break;

        case 4: //일단 1미터 가는데, readDistanceSensorFront <= 500이면 멈추기
            if (!motion.active) {
                startForwardUntil(1000, readDistanceSensorFront, LE, 500);
                test = 5;
            }
            break;

        case 5:
            prizm.PrizmEnd();
            break;


    }




/*
    switch (step1_scanPalette) //팔레트 스캔하기
    {
        case 0: //Y축 중심으로 이동
            if (!motion.active) {
                gotoWorldY(700);
                step1_scanPalette = 1; // 다음 스텝으로 넘어가기
            }
            break;

        case 1: //X좌표 맞추기
            if (!motion.active) {
                gotoWorldX(3050); //X = 3050
                step1_scanPalette = 2;
            }
            break;

        case 2: //Y좌표 맞추기
            if (!motion.active) {
                gotoWorldY(1050); //X = 3050
                step1_scanPalette = 3;
            }
            break;

        case 3: //QR 스캐닝, 위치 오차 보정
            if (!motion.active) {
                palletDest[6] = readQRdataTimeout(1000); //1초 동안 QR을 스캔함
                setPoseY(); // 센서 이용하여 위치 오차 보정
                step1_scanPalette = 4;
            }
            break;

        case 4:
            step2 = 1; // 다시 짤 예정
            break;


    }

    */




    //prizm.PrizmEnd();
}