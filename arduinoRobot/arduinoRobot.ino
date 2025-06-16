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
#define TARGET_SPD_DEG 500 //목표 속도, 0 ~ 720 [deg/s]
#define MIN_SPD_DEG 60 //이동 시 최소 속도를 설정합니다. [deg/s]
#define ACCEL_DPS2 100 //가감속도 [deg/s^2]
#define LOOP_DT_MS 5 //적분 주기를 설정합니다. [ms]

//바퀴 지름 설정
#define WHEEL_D_FWD   100.0f // 앞뒤용 휠 지름 [mm]
#define WHEEL_D_SIDE  100.0f // 좌우용 휠 지름 [mm]

//모터 2번 반전여부, -1 or +1
#define SIGN_M2 1

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




//x, y를 업데이트 합니다.
void updateOdom()
{
  /* ① 현재 인코더 읽기 */
  long curF =  -prizm.readEncoderCount(2);
  long curS =  -prizm.readEncoderCount(1);

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
    long encNow   = - prizm.readEncoderCount(motion.motIdx);
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
    motion.originTick = -prizm.readEncoderCount(1); // 시작 기준
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
    motion.originTick = -prizm.readEncoderCount(1);
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

    x_mm = world_X - (355 * 0.5) - readDistanceSensorRight();
    Serial.print(F("x_mm : "));
    Serial.println(x_mm);

    Serial.println(F("location set complete."));
}

void setPoseY()
{
    y_mm = world_Y - (355 * 0.5) - readDistanceSensorFront();
    Serial.print(F("y_mm : "));
    Serial.println(y_mm);

    Serial.println(F("Y set complete"));
}

void setPoseX()
{
    x_mm = world_X - (355 * 0.5) - readDistanceSensorRight();
    Serial.print(F("x_mm : "));
    Serial.println(x_mm);

    Serial.println(F("X set complete"));
}




/* --------------------------------------------------------------------
   1. gotoWorldY  ―  목표 Y(mm)까지 직진
      · targetY_mm : 절대 좌표 (월드 기준)
      · tol_mm     : 오차 허용(기본 1mm). 1mm 이내면 이동 생략.
   -------------------------------------------------------------------- */
void gotoWorldY(float targetY_mm, float tol_mm = 1.0f)
{
    float tgt = clampY(targetY_mm);        // 벽 넘어가지 않도록
    float dy  = tgt - y_mm;                // +앞 / –뒤
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




const int REGION_X[7] = { 0,350,1050,350,1050,2600,3050 };


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




int palletDest[7] = {0,0,0,0,0,0,0}; //구역 별 팔레트 목적지, 0번 인덱스는 아마 사용 안 할듯



//const int REGION_X[7] = { 0,350,1050,350,1050,2600,3050 };

bool scanOneZone(uint8_t zoneIdx)
{
    lastQR = 0;                        // 1) QR 초기화
    gotoWorldY(700);                   // 2) Y 중앙선
    gotoWorldX(REGION_X[zoneIdx]);     // 3) X 정렬
    if(zoneIdx == 3 || zoneIdx == 4 ){
        driveUntilFrontGE(-1000, 1300); 
    }else{
        driveUntilFrontLE(1000, 100); 
    }

    if (lastQR != 0) {                 // 5) QR이 실제로 읽혔을 때만
        palletDest[zoneIdx] = lastQR;  //    배열에 기록
        setPoseY();                    //    Y 보정
        return true;                   //    성공
    }
    return false;                      // QR 없음(0) → 실패
}

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



int tmp =0; //임시 저장용

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
  prizm.setMotorInvert(1,1);
  prizm.setMotorInvert(2,1);

  //I2C
  Wire.begin();

  //VL53L0X
  distanceSensor_init(); 

  //odometry thread
  odomThread.onRun(updateOdom);
  odomThread.setInterval(20);   // 50 Hz
  controller.add(&odomThread);
  //setPoseFirst(); //처음 위치를 설정합니다. 
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


    forward(1000);
    //side(1000);
    //driveUntilFront(1000, 500);

    /*
    for(int i = 1; i < 7; i++){ //1 ~ 6구간 모두 스캔 후 palletdest 변수에 저장함.
        tmp = tmp + scanOneZone(i);
        if (tmp == 4) break;
    }



    if(!findNearestMovableRegion(palletDest, x_mm)){
        
    }

*/
    //gotoWorldY(700); //Y축 센터로 이동




/*
    liftUp();
    delay(1000);
    liftDown();
    delay(1000);
*/

/*


    switch (test) //테스트용
    {
        case 0: //앞으로 1미터
            if (!motion.active) {
                startForward(1000);
                test = 1; 
            }
            break;

        case 1:
            float dist = readDistanceSensorFront();  // mm 단위
            if (dist > 0 && dist <= 500) {
                prizm.setMotorSpeeds(0, 0);
                motion.active = false;
                test = 2;
            } else if (!motion.active) {
                test = 2;
                }

            break;
            

        case 2:
            if (!motion.active) {
                prizm.PrizmEnd();
            }

            break;

        case 3:
            if (!motion.active) {
                prizm.PrizmEnd();
            }

            break;
    }
*/
    /*
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
                gotoWorldY(700);
                //startForwardUntil(1000, readDistanceSensorFront(), LE, 500);
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
                startForward(-1000);
                test = 4;
            }
            break;

        case 4: //일단 1미터 가는데, readDistanceSensorFront <= 500이면 멈추기
            if (!motion.active) {
                startSide(-1000);
                test = 5;
            }
            break;

        case 5:
            if (!motion.active) {
                prizm.PrizmEnd();
            }

            break;
    }
*/


/*
    switch (step1_scanPalette) //팔레트 스캔하기
    {
        case 0: //Y축 중심으로 이동
            if (!motion.active) {
                gotoWorldY(700);
                step1_scanPalette = 1; // 다음 스텝으로 넘어가기
            }
            break;                

        case 1: //QR 데이터 초기화
            if (!motion.active) {
                lastQR = 0;
                step1_scanPalette = 2;
            }
            break;

        case 2: //X좌표 맞추기
            if (!motion.active) {
                gotoWorldX(3050); //X = 3050
                step1_scanPalette = 3;
            }
            break;

        case 3: //Y좌표 맞추기 (6구역 진입)
            if (!motion.active) {
                gotoWorldY(1050); //X = 3050
                step1_scanPalette = 4;
            }
            break;

        case 4: //QR 스캐닝, 위치 오차 보정
            if (!motion.active) {
                palletDest[6] = lastQR; //QR을 스캔함
                setPoseY(); // 센서 이용하여 위치 오차 보정
                step1_scanPalette = 5;
            }
            break;

        case 5:
            step2 = 1; // 다시 짤 예정
            break;


    }

    */




    prizm.PrizmEnd();
}