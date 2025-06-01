/*
[I2C Adress]
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
1. Huskylens QR, object 학습시키기 (참고 : https://blog.naver.com/icbanq/223079420765)
2. setMotorInvert 하면 엔코더 데이터 값도 음수로 들어오는지 확인해야함
3. 휠 간 거리 측정
4. 센서 거리 측정
5. prizm.PrizmEnd 주석풀기
6. 거리 오차 테스트
7. 가속도 ?

노드 몇 개를 생성하고, 그 노드로 이동하기 ??
*/

//조건부 이동 함수
enum CmdOp { GE, LE, GT, LT, EQ };
/*
  GE : >=
  LE : <=
  GT : >
  LT : <
  EQ : ==
*/

/* 센서 / 상태를 반환하는 함수 포인터 타입 */
typedef float (*ValueFn)();      // 예: readFrontToF(), [](){ return x_mm; }


#include <VL53L0X.h> //pololu
#include <HUSKYLENS.h>
#include <PRIZM.h>
#include <Wire.h>
#include <Thread.h> //ivanseidel
#include <ThreadController.h> //ivanseidel

// address we will assign if dual sensor is present
#define frontToF_ADDRESS 0x30
#define rightToF_ADDRESS 0x31
#define frontToF_XSHUT 6
#define rightToF_XSHUT 7

// objects for the vl53l0x
VL53L0X frontToF = VL53L0X();
VL53L0X rightToF = VL53L0X();
PRIZM prizm;  
ThreadController controller;
Thread odomThread;
HUSKYLENS huskylens;
void printResult(HUSKYLENSResult result);

//robot's state
volatile float x_mm = 0, y_mm = 0, th_deg = 0;
long prevL=0, prevR=0;

/* ─── 하드웨어 파라미터 ───────────────────── */
const float WHEEL_D   = 108.6;               // 휠 지름 [mm]
const float WHEEL_C   = PI * WHEEL_D;        // 휠 거리 [mm]
const float WHEEL_BASE= 100.0;               // 휠 간 거리 [mm] 확인 후 바꿔야함
const int   TICKS_REV = 1440;                // 1회전 당 엔코더 틱 [ticks/rev]
const float MM_PER_TICK = WHEEL_C / TICKS_REV;
const int   world_X = 3400;
const int   world_Y = 1400; 

//sensor offset [mm]
const float frontToF_offset = 0; // 로봇 중심에서 전방 센서까지의 거리 [mm]
const float rightToF_offset = 0; // 로봇 중심에서 우측 센서까지의 거리 [mm]



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

//거리, 각도를 목표로 하는 이동함수
enum TaskMode { IDLE, STRAIGHT, TURN, ARC };

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





/* 새 함수 : 조건 코드와 목표값을 인자로 받음 */
void moveStraightUntil(int16_t speedDeg, ValueFn curValFn, CmdOp op, float refVal)
{
    prizm.setMotorSpeeds(speedDeg, speedDeg);

    while (true) {
        controller.run();                      // 오도메트리·센서 유지

        float cur = curValFn();                // ① 매 주기 현재값 읽기
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




void ToF_setID_LongRangeMode() {

  pinMode(frontToF_XSHUT, OUTPUT);
  pinMode(rightToF_XSHUT, OUTPUT);

  digitalWrite(frontToF_XSHUT, LOW);
  digitalWrite(rightToF_XSHUT, LOW);

  Serial.println(F("Both in reset mode...(pins are low)")); 
  Serial.println(F("Starting..."));

  // all reset
  digitalWrite(frontToF_XSHUT, LOW);    
  digitalWrite(rightToF_XSHUT, LOW);
  delay(10);

  // all unreset
  digitalWrite(frontToF_XSHUT, HIGH);
  digitalWrite(rightToF_XSHUT, HIGH);
  delay(10);

  // activating frontToF and resetting rightToF
  digitalWrite(frontToF_XSHUT, HIGH);
  digitalWrite(rightToF_XSHUT, LOW);

  // initing frontToF
  frontToF.init();
  frontToF.setAddress(frontToF_ADDRESS);
  frontToF.setTimeout(500);

  // activating rightToF
  digitalWrite(rightToF_XSHUT, HIGH);

  //initing rightToF
  rightToF.init();
  rightToF.setAddress(rightToF_ADDRESS);
  frontToF.setTimeout(500);

  //change parameters (long range mode)
  frontToF.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  frontToF.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
  frontToF.setMeasurementTimingBudget(50000);   // 50 ms
  frontToF.setSignalRateLimit(0.1);

  rightToF.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  rightToF.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
  rightToF.setMeasurementTimingBudget(50000);   // 50 ms
  rightToF.setSignalRateLimit(0.1);

  }


  void ToF_setID_NormalMode() {

  pinMode(frontToF_XSHUT, OUTPUT);
  pinMode(rightToF_XSHUT, OUTPUT);

  digitalWrite(frontToF_XSHUT, LOW);
  digitalWrite(rightToF_XSHUT, LOW);

  Serial.println(F("Both in reset mode...(pins are low)")); 
  Serial.println(F("Starting..."));

  // all reset
  digitalWrite(frontToF_XSHUT, LOW);    
  digitalWrite(rightToF_XSHUT, LOW);
  delay(10);
  // all unreset
  digitalWrite(frontToF_XSHUT, HIGH);
  digitalWrite(rightToF_XSHUT, HIGH);
  delay(10);

  // activating frontToF and resetting rightToF
  digitalWrite(frontToF_XSHUT, HIGH);
  digitalWrite(rightToF_XSHUT, LOW);

  // initing frontToF
  frontToF.init();
  frontToF.setAddress(frontToF_ADDRESS);
  frontToF.setTimeout(500);

  // activating rightToF
  digitalWrite(rightToF_XSHUT, HIGH);

  //initing rightToF
  rightToF.init();
  rightToF.setAddress(rightToF_ADDRESS);
  frontToF.setTimeout(500);
  }


//read VL53L0X Distance Sensor 
//센서의 위치를 고려한 거리를 리턴함
//2000[mm] 이상은 Out Of Range임
int16_t readFrontToF(){
  return frontToF.readRangeSingleMillimeters() - frontToF_offset;
}

int16_t readRightToF(){
  return rightToF.readRangeSingleMillimeters() - rightToF_offset;
}





void setFirstPose(){ // 초기 theta값은 90도로 가정
  th_deg = 90;
  x_mm = world_X - readRightToF();
  y_mm = world_Y - readFrontToF();
}

//setX, setY, setTheta, setFirsePose ... >> 입력할경우
//updateX, updateY >> 현재 세타값을 기준으로 X Y pose 업데이트





int16_t readQRdata(){ // 0 : fail
  int data = 0;
  if(!huskylens.request()) return 0;
  if(!huskylens.isLearned()) return 0; 
  if(!huskylens.available()) return 0;

  HUSKYLENSResult result = huskylens.read();

  if (result.ID >= 1 && result.ID <= 6) return result.ID;
  return 0;
}




void setup() {
  //Serial Port
  Serial.begin(115200);

  //PRIZM Board
  prizm.PrizmBegin();

  //I2C
  Wire.begin();

  //Huskylens
  huskylens.begin(Wire);

  //ToF Distance Sensor
  //ToF_setID_NormalMode(); // 약 1m (1000mm) 측정 가능
  ToF_setID_LongRangeMode(); // 약 1.5m (1500mm) 측정 가능

  //odometry thread
  odomThread.onRun(updateOdom);
  odomThread.setInterval(20);   // 50 Hz
  controller.add(&odomThread);



}

void loop() {
  controller.run(); //odometry update

  setFirstPose(); // 초기 theta는 반드시 90도로 맞추어야함


  //거리센서 테스트용
  Serial.print(F("Front Distance [mm] : "));
  Serial.print(readFrontToF());
  Serial.print(F(" "));
  Serial.print(F("Right Distance [mm] : "));
  Serial.println(readRightToF()); 
  delay(10);




  // 테스트 동작을 해보아요
  // moveStraight(mm, speed(-720 ~ 720))
  // turnInPlace(deg, deg/s(-720 ~ 720))
  moveStraight(1000, 720);
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


  //조건부 이동 함수
  moveStraightUntil(720, readFrontToF(), LE, 100); // readFrontTof <= 100이 참이될 때까지 720 deg/s의 속도로 전진
  moveStraightUntil(720, [](){ return x_mm; }, LE, 3150.0); // X <= 3150 일 때까지 전진



  //prizm.PrizmEnd();  //나중에 활성화하기




  /*
  Serial.print(F("X : "));
  Serial.print(x_mm);
  Serial.print(F(" "));
  Serial.print(F("Y : "));
  Serial.print(y_mm);
  Serial.print(F(" "));
  Serial.print(F("Theta : "));
  Serial.println(th_deg);
  */

}
