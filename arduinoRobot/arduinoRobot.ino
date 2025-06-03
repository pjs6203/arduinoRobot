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

// objects
PRIZM prizm;
ThreadController controller;
Thread odomThread;
VL53L0X distanceSensorFront;
VL53L0X distanceSensorRight;
HUSKYLENS huskylens;

//state
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

void setup() 
{
  //Serial Port
  Serial.begin(115200);

  //PRIZM Board
  prizm.PrizmBegin();
  prizm.setMotorInvert(2,1); //2번 모터를 반전

  //I2C
  Wire.begin();

  //Huskylens
  huskylens.begin(Wire);

  //odometry thread
  odomThread.onRun(updateOdom);
  odomThread.setInterval(20);   // 50 Hz
  controller.add(&odomThread);
}

void loop() 
{
  controller.run(); //odometry update
  //setFirstPose(); // 초기 theta는 반드시 90도로 맞추어야함
  distanceSensor_init(); 

  while(true)
  {
    //거리센서 테스트용
    Serial.print(F("Front Distance [mm] : "));
    Serial.print(readDistanceSensorFront());
    delay(5);
    Serial.print(F(" "));
    Serial.print(F("Right Distance [mm] : "));
    Serial.println(readDistanceSensorRight()); 
    delay(50);
  }

/*
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
  moveStraightUntil(720, readDistanceSensorFront(), LE, 100); // readDistanceSensorFront <= 100이 참이될 때까지 720 deg/s의 속도로 전진
  moveStraightUntil(720, [](){ return x_mm; }, LE, 3150.0); // X <= 3150 일 때까지 전진



  //prizm.PrizmEnd();  //나중에 활성화하기

*/


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
