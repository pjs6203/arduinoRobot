/* Implement PID control of TETRIX motors equipped with encoders using the PRIZM controller.
 * The recommended DC motor is the TETRIX TorqueNADO - part number 44260.
 * This example demonstrates how to use position targetting functions to accurately
 * position a DC motor + encoder to an encoder count position and hold the position
 * in a servo like mode. For more detailed information on using the PRIZM library functions,
 * Please see the Arduino Library functions grid in the TETRIX PRIZM Programmer's guide 
 * which can be viewed and downloaded at www.TETRIXRobotics.com
 */


#include <PRIZM.h>    // Include PRIZM Library

PRIZM prizm;          // Instantiate an object named prizm




void setup() {
  Serial.begin(115200);
  prizm.PrizmBegin(); // Initiates the PRIZM controller - must be called in the setup of each PRIZM sketch
  

}

void loop() {

  prizm.setMotorTarget(1,360,1440);   // Spin DC motor 1 at a constant 360 degrees per second and stop when encoder count reaches 1440
  delay(2000);    // wait here for 2 seconds while motor gets to it's destination

  prizm.setMotorTarget(1,360,0);      // Spin DC motor at 360 degrees per second back to encoder position "0".   
  delay(2000);    // wait here for 2 seconds, and repeat                    




  Serial.println(F("setMotorInvert(2,0)"));
  Serial.print(F("motor 1 : "));
  Serial.println(prizm.readEncoderCount(1));
  Serial.print(F("motor 2 : "));
  Serial.println(prizm.readEncoderCount(2));

  prizm.setMotorTarget(1,360,1440);
  prizm.setMotorTarget(2,360,-1440);
  delay(2000);

  prizm.setMotorTarget(1,360,0);
  prizm.setMotorTarget(2,360,0);  
  delay(2000);

  Serial.println(F("setMotorInvert(2,0)"));
  Serial.print(F("motor 1 : "));
  Serial.println(prizm.readEncoderCount(1));
  Serial.print(F("motor 2 : "));
  Serial.println(prizm.readEncoderCount(2));

  Serial.println(F("delay 3 sec"));


  prizm.resetEncoders(); //reset encoder count

  prizm.setMotorInvert(2, 1); // 2번 모터를 invert


  Serial.println(F("setMotorInvert(2,1)"));
  Serial.print(F("motor 1 : "));
  Serial.println(prizm.readEncoderCount(1));
  Serial.print(F("motor 2 : "));
  Serial.println(prizm.readEncoderCount(2));

  prizm.setMotorTarget(1,360,1440);
  prizm.setMotorTarget(2,360,-1440);
  delay(2000);

  prizm.setMotorTarget(1,360,0);
  prizm.setMotorTarget(2,360,0);  
  delay(2000);

  Serial.println(F("setMotorInvert(2,1)"));
  Serial.print(F("motor 1 : "));
  Serial.println(prizm.readEncoderCount(1));
  Serial.print(F("motor 2 : "));
  Serial.println(prizm.readEncoderCount(2));

  Serial.println(F("delay 3 sec"));

}

