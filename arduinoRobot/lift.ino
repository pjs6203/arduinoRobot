


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