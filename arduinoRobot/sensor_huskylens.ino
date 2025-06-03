// Huskylens Pro
// 팔레트 인식도 가능할까 ? ..

void printResult(HUSKYLENSResult result);

int16_t readQRdata(){ //테스트 안 해봄
  int data = 0;
  if(!huskylens.request()) return 0;
  if(!huskylens.isLearned()) return 0; 
  if(!huskylens.available()) return 0;

  HUSKYLENSResult result = huskylens.read();

  if (result.ID >= 1 && result.ID <= 6) return result.ID;
  return 0;
}

