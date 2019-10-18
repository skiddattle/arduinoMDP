char lastRPIcommand;
int orientation = 0;
int vert_counter = 0;
int hori_counter = 0;
int alignThreshold = 5;//4
void setup() {
  Serial.begin(115200);
  setupMotorEncoder();
  setupPID();
  delay(20);

}

boolean readSomething = false;

void loop() {
//  delay(5000);
//  int test =4;
//  while(test>0){
//    rotateRight(2);
//    delay(1500);
//    test--;
//  }
  //////////////////////////////////////////
  delay(2);
   String RPIcommand;
  long int DistNum;
  while (Serial.available()) {
    RPIcommand = readStr();
    DistNum = RPIcommand.substring(1).toInt();
    if (RPIcommand[0] == 'w') {
      moveForward(DistNum);
     if (checkLeftAlign()) {
        wallAlign();
       } 
    if (checkFrontAlign()) {
        alignFront();
    }
//      counter++;
//      if(abs(SharpIR5.distance()-SharpIR4.distance())>=1.5 && SharpIR4.distance()<=10&&SharpIR5.distance()<=10){
//        wallAlign();
//        counter=0;
//      }else if(counter>=3&&SharpIR4.distance()<=10&&SharpIR5.distance()<=10){
//        wallAlign();
//        counter=0;        
//      }
//      if(SharpIR2.distance()<=9&&SharpIR3.distance()<=9){
//        alignFront();
//      }
      lastRPIcommand = 'w';
      done();
    }
    if (RPIcommand[0] == 'd') {
      rotateRight(DistNum);
    if (checkFrontAlign()) {
        alignFront();
    }
      lastRPIcommand = 'd';
      done();
      
    }
    if (RPIcommand[0] == 's') {
        moveBackward(DistNum);
        lastRPIcommand = 's';
      done();
      
    }
    if (RPIcommand[0] == 'a') {
      rotateLeft(DistNum);
      lastRPIcommand = 'a';
      done();
    }
    if (RPIcommand[0] == 'z') {
      sensorReading();
      lastRPIcommand = 'z';
    }
    if (RPIcommand[0] == 'y') {
      rightSensorReading();
      lastRPIcommand = 'y';
    }
    
  }
}

String readStr() {
  String Str = "";
  char Character;
  while (Serial.available()) {
    Character = Serial.read();
    Str.concat(Character);
    delay(10);
  }

  if (Str != "") {
    return Str;
  }
}

void done() {
  Serial.println("done");
}
