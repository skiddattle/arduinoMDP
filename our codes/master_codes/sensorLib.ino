/* ========================= Alignment + SensorLIB =======================================*/
void checkLeftAlign() {
    boolean align;
    
    if (orientation % 2 == 0) {     //facing north or south
        if (hori_counter > alignThreshold) {
            align = true;
        } else { 
            hori_counter++;         
            align = false;
        }
    } else {
        if (vert_counter > alignThreshold) {
            align = true;
        } else { 
            vert_counter++;         
            align = false;
        }
    }

    
    
    //check left left and left right, 5 and 4
    if (align && sensorfive(false)<=10 && sensorfour(true)<=10) {
        wallAlign(&sensortwo, &sensorthree);                    //left and right
        resetLeftAlignCounter();
    }
//    else if (align && RPIcommand[0] == 'w'  && middleLeftisBlock && sensorfour(true)<=10) {       //2by2 align
//        wallAlign(&sensorone, &sensorthree);                    //mid and right
//        resetLeftAlignCounter();
//    }
    
    
    //code to keep track of left blocks
    if (sensorfour(false)<=10) {
        middleLeftisBlock = true;
    } else {
        middleLeftisBlock = false;
    }
}

void checkFrontAlign() {
    boolean align;
    
    if (orientation % 2 == 0) {     //facing north or south
        if (vert_counter > alignThreshold) {
            align = true;
        } else { 
            vert_counter++;         
            align = false;
        }
    } else {
        if (hori_counter > alignThreshold) {
            align = true;
        } else { 
            hori_counter++;         
            align = false;
        }
    }
    //check left and right, 2 and 3
    if (align && sensortwo(false)<=10&& sensorthree(false)<=10) {
        alignFront(&sensortwo, &sensorthree);
        resetFrontAlignCounter();
    } else if (align && sensortwo(false)<=10&& sensorone(false)<=10) {      //check left and mid, 2 and 1
        alignFront(&sensortwo, &sensorone);                                 //order matters
        resetFrontAlignCounter();
    } else if (align && sensorone(false)<=10&& sensorthree(false)<=10) {      //check mid and right, 1 and 3
        alignFront(&sensorone, &sensorthree);
        resetFrontAlignCounter();
    } //else if (align && sensortwo(false)<=23&& sensorthree(false)<=23) {
//      alignUnevenFront(&sensortwo, &sensorthree);
//      resetFrontAlignCounter();
//    } else if (align && sensortwo(false)<=23&& sensorone(false)<=23) {      //check left and mid, 2 and 1
//      alignUnevenFront(&sensortwo, &sensorone);                                 //order matters
//      resetFrontAlignCounter();
//    } else if (align && sensorone(false)<=23&& sensorthree(false)<=23) {      //check mid and right, 1 and 3
//      alignUnevenFront(&sensorone, &sensorthree);
//      resetFrontAlignCounter();
//    }
}

void resetFrontAlignCounter() {
  if (orientation % 2 == 0) {     //facing north or south
      vert_counter = 0;
  } else {
      hori_counter = 0;
  }
}

void resetLeftAlignCounter() {
    if (orientation % 2 == 0) {     //facing north or south
        hori_counter = 0;
    } else {
        vert_counter = 0;
    }
}

void alignLeft(){
   float rotate;
   float alignedl;
    rotate = sensorfive(true) - sensorfour(true);
    //rotate clockwise
    if(rotate>0){
      while(1){
      alignedl = sensorfive(true) - sensorfour(true);
      if(alignedl<=0.4){
        break;
      }  
      md.setSpeeds(0,100);  
      delay(100);
      md.setSpeeds(0, 0);
      md.setBrakes(400, 400);
      delay(5);
      if(sensorfive(true)>10||sensorfour(true)>10){
         break;
      }

    }
  }
  //rotate anti-clockwise
  else if(rotate<0){

      while(1){
      alignedl = sensorfive(true) - sensorfour(true);
      if(alignedl>=-0.4){
        break;
        }  
      md.setSpeeds(0,-100);  
      delay(100);
      md.setSpeeds(0, 0);
      md.setBrakes(400, 400);
      delay(5);
      if(sensorfive(true)>10||sensorfour(true)>10){
         break;
      }
      }
    }
    
}


void alignFront(float (*left)(boolean),float (*right)(boolean)){
  float near;
  float aligned;
  float diff = left(true) - right(true);

  if(diff>0){
      while(1){
      aligned = left(false) - right(false); //changed this from false to true
      if(aligned<=0.2){
        break;
      }
      md.setSpeeds(-100,0);  
      delay(100);
      md.setSpeeds(0, 0);
      md.setBrakes(400, 400);
      delay(5);

      resetSensorsReadings();
      
      if(left(false)>10||right(false)>10){
         break;
      }
    }
  }
  else if(diff<0){

      while(1){
      aligned = left(false) - right(false);//changed this from false to true
      if(aligned>=-0.2){
        break;
      }  
      md.setSpeeds(0,-100);  
      delay(100);
      md.setSpeeds(0, 0);
      md.setBrakes(400, 400);
      delay(5);

      resetSensorsReadings();
      
      if(left(false)>10||right(false)>10){
         break;
      }

    }
  }
  float average = (left(true) + right(true))/2;
  if(average<=4.8){
    while(1){
       near = ((left(true) + right(true))/2);
      if(near>=4.9){
        break;
      }
      moveBackward(99);  


    }
  }else if(average>=5.2){
      while(1){
      near = ((left(true) + right(true))/2);
      if(near<=5){
        break;
      }
      moveForward(99);

    }
  }
   
  //get new raw sensors readings
  resetSensorsReadings();
}

void alignUnevenFront(float (*left)(boolean),float (*right)(boolean)){
  float distL = left(true);
  float distR = right(true);
  int blkL = distL/10;
  int blkR = distR/10;
  float diff = (distL - (blkL*10)) - (distR - (blkR*10));
  //rotation
   while(1){
    if(diff>0.4){
      md.setSpeeds(-100,0);  
      delay(100);
      md.setSpeeds(0, 0);
      delay(5);
      distL = left(true);
      distR = right(true);
      blkL = distL/10;
      blkR = distR/10;
      if (distL>=30 || distR>=30){
        md.setBrakes(400,400);
        resetSensorsReadings();
        delay(5);
        break;
      }
    }
    else if(diff<-0.4){
      md.setSpeeds(0,-100);
      delay(100);
      md.setSpeeds(0,0);
      delay(5);
      distL = left(true);
      distR = right(true);
      blkL = distL/10;
      blkR = distR/10;
      if (distL>=30 || distR>=30){
        md.setBrakes(400,400);
        resetSensorsReadings();
        delay(5);
        break;
      }
    }
    else{
      md.setSpeeds(0,0);
      md.setBrakes(400,400);
      resetSensorsReadings();
      delay(5);
      break;
    }
    diff = (distL-blkL*10)-(distR-blkR*10);
   }
  float average = ((distL-blkL*10) + (distR-blkR*10))/2;
  if(average<=4.8){
    while(1){
       average = ((left(true)-(blkL*10)) + (right(true)-(blkR*10)))/2;
      if(average>=4.9){
        break;
      }
      moveBackward(99);  
    }
  }else if(average>=5.2){
      while(1){
      average = ((left(true)-(blkL*10)) + (right(true)-(blkR*10)))/2;
      if(average<=5){
        break;
      }
      moveForward(99);
    }
  }
   
  //get new raw sensors readings
  resetSensorsReadings();
}

void wallAlign(float (*left)(boolean),float (*right)(boolean)){
  resetLeftAlignCounter();

  rotateLeft(2);
  alignFront(left, right);
  rotateRight(2);
  //alignLeft();        //removed as we are implementing 2-block alignment. If keeping, we have to check sensors are ok before alignment

  //take note of displacement after this alignment 


}

/* ================================ SENSORS ====================================== */
void resetSensorsReadings() {
    //signals that sensors should get raw readings next time.
    prevIR1reading = -1;
    prevIR2reading = -1;
    prevIR3reading = -1;
    prevIR4reading = -1;
    prevIR5reading = -1;
}

float sensorone(boolean useRaw){
  //if less than 0, it means get new sensor reading. 
  if (useRaw || prevIR1reading<0) {
//      SharpIR1.distance() ; //flush sensors
      prevIR1reading = SharpIR1.distance()-0.5;
  }
  
  return prevIR1reading;
}

float sensortwo(boolean useRaw){
  //if less than 0, it means get new sensor reading. 
  if (useRaw || prevIR2reading<0) {
//      SharpIR2.distance() ; //flush sensors
      prevIR2reading = SharpIR2.distance();
  }
  
  return prevIR2reading;
}

float sensorthree(boolean useRaw){
  //if less than 0, it means get new sensor reading. 
  if (useRaw || prevIR3reading<0) {
//      SharpIR3.distance() ; //flush sensors
      prevIR3reading = SharpIR3.distance();
  }
  
  return prevIR3reading;
}

float sensorfour(boolean useRaw){
  //if less than 0, it means get new sensor reading. 
  if (useRaw || prevIR4reading<0) {
//      SharpIR4.distance() ; //flush sensors
      prevIR4reading = SharpIR4.distance()+0.2;
  }
  
  return prevIR4reading;
}

float sensorfive(boolean useRaw){
  //if less than 0, it means get new sensor reading. 
  if (useRaw || prevIR5reading<0) {
//      SharpIR5.distance() ; //flush sensors
      prevIR5reading = SharpIR5.distance() ;
  }
  
  return prevIR5reading;
}

float sensorsix(){
//  SharpIR6.distance() ;
  return SharpIR6.distance() ;
}


void sensorReading(long Sensor) {
  String rawDistance = "";
  String blocks = "";
  int disR, disLTL, disLTR, disFR, disFL, disFM = 0;
  int blkR, blkLTL, blkLTR, blkFR, blkFL, blkFM = 0;
  float newdisFM =0;
  float newdisFL =0;
  float newdisFR =0;
  if (Sensor % 10 == 1) {
    disR = sensorsix(); // this returns the distance for Right
    blkR = rightCal(disR);
  } else {
    disR = -1;
    blkR = -1;
  }
  Sensor /= 10;
  if (Sensor % 10 == 1) {
    disLTL = sensorfive(false); // this returns the distance for Left Top Left
    blkLTL = leftCal(disLTL);
  } else {
    disLTL = -1;
    blkLTL = -1;
  }
  Sensor /= 10;
  if (Sensor % 10 == 1) {
    disLTR = sensorfour(false); // this returns the distance for Left Top Right
    blkLTR = leftCal(disLTR);
  } else {
    disLTR = -1;
    blkLTR = -1;
  }
  Sensor /= 10;
  if (Sensor % 10 == 1) {
    disFR = sensorthree(false); // this returns the distance for Front Right
    blkFR = frontCal(disFR);
  } else {
    blkFR = -1;
    disFR = -1;
  }
  Sensor /= 10;
  if (Sensor % 10 == 1) {
    disFL = sensortwo(false); // this returns the distance for Front Left
    blkFL = frontCal(disFL);
  } else {
    disFL = -1;
    blkFL = -1;
  }
  Sensor /= 10;
  if (Sensor % 10 == 1) {
    disFM = sensorone(false); // this returns the distance for Front Middle
    blkFM = frontMidCal(disFM);
  } else {
    disFM = -1;
    blkFM = -1;
  }
  Sensor /= 10;

  rawDistance.concat(disFM);
  rawDistance.concat(",");
  rawDistance.concat(disFL);
  rawDistance.concat(",");
  rawDistance.concat(disFR);
  rawDistance.concat(",");
  rawDistance.concat(disLTR);
  rawDistance.concat(",");
  rawDistance.concat(disLTL);
  rawDistance.concat(",");
  rawDistance.concat(disR);

  blocks.concat(blkFM);
  blocks.concat(",");
  blocks.concat(blkFL);
  blocks.concat(",");
  blocks.concat(blkFR);
  blocks.concat(",");
  blocks.concat(blkLTR);
  blocks.concat(",");
  blocks.concat(blkLTL);
  blocks.concat(",");
  blocks.concat(blkR);

 // Serial.println(rawDistance);
 Serial.println(blocks);


}
void rightSensorReading(){
  String blocks = "";
  int disRM=SharpIR6.distance();  // this returns the distance for Left Top Left
  int blkRM=rightCal(disRM);
  

  blocks.concat(blkRM);

  Serial.println(blocks);
}

int rightCal(int receiveDFR) {
  int sentBFR = -1;
  if( receiveDFR<=16){
    sentBFR =-1;
  }
  else if (receiveDFR >= 17 && receiveDFR <= 27) {
    sentBFR = 2;
  }
  else if (receiveDFR >= 26 && receiveDFR <= 37) {
    sentBFR = 3;
  }
  else if (receiveDFR >= 36 && receiveDFR <= 47) {
    sentBFR = 4;
  }
  else if (receiveDFR >= 48) {
    sentBFR = 5;
  }
  return sentBFR;
}

int frontCal(int receiveDFR) {
  int sentBFR = -1;
  if( receiveDFR<=8){
    sentBFR =0;
  }
  else if (receiveDFR >= 9 && receiveDFR <= 17) { //16
    sentBFR = 1;
  }
  else if (receiveDFR >= 18 && receiveDFR <= 26) {//17
    sentBFR = 2;
  }
  else if (receiveDFR >= 27) {
    sentBFR = 2;
    //sentBFR = 3;
  }
  return sentBFR;
}


int frontMidCal(int receiveDFR) {
  int sentBFR = -1;
  if( receiveDFR<=8){
    sentBFR =0;
  }
  else if (receiveDFR >= 9 && receiveDFR <= 17) {//16
    sentBFR = 1;
  }
  else if (receiveDFR >= 18 && receiveDFR <= 26) {//17
    sentBFR = 2;
  }
  else if (receiveDFR >= 27) {
    sentBFR = 2;
    //sentBFR = 3;
  }
  return sentBFR;
}

int leftCal(int receiveDFL) {
  int sentBFR = -1;
  if( receiveDFL<=8){
    sentBFR =0;
  }
  else if (receiveDFL >= 9 && receiveDFL <= 17) {
    sentBFR = 1;
  }
  else if (receiveDFL >= 18 && receiveDFL <= 26) {
    sentBFR = 2;
  }
  else if (receiveDFL >= 27) {
    sentBFR = 2;
    //sentBFR = 3;
  }
  return sentBFR;
}
