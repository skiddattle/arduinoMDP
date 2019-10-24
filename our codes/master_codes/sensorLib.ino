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
//    //desperation! use staircase align
//    else if (align && RPIcommand[0] == 'w'  && middleLeftisBlock && sensorfour(true)<=20) {
//        rotateLeft(2);
//        alignStaircase(&sensorone, &sensorthree, true);                    //mid and right
//        rotateRight(2);
//
//        resetLeftAlignCounter();
//    } else if (align && RPIcommand[0] == 'w'  && middleLeftisBlock && sensorfive(true)<=20) {
//        rotateLeft(2);
//        alignStaircase(&sensortwo, &sensorone, false);                    //mid and right
//        rotateRight(2);
//
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
    if (align && sensortwo(false)<=10 && sensorthree(false)<=10) {
        alignFront(&sensortwo, &sensorthree);
        resetFrontAlignCounter();
    } //else if (align && sensortwo(false)<=10 && sensorone(false)<=10) {      //check left and mid, 2 and 1
//        alignFront(&sensortwo, &sensorone);                                 //order matters
//        resetFrontAlignCounter();
//    } else if (align && sensorone(false)<=10 && sensorthree(false)<=10) {      //check mid and right, 1 and 3
//        alignFront(&sensorone, &sensorthree);
//        resetFrontAlignCounter();
//    }
    
//    //desperation! use staircase align
//    else if (align && sensortwo(false)<=10) {
//        if (sensorone(false)<=20) {
//            alignStaircase(&sensortwo, &sensorone, true);
//        }
//        resetFrontAlignCounter();
//    } else if (align && sensorthree(false)<=10) {      //check left and mid, 2 and 1
//        if (sensorone(false)<=20) {
//            alignStaircase(&sensorone, &sensorthree, false);
//        }
//        resetFrontAlignCounter();
//    } else if (align && sensorone(false)<=10) {      //check mid and right, 1 and 3
//        if (sensortwo(false)<=20) {
//            alignStaircase(&sensortwo, &sensorone, false);
//        } else if (sensorthree(false)<=20) {
//            alignStaircase(&sensorone, &sensorthree, true);
//        }
//        resetFrontAlignCounter();
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
  
  while(left(true)<5.5 || right(true)<5.5){
    md.setSpeeds(-200,-200);
    delay(50);
    md.setSpeeds(0,0);
  }
  float diff = left(false) - right(false);

  if(diff>0){
      while(1){
      aligned = left(false) - right(false); //changed this from false to true
      if(aligned<=0.2){
        break;
      }
      md.setSpeeds(-150,0);  
      delay(50);
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
      md.setSpeeds(150,0);  
      delay(50);
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

      moveBackward(99);  

      resetSensorsReadings();
      
      near = ((left(false) + right(false))/2);
      if(left(false)>10||right(false)>10){
         break;
      }
      if(near>=4.9){
        break;
      }

    }
  }else if(average>=5.2){
      while(1){

      moveForward(99);

      resetSensorsReadings();
      
      near = ((left(false) + right(false))/2);
      if(left(false)>10||right(false)>10){
         break;
      }
      if(near<=5){
        break;
      }
    }
  }
   
  //get new raw sensors readings
  resetSensorsReadings();
}

void alignStaircase(float (*left)(boolean),float (*right)(boolean), boolean leftisNear){
  float flatdist = 10;
  if (leftisNear) {
      flatdist = flatdist * -1;         //flip flat distance to negative
  }
    
  float near;
  float aligned;
  float diff = left(true) - right(true) - flatdist;

  if(diff>0){
      while(1){
      aligned = left(false) - right(false) - flatdist; 
      if(aligned<=0.5){
        break;
      }
      
      //HI BRYAN! FIX CALIBRATION BEFORE FIXING ;
      md.setSpeeds(-150,0);//HI BRYAN! FIX CALIBRATION 
      delay(50);
      md.setSpeeds(0, 0);
      md.setBrakes(400, 400);
      delay(5);

      resetSensorsReadings();
      
      if (leftisNear && left(false)>10) {
          break;
      }
      else if (!leftisNear && right(false)>10) {
          break;
      }
    }
  }
  else if(diff<0){

      while(1){
      aligned = left(false) - right(false) - flatdist;
      if(aligned>=-0.5){
        break;
      }  
      
      //HI BRYAN! FIX CALIBRATION
      md.setSpeeds(150,0);//HI BRYAN! FIX CALIBRATION
      delay(50);
      md.setSpeeds(0, 0);
      md.setBrakes(400, 400);
      delay(5);

      resetSensorsReadings();
      
      if (leftisNear && left(false)>10) {
          break;
      }
      else if (!leftisNear && right(false)>10) {
          break;
      }

    }
  }
  
  //displacement
  float average = (left(true) + right(true) - abs(flatdist) )/2 ;
  if(average<=4.8){
    while(1){
      near = ((left(true) + right(true) - abs(flatdist) )/2) ;
      
      if (leftisNear && left(false)>10) {
          break;
      }
      else if (!leftisNear && right(false)>10) {
          break;
      }
      if(near>=4.9){
        break;
      }
      moveBackward(99);  


    }
  }else if(average>=5.2){
      while(1){
      near = ((left(true) + right(true) - abs(flatdist) )/2);
      if (leftisNear && left(false)>10) {
          break;
      }
      else if (!leftisNear && right(false)>10) {
          break;
      }
      if(near<=5){
        break;
      }
      moveForward(99);

    }
  }
   
  //get new raw sensors readings
  resetSensorsReadings();
}


void wallAlign(float (*left)(boolean),float (*right)(boolean)){
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
      if (useRaw){SharpIR1.distance() ;} //flush sensors
      prevIR1reading = SharpIR1.distance();//-0.5
  }
  
  return prevIR1reading;
}

float sensortwo(boolean useRaw){
  //if less than 0, it means get new sensor reading. 
  if (useRaw || prevIR2reading<0) {
      if (useRaw){SharpIR2.distance() ;} //flush sensors
      prevIR2reading = SharpIR2.distance();
  }
  
  return prevIR2reading;
}

float sensorthree(boolean useRaw){
  //if less than 0, it means get new sensor reading. 
  if (useRaw || prevIR3reading<0) {
      if (useRaw){SharpIR3.distance() ;} //flush sensors
      prevIR3reading = SharpIR3.distance();
  }
  
  return prevIR3reading;
}

float sensorfour(boolean useRaw){
  //if less than 0, it means get new sensor reading. 
  if (useRaw || prevIR4reading<0) {
      if (useRaw){SharpIR4.distance() ;} //flush sensors
      prevIR4reading = SharpIR4.distance();
  }
  
  return prevIR4reading;
}

float sensorfive(boolean useRaw){
  //if less than 0, it means get new sensor reading. 
  if (useRaw || prevIR5reading<0) {
      if (useRaw){SharpIR5.distance() ;} //flush sensors
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
    disLTL = sensorfive(true); // this returns the distance for Left Top Left
    blkLTL = leftCal(disLTL);
  } else {
    disLTL = -1;
    blkLTL = -1;
  }
  Sensor /= 10;
  if (Sensor % 10 == 1) {
    disLTR = sensorfour(true); // this returns the distance for Left Top Right
    blkLTR = leftCal(disLTR);
  } else {
    disLTR = -1;
    blkLTR = -1;
  }
  Sensor /= 10;
  if (Sensor % 10 == 1) {
    disFR = sensorthree(true); // this returns the distance for Front Right
    blkFR = frontCal(disFR);
  } else {
    blkFR = -1;
    disFR = -1;
  }
  Sensor /= 10;
  if (Sensor % 10 == 1) {
    disFL = sensortwo(true); // this returns the distance for Front Left
    blkFL = frontCal(disFL);
  } else {
    disFL = -1;
    blkFL = -1;
  }
  Sensor /= 10;
  if (Sensor % 10 == 1) {
    disFM = sensorone(true); // this returns the distance for Front Middle
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
