/* ========================= MOTORLIB =======================================*/
const int LEFT_PULSE = 3;
const int RIGHT_PULSE = 11; 
const int MOVE_FAST_SPEED = 375;
const int MOVE_MAX_SPEED = 310; 
const int MOVE_MIN_SPEED = 200;
const int TURN_MAX_SPEED = 260;
const int ROTATE_MAX_SPEED = 250;//150
const int TURN_TICKS_L = 770;
const int TURN_TICKS_R = 763;
const int TICKS[10] = {545, 1155, 1760, 2380, 2985, 3615, 4195, 4775, 5370};
const double DIST_WALL_CENTER_BOX = 1.58;
const double kp = 1, ki =0.7, kd =0;

int TENCM_TICKS_OFFSET = 0;

double tick_R = 0;
double tick_L = 0;
double speed_O = 0;
double previous_tick_R = 0;
double previous_error = 0;

DualVNH5019MotorShield md;
PID myPID(&tick_R, &speed_O, &tick_L, kp, ki, kd, REVERSE);

////--------------------------Motor Codes-------------------------------
void setupMotorEncoder() {
  md.init();
  pinMode(LEFT_PULSE, INPUT);
  pinMode(RIGHT_PULSE, INPUT);
  enableInterrupt(LEFT_PULSE, leftMotorTime, CHANGE);
  enableInterrupt(RIGHT_PULSE, rightMotorTime, CHANGE);
}

void stopMotorEncoder() {
  disableInterrupt(LEFT_PULSE);
  disableInterrupt(RIGHT_PULSE);
}

void setupPID() {
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-370, 370); 
  myPID.SetSampleTime(5);
}
//
void moveForward(int blockstomove) {
  initializeTick();
  initializeMotor_Start();
  int tickstomove;
  switch (blockstomove) {
    case 1:
      {
          tickstomove = 545;//275

        break;
      }
    case 2:
      {
        tickstomove = 1155;
        break;
      }
    case 3:
      {
        tickstomove = 1760;
        break;
      }
    case 4:
      {
        tickstomove = 2380;
        break;
      }
    case 5:
      {
        tickstomove = 2985;
        break;
      }
    case 6:
      {
        tickstomove = 3615;
        break;
      }
    case 7:
      {
        tickstomove = 4195;
        break;
      }
    case 8:
      {
        tickstomove = 4775;
        break;
      }
    case 9:
      {
        tickstomove = 5370;
        break;
      }
    case 99:
    {
        tickstomove = 26;
        break;
  }
 }
//  distance = cmToTicks(distance);
//  double currentSpeed = 0;
//  if (tickstomove < 60) {
//    currentSpeed = MOVE_MIN_SPEED;
//  } else {
//    currentSpeed = MOVE_MAX_SPEED;
//  }
//  while (tick_R <= tickstomove || tick_L <= tickstomove) {
//    md.setSpeeds(currentSpeed-16,currentSpeed);
//
//    
//    
//  }

  double currentSpeed = 0;
  if (tickstomove < 60) {
    currentSpeed = MOVE_MIN_SPEED;
  } else {
    currentSpeed = MOVE_MAX_SPEED;
  }
  double offset = 0;
  long last_tick_R = 0;
  while (tick_R <= tickstomove || tick_L <= tickstomove) {
    if ((tick_R - last_tick_R) >= 10 || tick_R == 0 || tick_R == last_tick_R) {
      last_tick_R = tick_R;
      offset += 0.1;
    }
    if (myPID.Compute() || tick_R == last_tick_R) {
      if (offset >= 1)
        md.setSpeeds((currentSpeed + speed_O), (currentSpeed - speed_O));
      else
        md.setSpeeds((offset * (currentSpeed + speed_O)), (offset * (currentSpeed - speed_O)));
    }
  }

  initializeMotor_End();
}
//
void moveBackward(int blockstomove) {
  initializeTick();
  initializeMotor_Start();
//  distance = cmToTicks(distance);
  int tickstomove;
  switch (blockstomove) {
    case 1:
      {
          tickstomove = 600;
      }
    case 2:
      {
        tickstomove = 540;
        break;
      }
    case 3:
      {
        tickstomove = 830;
        break;
      }
    case 4:
      {
        tickstomove = 1130;
        break;
      }
    case 5:
      {
        tickstomove = 1410;
        break;
      }
    case 6:
      {
        tickstomove = 1720;
        break;
      }
    case 7:
      {
        tickstomove = 2030;
        break;
      }
    case 8:
      {
        tickstomove = 4775;
        break;
      }
    case 9:
      {
        tickstomove = 2600;
        break;
      }
    case 10:
      {
        tickstomove = 2930;
        break;
      }
    case 11:
      {
        tickstomove = 3230;
        break;
      }
    case 12:
      {
        tickstomove = 3500;
        break;
      }
    case 13:
      {
        tickstomove = 3800;
        break;
      }
    case 14:
      {
        tickstomove = 4120;
        break;
      }
    case 15:
      {
        tickstomove = 4350;
        break;
      }
    case 99:
    {
        tickstomove = 20;
        break;
  }
  }
  double currentSpeed = 0;
  if (tickstomove < 60) {
    currentSpeed = MOVE_MIN_SPEED;
  } else {
    currentSpeed = MOVE_MAX_SPEED;
  }
  double offset = 0;
  long last_tick_R = 0;
  while (tick_R <= tickstomove || tick_L <= tickstomove) {
    if ((tick_R - last_tick_R) >= 10 || tick_R == 0 || tick_R == last_tick_R) {
      last_tick_R = tick_R;
      offset += 0.1;
    }
    if (myPID.Compute() || tick_R == last_tick_R) {
      if (offset >= 1)
        md.setSpeeds(-(currentSpeed + speed_O), -(currentSpeed - speed_O));
      else
        md.setSpeeds(-(offset * (currentSpeed + speed_O)), -(offset * (currentSpeed - speed_O)));
    }
  }
  initializeMotor_End();
}
//
//
//
void rotateLeft(int degreetomove) {
  initializeTick();
  initializeMotor_Start();
  int tickstomove = 0;
  if (degreetomove == 1) {
    tickstomove = 105; //45 degrees
  }
  else if (degreetomove == 2) {
    tickstomove = 775;//377 //90 degrees
  }
  else if (degreetomove == 3) {//270
    tickstomove = 720; //180 degrees
  }
  else if (degreetomove == 99) {
    tickstomove = 5; //180 degrees
  }
  double currentSpeed = ROTATE_MAX_SPEED;
  double offset = 0;
  if (tickstomove < 3)
    return;
  while (tick_R < tickstomove || tick_L < tickstomove) {
    //    offset = computePID();
    if (myPID.Compute())
      md.setSpeeds((currentSpeed + speed_O), -(currentSpeed - speed_O));
  }
  initializeMotor_End();
}

void rotateRight(int degreetomove) {
  initializeTick();
  initializeMotor_Start();
  int tickstomove = 0;
  if (degreetomove == 1) {
    tickstomove = 105;//45 degrees
  }
  else if (degreetomove == 2) {
    tickstomove = 771; //90 degrees
  }
  else if (degreetomove == 3) {
    tickstomove = 720; //180 degrees
  }
  else if (degreetomove == 99) {
    tickstomove = 5; //180 degrees
  }
  double currentSpeed = ROTATE_MAX_SPEED;
  double offset = 0;
  if (tickstomove < 3)
    return;
  while (tick_R < tickstomove || tick_L < tickstomove) {
    if (myPID.Compute())
     md.setSpeeds(-(currentSpeed + speed_O), currentSpeed - speed_O);
  }
  initializeMotor_End();
}
//
//
//double getMin(double f1, double f2, double f3) {
//  if (f1 < f2) {
//    if (f1 < f3) {
//      return f1;
//    } else {
//      return f3;
//    }
//  } else {
//    if (f2 < f3) {
//      return f2;
//    } else {
//      return f3;
//    }
//  }
//}
//
void leftMotorTime() {
  tick_L++;
}

void rightMotorTime() {
  tick_R++;
}

void initializeTick() {
  tick_R = 0;
  tick_L = 0;
  speed_O = 0;
  previous_tick_R = 0;
}

void initializeMotor_Start() {
  md.setSpeeds(0, 0);
  md.setBrakes(0, 0);
}

void initializeMotor_End() {
  md.setSpeeds(0, 0);
  md.setBrakes(400, 350);
  delay(5);
}
