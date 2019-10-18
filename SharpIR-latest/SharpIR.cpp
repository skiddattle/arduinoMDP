#include "Arduino.h"
#include "SharpIR.h"

// Initialisation function
//  + irPin : is obviously the pin where the IR sensor is attached
//  + sensorModel is a int to differentiate the two sensor models this library currently supports:
//    > 1080 is the int for the GP2Y0A21Y and
//    > 20150 is the int for GP2Y0A02YK and
//    > 100500 is the long for GP2Y0A710K0F
//    The numbers reflect the distance range they are designed for (in cm)
SharpIR::SharpIR(int irPin, long sensorModel) {

    _irPin=irPin;
    _model=sensorModel;

    // Define pin as Input
    pinMode (_irPin, INPUT);

    #ifdef ARDUINO
      analogReference(DEFAULT);
    #endif
}

// Sort an array
void SharpIR::sort(int a[], int size) {
    for(int i=0; i<(size-1); i++) {
        bool flag = true;
        for(int o=0; o<(size-(i+1)); o++) {
            if(a[o] > a[o+1]) {
                int t = a[o];
                a[o] = a[o+1];
                a[o+1] = t;
                flag = false;
            }
        }
        if (flag) break;
    }
}

// Read distance and compute it
float SharpIR::distance() {

    int ir_val[NB_SAMPLE];
    int distanceCM;
    float currentdistance;

    delay(40);

    for (int i=0; i<NB_SAMPLE; i++){
        // Read analog value
        ir_val[i] = analogRead(_irPin);
        delayMicroseconds(1500);
    }

    // Sort it
    sort(ir_val,NB_SAMPLE);


    if (_model==1) {

        // Different expressions required as the Photon has 12 bit ADCs vs 10 bit for Arduinos
        #ifdef ARDUINO
        //currentdistance = 4190/(ir_val[NB_SAMPLE/2]-60); // range 9-55cm taken
        currentdistance = (5732/ir_val[NB_SAMPLE/2])-3.745; // range 10-30cm taken

        #elif defined(SPARK)
          distanceCM = 27.728 * pow(map(ir_val[NB_SAMPLE / 2], 0, 4095, 0, 5000)/1000.0, -1.2045);
        #endif

    } else if (_model==2){

        // Previous formula used by  Dr. Marcal Casas-Cartagena
        // puntualDistance=61.573*pow(voltFromRaw/1000, -1.1068);

        // Different expressions required as the Photon has 12 bit ADCs vs 10 bit for Arduinos
        #ifdef ARDUINO
        //currentdistance = 3798/(ir_val[NB_SAMPLE/2]-75); //range 9-50 taken
        currentdistance = (5717.45/ir_val[NB_SAMPLE/2])-7.086; // range 10-30cm taken
        #elif defined(SPARK)
          distanceCM = 60.374 * pow(map(ir_val[NB_SAMPLE / 2], 0, 4095, 0, 5000)/1000.0, -1.16);
        #endif

    } else if (_model==3){

        // Different expressions required as the Photon has 12 bit ADCs vs 10 bit for Arduinos
        #ifdef ARDUINO
        //currentdistance = 4324/(ir_val[NB_SAMPLE/2]-31); //range 9-50 taken
        currentdistance = (5442.33/ir_val[NB_SAMPLE/2])-6.99; // range 10-30cm taken
        #elif defined(SPARK)
          distanceCM = 12.08 * pow(map(ir_val[NB_SAMPLE / 2], 0, 4095, 0, 5000)/1000.0, -1.058);
        #endif

    } else if (_model==4){

        #ifdef ARDUINO
        //currentdistance = 3956/(ir_val[NB_SAMPLE/2]-75); //range 9-30 taken
        //currentdistance = (7457.59/ir_val[NB_SAMPLE/2])-8.76; // range 10-30cm taken
        currentdistance = (5672.279/ir_val[NB_SAMPLE/2])-3.46; // range 10-30cm taken
        #elif defined(SPARK)
          current = map(ir_val[NB_SAMPLE / 2], 0, 4095, 0, 5000);
        #endif
    }else if (_model == 5){
        #ifdef ARDUINO
        //currentdistance = 4134/(ir_val[NB_SAMPLE/2]-67); //range 9-30 taken
        currentdistance = (5823.95/ir_val[NB_SAMPLE/2])-4.011; // range 10-30cm taken
        #endif
    }else if (_model ==6){

        #ifdef ARDUINO
        //range 20-115cm taken
        //currentdistance = 11353.41/(ir_val[NB_SAMPLE/2]-44.9);
        //currentdistance = (15210.28/ir_val[NB_SAMPLE/2])-7.037;
        //currentdistance = (12337.26/ir_val[NB_SAMPLE/2])-9.72;
        currentdistance = (15117.73/ir_val[NB_SAMPLE/2])-24.107;
        #endif // ARDUINO

    }

    return currentdistance;
}




