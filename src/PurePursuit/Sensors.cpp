#include "main.h"
/** global variables */
double offset = 0;
Node position;
double encdS = 0, encdR = 0, bearing = 0, angle = halfPI;
double measuredV = 0, measuredVL = 0, measuredVR = 0;
// angle = mathematical angle, taken from x-axis, counter clockwise as positive
void sensors(void * ignore){
   //port data from all sensors
   Motor FLU(FLUPort);
   Motor FLD(FLDPort);
   Motor FRU(FRUPort);
   Motor FRD(FRDPort);
   Imu imu(imuPort);
   Rotation encoderR(encdRPort);
   Rotation encoderS(encdSPort);
   bool calibrated = false;
   int start = millis();
   while(true){
     encdR = encoderR.get_position()/100*inPerDeg; //centidegree /100 = degree
     encdS = encoderS.get_position()/100*inPerDeg;
     bearing = imu.is_calibrating()? 0 : (imu.get_rotation()*toRad + offset*toRad);
     angle = boundRad(halfPI - bearing);
     measuredVL = (FLD.get_actual_velocity() + FLD.get_actual_velocity())/2 * RPMToInPerMs;
     measuredVR = (FRD.get_actual_velocity() + FRD.get_actual_velocity())/2 * RPMToInPerMs;;
     measuredV = (measuredVL + measuredVR)/2;
     delay(5);
   }
}

void setOffset(double i) {
  offset = i;
}
