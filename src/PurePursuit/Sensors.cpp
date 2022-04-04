#include "main.h"
/** global variables */
double offset = 0;
Node position;
double encdS = 0, encdR = 0, bearing = 0, angle = halfPI;
double measuredV = 0, measuredVL = 0, measuredVR = 0;
// angle = mathematical angle, taken from x-axis, counter clockwise as positive
void sensors(void * ignore){
   //port data from all sensors
   Motor BLU(BLUPort);
   Motor BLD(BLDPort);
   Motor BRU(BRUPort);
   Motor BRD(BRDPort);
   Imu imu(imuPort);
   Rotation encoderR(encdRPort);
   Rotation encoderS(encdSPort);
   bool calibrated = false;
   int start = millis();
   while(true){
     encdR = encoderR.get_position()*inPerDeg;
     encdS = encoderS.get_position()*inPerDeg;
     bearing = imu.is_calibrating()? 0 : (imu.get_rotation()*toRad + offset*toRad);
     angle = boundRad(halfPI - bearing);
     measuredVL = (BLD.get_actual_velocity() + BLD.get_actual_velocity())/2 * RPMToInPerMs;
     measuredVR = (BRD.get_actual_velocity() + BRD.get_actual_velocity())/2 * RPMToInPerMs;;
     measuredV = (measuredVL + measuredVR)/2;
     delay(5);
   }
}

void setOffset(double i) {
  offset = i;
}
