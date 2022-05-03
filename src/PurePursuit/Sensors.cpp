#include "main.h"
/** global variables */
double offset = 0;
Node position;
double encdS = 0, encdR = 0, bearing = 0, angle = halfPI;
double measuredV = 0, measuredVL = 0, measuredVR = 0;
// angle = mathematical angle, taken from x-axis, counter clockwise as positive
void sensors(void * ignore){
   //port data from all sensors
   Motor FL1(FL1Port);
   Motor FL2(FL2Port);
   Motor FL3(FL3Port);
   Motor FR1(FR1Port);
   Motor FR2(FR2Port);
   Motor FR3(FR3Port);
   Imu imu(imuPort);
   Rotation encoderR(encdRPort);
   Rotation encoderS(encdSPort);
   int start = millis();
   while(true){
     encdR = encoderR.get_position()/100*inPerDeg; // centidegree / 100 = degree
     encdS = encoderS.get_position()/100*inPerDeg;
     bearing = imu.is_calibrating()? 0 : (imu.get_rotation()*toRad + offset*toRad);
     angle = boundRad(halfPI - bearing);
     measuredVL = (FL1.get_actual_velocity() + FL2.get_actual_velocity() + FL3.get_actual_velocity())/3 * RPMToInPerMs;
     measuredVR = (FR1.get_actual_velocity() + FR2.get_actual_velocity() + FR3.get_actual_velocity())/3 * RPMToInPerMs;;
     measuredV = (measuredVL + measuredVR)/2;
     delay(5);
   }
}

void setOffset(double i) {
  offset = i;
}
