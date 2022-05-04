#include "main.h"

const double armHeights[] = {1315,1680,1950,2615};
const double goalHeights[] = {1315,1475,1920,2115};
const double progArmHeights[] = {};
double armKP = 0.6, goalKP = 0.85, armDownKP = 0.3, armKD = 0.01, armKI = 0.01, armTarg = armHeights[0], prevArmError=0;
bool armClampState = LOW, needleState = LOW, batchState = LOW, set = true, armManual = false, toDelay = false;
int count = 0;

/**
4/5/2022:
* Rubber Bands Changed
* armKP, armKD changed.

Notes for tuning (since latex degrades over time)
kP: 0.6-0.7
kD: 0.08 - 0.12
kI: 0.01 - 0.02

downKP: 0.3 - 0.4
goalKP: 0.8 - 1.0

**/

void armControl(void*ignore) {
  Motor armLeft(armLeftPort,E_MOTOR_GEARSET_18,true,E_MOTOR_ENCODER_DEGREES);
	Motor armRight(armRightPort,E_MOTOR_GEARSET_18,false,E_MOTOR_ENCODER_DEGREES);
  armLeft.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
  armRight.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
  ADIDigitalOut clamp(clampPort);
  ADIDigitalOut batch(batchPort);
  ADIDigitalOut needle(needlePort);
  ADIAnalogIn armPotentiometer(armPotentiometerPort);
  ADIDigitalIn armLimit(armLimitPort);
  Controller partner(E_CONTROLLER_PARTNER);

  while(true) {
    double armError = armTarg - armPotentiometer.get_value();
    double deltaError = armError - prevArmError;
    double integral = integral + armError;
    if (fabs(armError) <= 15 || fabs(armError) >= 20)integral=0;
    double armPower;

    //PID
    if (armClampState) armPower = (armError>0?armError*goalKP : armError*armDownKP) + deltaError * armKD;
    else armPower = (armError>0?armError*armKP : armError*armDownKP) + deltaError * armKD + armKI * integral;

    if (armManual) armPower = partner.get_analog(ANALOG_RIGHT_Y);
    else {
      // rate Limiting/magic constants
      if (armTarg == armHeights[0]) armPower = rateLimit(armPower,-100);
      //if (armTarg == armHeights[1]) armPower += 10;
      if (armTarg == armHeights[3]) armPower = rateLimit(armPower,120);
      if (fabs(armError) <= 8) armPower = armPower / 2; //aim to reduce oscillation.
    }

    armLeft.move(armPower);
    armRight.move(armPower);
    prevArmError = armError;
    if (count%10==0) printf("Target: %f, Potentiometer: %d, Error: %f, Power: %f\n", armTarg, armPotentiometer.get_value(), armError, armPower);
    //if (count%10==0) printf("Left Motor Temp: %f, Right Motor Temp: %f\n", armLeft.get_temperature(), armRight.get_temperature());

    //setting pneumatics
    if (!set && !armClampState){
      if (armError> 0 && !needleState){
        batchState = true;
        count = 0;
        toDelay = true;
      }
      set = true;
    }
    if (count == 10 && toDelay){
      needleState = true;
      toDelay = false;
    }
    if (armLimit.get_new_press())armClampState=true;
    count++;

    batch.set_value(batchState);
    clamp.set_value(armClampState);
    needle.set_value(needleState);
    delay(2);
  }

}

void setArmHeight(double height) {armTarg = height;}

void driverArmPos(int pos) {
  armClampState ? armTarg = goalHeights[pos] : armTarg = armHeights[pos];
  set = false;
}

void setArmPos(int pos) {
  armClampState ? armTarg = goalHeights[pos] : armTarg = armHeights[pos];
  set = false;
}

void toggleArmManual(){
  ADIAnalogIn armPotentiometer(armPotentiometerPort);
  armManual = !armManual;
  if(armManual) setArmHeight(armPotentiometer.get_value());
}

void setArmClampState(bool state) {
  armClampState = state;
}

void toggleArmClampState() {
  armClampState = !armClampState;
}

void setBatchState(bool state) {
  batchState = state;
}

void toggleBatchState() {
  batchState = !batchState;
}

void setNeedleState(bool state) {
  needleState = state;
}
void toggleNeedleState() {
  needleState = !needleState;
}

double rateLimit(double input, double limit){
  bool exceed;
  (limit < 0) ? exceed = (input <= limit) : exceed = (input >= limit);
  if (exceed) return limit;
  else return input;
}

void toSet(bool state){
  set = state;
}
