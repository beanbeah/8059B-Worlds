#include "main.h"

const double armHeights[] = {1340,1650,1920,2555};
const double progArmHeights[] = {1340,1650,1920,2555};
double armHighestKP = 0.25, armKP = 1, armDownKP = 0.05, armKD = 0.15;
double armTarg = armHeights[0], prevArmError=0;
double leeway = 15;
bool armClampState = LOW, needleState = LOW, batchState = LOW, set = true;

void armControl(void*ignore) {
  Motor armLeft(armLeftPort,E_MOTOR_GEARSET_18,true,E_MOTOR_ENCODER_DEGREES);
	Motor armRight(armRightPort,E_MOTOR_GEARSET_18,false,E_MOTOR_ENCODER_DEGREES);
  ADIDigitalOut clamp(clampPort);
  ADIDigitalOut batch(batchPort);
  ADIDigitalOut needle(needlePort);
  ADIAnalogIn armPotentiometer(armPotentiometerPort);
  armLeft.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
  armRight.set_brake_mode(E_MOTOR_BRAKE_BRAKE);

  while(true) {
    double armError = armTarg - armPotentiometer.get_value();
    double deltaError = armError - prevArmError;
    double kp;
    if (armError > 0)(armTarg == armHeights[3]) ? kp = armHighestKP : kp = armKP;
    else kp = armDownKP;
    double armPower = armError * kp + deltaError * armKD;
    armLeft.move(armPower);
    armRight.move(armPower);
    printf("Target: %f, Potentiometer: %d, Error: %f, armKP: %f\n", armTarg, armPotentiometer.get_value(), armError, kp);
    clamp.set_value(armClampState);
    if (!set){
      if (armError> 0 && !needleState)needleState=true;
      else if (armTarg == armHeights[0] && needleState)needleState=false;
      set = true;
    }
    needle.set_value(needleState);
    batch.set_value(batchState);
    delay(2);
  }

}

void setArmHeight(double height) {armTarg = height;}

void driverArmPos(int pos) {
  armTarg = armHeights[pos];
  set = false;
}

void setArmPos(int pos) {
  armTarg = progArmHeights[pos];
  set = false;
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
