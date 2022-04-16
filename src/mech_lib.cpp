#include "main.h"

const double armHeights[] = {0,1,2,3};
const double progArmHeights[] = {0,0,0,0};
double armTarg = armHeights[0], armKP = 0.5;
const double tiltHeights[] = {1940,2350,2600,2800};
const double progTiltHeights[] = {1940,2350,2600,2800};
double tiltTarg = tiltHeights[0], tiltKP = 0.001;
bool tiltClampState = LOW, armClampState = LOW, canisterState = LOW;

void armControl(void*ignore) {
  Motor armLeft(armLeftPort,E_MOTOR_GEARSET_18,true,E_MOTOR_ENCODER_DEGREES);
	Motor armRight(armRightPort,E_MOTOR_GEARSET_18,false,E_MOTOR_ENCODER_DEGREES);
  ADIDigitalOut clamp(clampPort);
  ADIAnalogIn armPotentiometer(armPotentiometerPort);
  while(true) {
    double armError = armTarg - armPotentiometer.get_value();
    armLeft.move(armError*armKP);
    armRight.move(armError*armKP);
    //printf("Target: %f, Potentiometer: %d, Error: %f\n", armTarg, armPotentiometer.get_value(), armError);
    clamp.set_value(armClampState);
    delay(2);
  }
}

void setArmHeight(double height) {armTarg = height;}

void driverArmPos(int pos) {
  armTarg = armHeights[pos];
}

void setArmPos(int pos) {
  armTarg = progArmHeights[pos];
}

void setArmClampState(bool state) {
  armClampState = state;
}

void toggleArmClampState() {
  armClampState = !armClampState;
}

void tiltControl(void*ignore) {
  Motor tilt(tiltPort,E_MOTOR_GEARSET_36,true,E_MOTOR_ENCODER_DEGREES);
  ADIAnalogIn tiltPotentiometer(tiltPotentiometerPort);
  ADIDigitalOut tiltLeft(tiltLeftPort);
	ADIDigitalOut tiltRight(tiltRightPort);
  while(true) {
    double tiltError = -(tiltTarg - tiltPotentiometer.get_value());
    tilt.move(tiltError*armKP);
    printf("Target: %f, Potentiometer: %d, Error: %f\n", tiltTarg, tiltPotentiometer.get_value(), tiltError);
    tiltLeft.set_value(tiltClampState);
    tiltRight.set_value(tiltClampState);
    delay(2);
  }
}

void setTiltHeight(double height) {tiltTarg = height;}

void driverTiltPos(int pos) {
  tiltTarg = tiltHeights[pos];
}

void setTiltPos (int pos) {
  tiltTarg = progTiltHeights[pos];
}

void setTiltClampState(bool state) {
  tiltClampState = state;
}

void toggleTiltClampState() {
  tiltClampState = !tiltClampState;
}

void canisterControl(void*ignore) {
  ADIDigitalOut canisterL(canisterLeftPort);
	ADIDigitalOut canisterR(canisterRightPort);
  while(true) {
    canisterL.set_value(canisterState);
    canisterR.set_value(canisterState);
    delay(5);
  }
}

void setCanisterState(bool state){
  canisterState = state;
}
void toggleCanisterState() {
  canisterState = !canisterState;
}

void release(double delayTime) {
  setCanisterState(true);
  delay(delayTime);
  setCanisterState(false);
}
