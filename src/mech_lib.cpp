#include "main.h"

const double armHeights[] = {2405,3050,3590};
const double progArmHeights[] = {2405,2700,3050,2995,3590};
double armTarg = armHeights[0], armKP = 0.22, armDownKP = 0.18;
bool armClampState = LOW;

void armControl(void*ignore) {
  Motor armLeft(armLeftPort,E_MOTOR_GEARSET_18,true,E_MOTOR_ENCODER_DEGREES);
	Motor armRight(armRightPort,E_MOTOR_GEARSET_18,false,E_MOTOR_ENCODER_DEGREES);
  ADIDigitalOut clamp(clampPort);
  ADIAnalogIn armPotentiometer(armPotentiometerPort);
  armLeft.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
  armRight.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
  while(true) {
    double kp;
    double armError = armTarg - armPotentiometer.get_value();
    armError < 0 ? kp = armDownKP: kp = armKP;
    armLeft.move(armError*kp);
    armRight.move(armError*kp);
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
