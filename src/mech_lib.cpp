#include "main.h"

struct GoalHeight {
  double armHeight, tiltHeight;
  bool hasInner;
  double armInnerHeight, tiltInnerHeight;
};
struct GoalHeight tall = {3590,2380,true,3590,2650}, neutral = {2950,2450,true,3050,3000}, alliance = {2700,2900,false,0,0}, init = {2405,1940,false,0,0},selected=init;
bool innerBranch = false, lifted = false, unselected = true;

double armKP = 0.3, armDownKP = 0.15, armHighestKP = 0.1;

double armTarg = init.armHeight, defaultKP = armKP;
double tiltTarg = init.tiltHeight, tiltKP = 0.25;

bool tiltClampState = LOW, armClampState = LOW, canisterState = LOW;

void armControl(void*ignore) {
  Motor armLeft(armLeftPort,E_MOTOR_GEARSET_18,true,E_MOTOR_ENCODER_DEGREES);
	Motor armRight(armRightPort,E_MOTOR_GEARSET_18,false,E_MOTOR_ENCODER_DEGREES);
  ADIDigitalOut clamp(clampPort);
  ADIAnalogIn armPotentiometer(armPotentiometerPort);
  armLeft.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
  armRight.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
  while(true) {
    double armError = armTarg - armPotentiometer.get_value();

    if (armError < 0){
      if (fabs(armError) >=500) defaultKP = armHighestKP;
      else defaultKP = armDownKP;
    } else defaultKP = armKP;


    armLeft.move(armError*defaultKP);
    armRight.move(armError*defaultKP);
    //printf("Target: %f, Potentiometer: %d, Error: %f\n", armTarg, armPotentiometer.get_value(), armError);
    clamp.set_value(armClampState);
    delay(2);
  }
}

void setArmHeight(double height) {armTarg = height;}

void tiltControl(void*ignore) {
  Motor tilt(tiltPort,E_MOTOR_GEARSET_36,true,E_MOTOR_ENCODER_DEGREES);
  ADIAnalogIn tiltPotentiometer(tiltPotentiometerPort);
  ADIDigitalOut tiltLeft(tiltLeftPort);
	ADIDigitalOut tiltRight(tiltRightPort);
  tilt.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
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

void tallSelected(){
  if(!lifted){selected = tall; unselected = false;}
}
void neutralSelected(){
  if(!lifted){selected = neutral; unselected = false;}
}
void allianceSelected(){
  if(!lifted){selected = alliance; unselected = false;}
}

void reset(){
  selected=init;
  lifted = false;
  unselected = true;
  armTarg = selected.armHeight;
  tiltTarg = selected.tiltHeight;
}

void toggleInnerBranch(){
  if (!lifted && !unselected){
    innerBranch = false;
    lifted = true;
  }
  if(selected.hasInner)innerBranch = !innerBranch;

  if (innerBranch && lifted){
    armTarg = selected.armInnerHeight;
    tiltTarg = selected.tiltInnerHeight;
  } else if (!innerBranch && lifted) {
    armTarg = selected.armHeight;
    tiltTarg = selected.tiltHeight;
  }

}

void setArmClampState(bool state) {
  armClampState = state;
}

void toggleArmClampState() {
  armClampState = !armClampState;
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
