#include "main.h"

struct Goal tall = {
  3590,2380,  //armHeight, tiltHeight
  true,       //has Inner Branch
  3590,2650   //armInnerHeight, tiltInnerHeight
  //upKP, downKP, changeKP
};
struct Goal neutral = {
  2950,2450,  //armHeight, tiltHeight
  true,       //has Inner Branch
  3050,3000   //armInnerHeight, tiltInnerHeight
  //upKP, downKP, changeKP
};
struct Goal alliance = {
  2700,2900,  //armHeight, tiltHeight
  false,      //has Inner Branch
  0,0         //armInnerHeight, tiltInnerHeight
  //upKP, downKP, changeKP
};
struct Goal init = {
  2405,1940,  //armHeight, tiltHeight
  false,      //has Inner Branch
  0,0,        //armInnerHeight, tiltInnerHeight
  0,0,0       //upKP, downKP, changeKP
};

struct Goal selected=init;
bool innerBranch = false, lifted = false, unselected = true, tiltAtInit = false;
double leeway = 12.5;
double armTarg = init.armHeight, armKP = 0.2;
double tiltTarg = init.tiltHeight, tiltKP = 0.2;
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
    armLeft.move(armError*armKP);
    armRight.move(armError*armKP);
    //printf("Target: %f, Potentiometer: %d, Error: %f\n", armTarg, armPotentiometer.get_value(), armError);
    clamp.set_value(armClampState);
    if (!lifted)fabs(armError) <= leeway ? lifted = true : lifted = false;
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
    tilt.move(tiltError*tiltKP);
    printf("Target: %f, Potentiometer: %d, Error: %f\n", tiltTarg, tiltPotentiometer.get_value(), tiltError);
    tiltLeft.set_value(tiltClampState);
    tiltRight.set_value(tiltClampState);
    if (tiltTarg == init.tiltHeight)fabs(tiltError) <= leeway ? tiltClampState = false : tiltClampState = true;
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
  armKP = selected.downKP;
  selected=init;
  lifted = false;
  unselected = true;
  armTarg = selected.armHeight;
  tiltTarg = selected.tiltHeight;
}

void toggleInnerBranch(){
  if (!lifted && !unselected){
    innerBranch = false;
    armKP = selected.upKP;
    armTarg = selected.armHeight;
    tiltTarg = selected.tiltHeight;
  }
  if(selected.hasInner)innerBranch = !innerBranch;

  if (innerBranch && lifted){
    armTarg = selected.armInnerHeight;
    tiltTarg = selected.tiltInnerHeight;
    armKP = selected.changeKP;
  } else if (!innerBranch && lifted) {
    armTarg = selected.armHeight;
    tiltTarg = selected.tiltHeight;
    armKP = selected.changeKP;
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
