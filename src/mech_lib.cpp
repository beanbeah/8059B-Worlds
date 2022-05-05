#include "main.h"

const double armHeights[] = {0,34,56,107.50};
const double goalHeights[] = {0,10,55,75};
const double progArmHeights[] = {};
double armKP = 4.85, armDownKP = 4.5, goalKP = 6.5, armKD = 0.1, armKI = 0, armTarg = armHeights[0], prevArmError=0;
bool armClampState = LOW, needleState = LOW, batchState = LOW, set = true, armManual = false, toDelay = false;
int count = 0;

/**
kP: 4.5 - 5.2
downKP: 4.85 (gives best results)
kD: 0.1 - 0.6 (theoretical max based on kP/8)
kI: 0 (seems to be fine)
goalKP:

//magic constants:
Note: kP of 4.85 gives CONSISTENT error of max -2 deg of rotation
*/

void armControl(void*ignore) {
  Motor armLeft(armLeftPort,E_MOTOR_GEARSET_18,true,E_MOTOR_ENCODER_DEGREES);
	Motor armRight(armRightPort,E_MOTOR_GEARSET_18,false,E_MOTOR_ENCODER_DEGREES);
  armLeft.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
  armRight.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
  ADIDigitalOut clamp(clampPort);
  ADIDigitalOut batch(batchPort);
  ADIDigitalOut needle(needlePort);
  //ADIAnalogIn armPotentiometer(armPotentiometerPort);
  Rotation armRot(armRotPort);
  ADIDigitalIn armLimit(armLimitPort);

  Controller partner(E_CONTROLLER_PARTNER);
  bool init = true;
  int tick = 0;

  while(true) {
    double armError = armTarg - (armRot.get_position()/100);
    double deltaError = armError - prevArmError;
    double integral = integral + armError;
    if (fabs(armError) <= 12 || fabs(armError) >= 20)integral=0;
    double armPower;

    //PID
    if (armClampState) armPower = (armError>0?armError*goalKP : armError*armDownKP) + deltaError * armKD;
    else armPower = (armError>0?armError*armKP : armError*armDownKP - 8) + deltaError * armKD + armKI * integral;


    if (partner.get_digital_new_press(DIGITAL_X)) armManual = !armManual;
    if (armManual){
      armPower = partner.get_analog(ANALOG_RIGHT_Y) / 1.5;
      armTarg = armRot.get_position();
    }
    else {
      // rate Limiting/magic constants
      if (armTarg == armHeights[0]) armPower = rateLimit(armPower,-100);
      //if (armTarg == armHeights[1]) armPower += 10;
      if (armTarg == armHeights[3]) armPower = rateLimit(armPower,100);
       //if (fabs(armError) <= 8) {
        // armPower = armPower/3; //aim to reduce oscillation.
      //   armTarg = armPotentiometer.get_value();
       //}
    }


    armLeft.move(armPower);
    armRight.move(armPower);
    prevArmError = armError;
    if (count%10==0 && !armManual) printf("Target: %f, Potentiometer: %d, Error: %f, Power: %f\n", armTarg, armRot.get_position()/100, armError, armPower);
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

void driverArmPos(int pos, bool state) {
  armClampState ? armTarg = goalHeights[pos] : armTarg = armHeights[pos];
  set = state;
}

void setArmPos(int pos) {
  armClampState ? armTarg = goalHeights[pos] : armTarg = armHeights[pos];
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

int findPosition(const double arr[], int n, double k){
  int index;
  for (int i = 0; i<n; i++){
    if (k == arr[i])index = i;
    else if (k >= arr[i])index = i;
  }
  return index;
}

int getNearestPosition() {
  return findPosition(armHeights, 4, armTarg);
}

void toSet(bool state){
  set = !state;
}
