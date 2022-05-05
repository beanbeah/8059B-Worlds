#ifndef _MECH_LIB_HPP_
#define _MECH_LIB_HPP_

void armControl(void*ignore);
void setArmHeight(double height);
void setArmPos(int pos);
void driverArmPos(int pos,bool state =false);
void setArmClampState(bool state);
void toggleArmClampState();
void setBatchState(bool state);
void toggleBatchState();
void setNeedleState(bool state);
void toggleNeedleState();
double rateLimit(double input, double limit);
int findPosition(const double arr[], int n, double k);
int getNearestPosition();
void toSet(bool state);
void resetLift();
extern bool armClampState, needleState, batchState,armManual;
#endif
