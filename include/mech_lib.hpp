#ifndef _MECH_LIB_HPP_
#define _MECH_LIB_HPP_

void armControl(void*ignore);
void setArmHeight(double height);
void setArmPos(int pos);
void driverArmPos(int pos);
void toggleArmManual();
void setArmClampState(bool state);
void toggleArmClampState();
void setBatchState(bool state);
void toggleBatchState();
void setNeedleState(bool state);
void toggleNeedleState();
double rateLimit(double input, double limit);
void toSet(bool state);

extern bool armClampState, needleState, batchState;
#endif
