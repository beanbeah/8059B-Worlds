#ifndef _MECH_LIB_HPP_
#define _MECH_LIB_HPP_

void armControl(void*ignore);
void setArmHeight(double height);
void setArmPos(int pos);
void driverArmUp();
void driverArmDown();
void setArmClampState(bool state);
void toggleArmClampState();
void setBatchState(bool state);
void toggleBatchState();
void setNeedleState(bool state);
void toggleNeedleState();
double rateLimit(double input, double limit);
void toSet(bool state);
int findPosition(const double arr[], int n, double k);
extern bool armClampState, needleState, batchState,armManual;
#endif
