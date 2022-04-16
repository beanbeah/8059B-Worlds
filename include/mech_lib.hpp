#ifndef _MECH_LIB_HPP_
#define _MECH_LIB_HPP_

void armControl(void*ignore);
void setArmHeight(double height);
void setArmPos(int pos);
void driverArmPos(int pos);
void setArmClampState(bool state);
void toggleArmClampState();

void tiltControl(void*ignore);
void setTiltHeight(double height);
void setTiltPos (int pos);
void driverTiltPos(int pos);
void setTiltClampState(bool state);
void toggleTiltClampState();

void canisterControl(void*ignore);
void setCanisterState(bool state);
void toggleCanisterState();
void release(double delayTime);
#endif
