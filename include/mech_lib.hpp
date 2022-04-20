#ifndef _MECH_LIB_HPP_
#define _MECH_LIB_HPP_

struct Goal {
  double armHeight, tiltHeight;
  bool hasInner;
  double armInnerHeight, tiltInnerHeight;
  double upKP, downKP, changeKP;
};

void armControl(void*ignore);
void setArmHeight(double height);
void setArmClampState(bool state);
void toggleArmClampState();

void tiltControl(void*ignore);
void setTiltHeight(double height);
void setTiltClampState(bool state);
void toggleTiltClampState();

void tallSelected();
void neutralSelected();
void allianceSelected();
void reset();
void toggleInnerBranch();

void canisterControl(void*ignore);
void setCanisterState(bool state);
void toggleCanisterState();
void release(double delayTime);
#endif
