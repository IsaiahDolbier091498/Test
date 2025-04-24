#ifndef CONTROL_H
#define CONTROL_H
//Exports control functions

void initServos();
void updateIMUandServos();

extern float adjustedPitch;
extern float adjustedRoll;
extern float adjustedYaw;

#endif
