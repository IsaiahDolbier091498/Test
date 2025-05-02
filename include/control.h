#ifndef CONTROL_H
#define CONTROL_H
//Exports control functions

void initServos();
void updateIMUandServos();

extern float adjustedPitch;
extern float adjustedRoll;
extern float adjustedYaw;
extern int s1;
extern int s2;
extern int s3;
extern int s4;

#endif
