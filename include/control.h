#ifndef CONTROL_H
#define CONTROL_H
//Exports control functions

void initIMU();
void updateIMUandServos();
void calibrateIMU(int sampleAmount);
void printData();
void initServos();

extern float adjustedPitch;
extern float adjustedRoll;
extern float adjustedYaw;
extern int s1;
extern int s2;
extern int s3;
extern int s4;

#endif
