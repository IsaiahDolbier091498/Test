#ifndef ALTIMETER_H
#define ALTIMETER_H
//Exports sensor functions and shared data



// Expose variables you might want for control logic
extern float velocity;
extern float relativeAltitude;

void initSensors();
void updateAltitude();
void calibrateAltimeter(int sampleAmount);

#endif