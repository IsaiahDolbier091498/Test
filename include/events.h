#ifndef EVENTS_H
#define EVENTS_H

extern float getAvgAlt(bool newData);
extern volatile bool newDataFlag;
void ejectionChargeMain();
void ejectionChargeBackup();

#endif