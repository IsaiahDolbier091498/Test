#ifndef DEBUG_H
#define DEBUG_H

void reset();
void checkI2CLines();
void measureBattery();
void checkMIC2544MainFlag();
void checkMIC2544BackupFlag();
void nominalStatusLED();
void errorStatusLED();

#endif