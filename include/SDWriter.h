#ifndef SDWRITER_H
#define SDWRITER_H

void initSDCard();
void SDCardWrite(unsigned long timeStamp);
void logTelemetry(unsigned long ms, unsigned long initTimeTaken, unsigned long interval);

#endif