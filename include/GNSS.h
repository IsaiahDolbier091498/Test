#ifndef GNSS_H
#define GNSS_H

void initGnss();
void setOrigin(int samples);
void getGnssCoords();

float originLat;
float OriginLong;

#endif