#ifndef GNSS_H
#define GNSS_H

float originLat;
float OriginLong;

class GNSS
{
    private:

    public:
      void initGnss();
      void setOrigin(int samples);
      void getGnssCoords();

      float getOriginLat();
      float getOriginLong();
};

#endif