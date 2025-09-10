#ifndef GNSS_H
#define GNSS_H

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