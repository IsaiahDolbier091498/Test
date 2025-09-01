#ifndef SDWRITER_H
#define SDWRITER_H
class SDWriter
{
  private:
    void SDCardWrite(unsigned long timeStamp);
  
  public:
    void initSDCard();
    void logTelemetry(unsigned long ms, unsigned long initTimeTaken, unsigned long interval);
};

#endif