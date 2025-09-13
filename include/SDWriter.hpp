#ifndef SDWRITER_H
#define SDWRITER_H

class SDWriter
{
  private:
    void SDCardWrite(unsigned long timeStamp);

  public:
    void initSDCard();
    void logTelemetry(unsigned long ms, unsigned long initTimeTaken, unsigned long interval);
    static void logAvionicsInit(const char* line);
};

inline void log(const char* line, ...) {
    char buffer[256];
    va_list args;
    va_start(args, line);
    vsnprintf(buffer, sizeof(buffer), line, args);
    va_end(args);

    SDWriter::logAvionicsInit(buffer);
}

#endif