#ifndef TEENSY41_H
#define TEENSY41_H

class Teensy41
{
    private:
    const char* batteryStatus(float percentage);

    public:
    void reset();
    void checkI2CLines();
    void measureBattery();

    void checkMIC2544MainFlag();
    void checkMIC2544BackupFlag();
    void ejectionChargeMain();
    void ejectionChargeBackup();

    void setLEDStatus(bool nominal);
};

#endif