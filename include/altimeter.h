#ifndef ALTIMETER_H
#define ALTIMETER_H
#include <stdint.h>
class Altimeter
{
    private:
    bool i2c_write_register(uint8_t deviceAddr, uint8_t regAddr, uint8_t value);
    bool enableBmp390Interrupt();

    public:
        void initAltimeter();
        void calibrateAltimeter(int sampleAmount);
        void updateAltitude();

        float getVelocity();
        float getRelativeAltitude();
};

#endif