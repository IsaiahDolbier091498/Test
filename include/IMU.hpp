#ifndef IMU_H
#define IMU_H
//Exports control functions

struct Quaternion
{
  float w, x, y, z;
};

class IMU
{
  private:

  public:
    void resetBNO085();
    void initIMU();
    void updateOrientation();
    void calibrateIMU(int sampleAmount);

    float getPitch();
    float getRoll();
    float getYaw();

    float getPitchCorrection();
    float getRollCorrection();
    float getYawCorrection();
};

#endif
