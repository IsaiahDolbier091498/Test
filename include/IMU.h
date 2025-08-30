#ifndef IMU_H
#define IMU_H
//Exports control functions

struct Quaternion 
{
  float w, x, y, z;
};

class IMU
{
  public:
    void initIMU();
    void updateOrientation();
    void calibrateIMU(int sampleAmount);

    float getPitchCorrection();
    float getRollCorrection();
    float getYawCorrection();

  private:

};

#endif
