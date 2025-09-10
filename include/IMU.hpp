#ifndef IMU_H
#define IMU_H
//Exports control functions

struct Quaternion
{
  float w, x, y, z;
};

struct PID {
  float Kp, Ki, Kd;        // gains
  float integral;          // accumulated error
  float prevError;         // last error for derivative
};

class IMU
{
  private:

  public:
    void resetBNO085();
    void initIMU();
    void updateOrientation();
    float updatePID(PID &pid, float setpoint, float measurement, float dt);
    void calibrateIMU(int sampleAmount);

    float getPitch();
    float getRoll();
    float getYaw();

    float getPitchCorrection();
    float getRollCorrection();
    float getYawCorrection();
};

#endif
