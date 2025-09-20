#ifndef ACTUATORS_H
#define ACTUATORS_H

struct Servo {
  int pin;
  float angle;
};

class Actuators
{
  private:
    static int pulseWidth(float servoAngle);

    static bool sortByAngle(Servo &a, Servo &b);
    static bool sortByPin(Servo &a, Servo &b);

    static void stopPulse();
    static void startPulse();

  public:
    void initServos();
    void updateAngles();

    int getS1();
    int getS2();
    int getS3();
    int getS4();
};

#endif