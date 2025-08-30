#ifndef ACTUATORS_H
#define ACTUATORS_H

struct Servo {
  int pin;
  int angle;
};

class Actuators
{
  public:
  void initServos();
  void updateAngles();
  int getS1();
  int getS2();
  int getS3();
  int getS4();

  private:
    static int pulseWidth(int servoAngle);
    
    static bool sortByAngle(Servo &a, Servo &b);
    static bool sortByPin(Servo &a, Servo &b);

    static void stopPulse();
    static void startPulse();
};

#endif