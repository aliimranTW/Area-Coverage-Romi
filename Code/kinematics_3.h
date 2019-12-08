#ifndef _Kinematics
#define _Kinematics_h

class Kinematics
{
  public:
    Kinematics(float X, float Y, float T);
    //Public variables and methods go here
    //You may want to use these variables
    const int WHEEL_DIAMETER = 70;
    const int WHEEL_DISTANCE = 146;
    const int GEAR_RATIO = 120;
    const int COUNTS_PER_SHAFT_REVOLUTION = 12;
    const int COUNTS_PER_WHEEL_REVOLUTION =  1440;
    const float COUNTS_PER_MM = 1 / 0.15;
    const float MM_PER_COUNT = 0.15;
    const float radians_PER_COUNT = 0.0043;
    const float degree_PER_COUNT = 0.24;
    void setParameters(float X, float Y, float T);
    float update(float e0, float e1);
    float getX();
    float getY();
    float getT();
    float returnD();


  private:
    //Private variables and methods go here
    float t = 0;
    float x = 0;
    float y = 0;
    float old_x = 0;
    float old_y = 0;
    float old_t = 0;
    float last_e1 = 0;
    float last_e0 = 0;
    float d = 0;


};

Kinematics::Kinematics(float X, float Y, float T)
{
  setParameters(X, Y, T);

  //Set last_millis
  //last_millis = millis();
}

float Kinematics::update(float e0, float e1)
{
  //convert encoder into mm
  //kinematics
  d = (e1 + e0 - last_e0 - last_e1) / 2;
  t = old_t + (e0 - e1 - last_e0 + last_e1) * 0.15 / WHEEL_DISTANCE;
  //t = (e0-e1)*radians_PER_COUNT;
  x = old_x + d * sin(t);
  y = old_y + d * cos(t);
  while (t >= 2 * PI) {
    t = t - 2 * PI;
  }
  while (t <= -2 * PI) {
    t = t + 2 * PI;
  }
  if (t > 0) {
    if (t > PI ) {
      t = t - 2 * PI;
    }
  }
  if (t < 0) {
    if (-t > PI) {
      t = t + 2 * PI;
    }
  }


  last_e1 = e1;
  last_e0 = e0;
  old_x = x;
  old_y = y;
  old_t = t;

  //end of loop
  //Print debugging information if required
  //Print response if required



}

float Kinematics::getX() {
  return x;
}

float Kinematics::getY() {
  return y;
}

float Kinematics::getT() {
  return t;
}




void Kinematics::setParameters(float X, float Y, float T) {
  x = X;
  y = Y;
  t = T;

}
#endif
