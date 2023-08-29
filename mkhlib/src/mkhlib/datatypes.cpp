#include "mkhlib/datatypes.hpp"

#include <math.h>

double operator*(Vector2 const &obj1, Vector2 const &obj2) {return obj1.x * obj2.x + obj1.y * obj2.y;}

Vector2 operator+(Vector2 const &obj1, Vector2 const &obj2) {
    return Vector2(obj1.x + obj2.x, obj1.y + obj2.y);
  }

Vector2 operator*(double const &scal, Vector2 const &vec) { return Vector2(vec.x * scal, vec.y * scal); }

Vector2 operator*(Vector2 const &vec, double const &scal) {return scal * vec;};

double Vector2::getMagnitude() { return sqrt(x * x + y * y); }
Angle Vector2::getAngleDirection() {
  if(x == 0) {
    if(y >= 0) {
      return Angle::fromDeg(90);
    }
    else {
      return Angle::fromDeg(-90);
    }
  }

  return Angle::fromRad(atan2(y, x));
}
Vector2 Vector2::getNormalized() {
  if(getMagnitude() == 0) {
    return Vector2(0,0);
  }
  return Vector2(x / getMagnitude(), y / getMagnitude());
}
void Vector2::setMagnitude(double value) {
  double mag = getMagnitude();

  x = x / mag * value;

  y = y / mag * value;
}
void Vector2::setAngleDirection(Angle value) {
  double mag = getMagnitude();

  x = mag * cos(value.getRad());

  y = mag * sin(value.getRad());
}

double Angle::shortestError(Angle from,
                            Angle to) // returns shortest error from angle this
                                      // method is being called on to other
{
  Angle error = to-from;
  // must clamp between pi and -pi
  
  if(error.getDeg() > 180) {
    return error.getRad() - 360 * DEG_TO_RAD;
  }
  else if(error.getDeg() < -180) {
    return error.getRad() + 360 * DEG_TO_RAD;
  }
  return error.getRad();
}