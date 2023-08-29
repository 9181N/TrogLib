#pragma once

#include <cmath>

class Angle {
public:

  static constexpr double PI = 3.141592653589793238;
  static constexpr double RAD_TO_DEG = 180 / PI;
  static constexpr double DEG_TO_RAD = PI / 180;

  Angle() {
    _deg = 0;
    _rad = 0;
  }

  Angle(const Angle &copy) {
    _deg = copy._deg;
    _rad = copy._rad;
  }

  //Angle(double value) { SetDeg(value); }

  static Angle fromDeg(double deg) {Angle a; a.setDeg(deg); return a;}
  static Angle fromRad(double rad) {Angle a; a.setRad(rad); return a;}

  void setDeg(double value) {
    _deg = value;
    _rad = value * DEG_TO_RAD;

    if (_deg >= 360.0) {
      setDeg(_deg - (floor(_deg / 360.0) * 360));
    }

    if (_deg < 0) {
      setDeg(360.0 + _deg - (ceil(_deg / 360.0) * 360));
    }
  }
  void setRad(double value) {
    _rad = value;
    _deg = value * RAD_TO_DEG;

    if (_deg >= 360.0) {
      setDeg(_deg - (floor(_deg / 360.0) * 360));
    }

    if (_deg < 0) {
      setDeg(360 + _deg - (ceil(_deg / 360.0) * 360));
    }
  }
  double getDeg() { return _deg; }
  double getRad() { return _rad; }

  static double shortestError(Angle from, Angle to);

  Angle operator+(const Angle &obj) {return fromRad(_rad + obj._rad);}
  Angle operator-(const Angle &obj) {return fromRad(_rad - obj._rad);}
  Angle operator-() {return fromRad(-_rad);}


private:
  double _deg;
  double _rad;
};

class Vector2 {
public:
  Vector2(double xValue=0, double yValue=0) {
    x = xValue;
    y = yValue;
  }

  Vector2(const Vector2 &copy) {
    x = copy.x;
    y = copy.y;
  }

  static Vector2 fromPolar(double magnitude, Angle angle) {
    return Vector2(magnitude * cos(angle.getRad()), magnitude * sin(angle.getRad()));
  }

  double x;
  double y;

  Vector2 getNormalized();

  double getMagnitude();
  Angle getAngleDirection();

  void setMagnitude(double value); // sets magnitude while preserving direction
  void setAngleDirection(
      Angle value); // like rotating a vector, changes its polar
                     // coordinate angle while preserving magnitude

  double dot(Vector2 obj) { return x * obj.x + y * obj.x; }

  

  Vector2 operator-(Vector2 const &obj) {
    return Vector2(x - obj.x, y - obj.y);
  }

  // private:
};

double operator*(Vector2 const &obj1, Vector2 const &obj2);

Vector2 operator+(Vector2 const &obj1, Vector2 const &obj2);

Vector2 operator*(double const &scal, Vector2 const &vec);

Vector2 operator*(Vector2 const &vec, double const &scal);