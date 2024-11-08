#ifndef __POINT_HPP__
#define __POINT_HPP__

namespace driftless {
namespace control {
class Point {
 private:
  // x position
  double m_x{};

  // y position
  double m_y{};

 public:
  // default constructor
  Point() = default;

  // constructor
  Point(double x, double y);

  // default copy constructor
  Point(const Point& other) = default;

  // default move constructor
  Point(Point&& other) = default;

  // default destroyer
  ~Point() = default;

  // sets the x position
  void setX(double x);

  // sets the y position
  void setY(double y);

  // gets the x position
  double getX();

  // gets the y position
  double getY();

  // ---OPERATORS---

  // copy overload
  Point& operator=(const Point& rhs) = default;

  // move overload
  Point& operator=(Point&& rhs) = default;

  // addition overload
  Point operator+(const Point& rhs);

  // subtraction overload
  Point operator-(const Point& rhs);

  // multiplication overload
  Point operator*(double rhs);

  // division overload
  Point operator/(double rhs);

  // addition assignment overload
  Point& operator+=(const Point& rhs);

  // subtraction assignment overload
  Point& operator-=(const Point& rhs);

  // multiplication assignment overload
  Point& operator*=(double rhs);

  // division assignment overload
  Point& operator/=(double rhs);
};
}  // namespace control
}  // namespace driftless
#endif