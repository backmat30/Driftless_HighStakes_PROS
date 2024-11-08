#include "driftless/control/Point.hpp"

namespace driftless {
namespace control {
Point::Point(double x, double y) : m_x{x}, m_y{y} {}

void Point::setX(double x) { m_x = x; }

void Point::setY(double y) { m_y = y; }

double Point::getX() { return m_x; }

double Point::getY() { return m_y; }

Point Point::operator+(const Point& rhs) {
  return Point{m_x + rhs.m_x, m_y + rhs.m_y};
}

Point Point::operator-(const Point& rhs) {
  return Point{m_x - rhs.m_x, m_y - rhs.m_y};
}

Point Point::operator*(double rhs) { return Point{m_x * rhs, m_y * rhs}; }

Point Point::operator/(double rhs) { return Point{m_x / rhs, m_y / rhs}; }

Point& Point::operator+=(const Point& rhs) {
  m_x += rhs.m_x;
  m_y += rhs.m_y;
  return *this;
}

Point& Point::operator-=(const Point& rhs) {
  m_x -= rhs.m_x;
  m_y -= rhs.m_y;
  return *this;
}

Point& Point::operator*=(double rhs) {
  m_x *= rhs;
  m_y *= rhs;
  return *this;
}

Point& Point::operator/=(double rhs) {
  m_x /= rhs;
  m_y /= rhs;
  return *this;
}
}  // namespace control
}  // namespace driftless