#ifndef __POINT_HPP__
#define __POINT_HPP__

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for control algorithms
/// @author Matthew Backman
namespace control {

/// @brief Class representing a point on the field
/// @author Matthew Backman
class Point {
 private:
  // x position
  double m_x{};

  // y position
  double m_y{};

 public:
  /// @brief Constructs a new point
  Point() = default;

  /// @brief Constructs a new point
  /// @param x __double__ The x coordinate
  /// @param y __double__ The y coordinate
  Point(double x, double y);

  /// @brief Constructs a new point
  /// @param other __const Point&__ The point to copy
  Point(const Point& other) = default;

  /// @brief Constructs a new point
  /// @param other __Point&&__ The point to move
  Point(Point&& other) = default;

  /// @brief Destroys the point
  ~Point() = default;

  /// @brief Sets the x position of the point
  /// @param x __double__ The new x position
  void setX(double x);

  /// @brief Sets the y position of the point
  /// @param y __double__ The new y position
  void setY(double y);

  /// @brief Gets the x position of the point
  /// @return __double__ The x position
  double getX();

  /// @brief Gets the y position of the point
  /// @return __double__ The y position
  double getY();

  // ---OPERATORS---

  /// @brief Copies another point
  /// @param rhs __const Point&__ The point being copied
  /// @return __Point&__ Reference to the new copy
  Point& operator=(const Point& rhs) = default;

  /// @brief Moves another point
  /// @param rhs __Point&&__ The point being moved
  /// @return __Point&__ Reference to the moved point
  Point& operator=(Point&& rhs) = default;

  /// @brief Overloads the + operator
  /// @param rhs __const Point&__ The point being added
  /// @return __Point__ The sum of the points
  Point operator+(const Point& rhs);

  /// @brief Overloads the - operator
  /// @param rhs __const Point&__ The point being added
  /// @return __Point__ The difference of the points
  Point operator-(const Point& rhs);

  /// @brief Overloads the * operator
  /// @param rhs __double__ The multiplier
  /// @return __Point__ The product
  Point operator*(double rhs);

  /// @brief Overloads the / operator
  /// @param rhs __double__ The divisor
  /// @return __Point__ The quotient
  Point operator/(double rhs);

  /// @brief Overloads the += operator
  /// @param rhs __const Point&__ The point being added
  /// @return __Point&__ Reference to the result
  Point& operator+=(const Point& rhs);

  /// @brief Overloads the -= operator
  /// @param rhs __const Point&__ The point being substracted
  /// @return __Point&__ Reference to the result
  Point& operator-=(const Point& rhs);

  /// @brief Overloads the *= operator
  /// @param rhs __double__ The multiplier
  /// @return __Point&__ Reference to the result
  Point& operator*=(double rhs);

  /// @brief Overloads the /= operator
  /// @param rhs __double__ The divisor
  /// @return __Point&__ Reference to the result
  Point& operator/=(double rhs);
};
}  // namespace control
}  // namespace driftless
#endif