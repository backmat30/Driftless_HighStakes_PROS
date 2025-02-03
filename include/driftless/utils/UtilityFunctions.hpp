#ifndef __UTILITY_FUNCTIONS_HPP__
#define __UTILITY_FUNCTIONS_HPP__

#include <stdint.h>

#include <cmath>

#include "driftless/control/Point.hpp"

/// @brief Gets the sign of a value (+ or -)
/// @param value __double__ The value to get the sign of
/// @return __int8_t__ The sign of the value (-1 or 1)
int8_t sign(double value);

/// @brief Binds a value between -PI and PI
/// @param radians __double__ The value to bind
/// @return __double__ The bound value
double bindRadians(double radians);

/// @brief Gets the angle between two points
/// @param x1 __double__ The x value of the first point
/// @param y1 __double__ The y value of the first point
/// @param x2 __double__ The x value of the second point
/// @param y2 __double__ The y value of the second point
/// @return __double__ The angle between the two points
double angle(double x1, double y1, double x2, double y2);

/// @brief Gets the distance between two points
/// @param x1 __double__ The x value of the first point
/// @param y1 __double__ The y value of the first point
/// @param x2 __double__ The x value of the second point
/// @param y2 __double__ The y value of the second point
/// @return __double__ The distance between the two points
double distance(double x1, double y1, double x2, double y2);

/// @brief Gets the binomial coefficient of n and k (n choose k)
/// @param n __int8_t__ The number of items
/// @param k __int8_t__ The number of items to choose
/// @return __int16_t__ The binomial coefficient
int16_t binomialCoefficient(int8_t n, int8_t k);

/// @brief Mirrors a target over a mirror point
/// @param target The target value
/// @param mirror The mirror value
/// @return __double__ The mirrored value
double mirrorValue(double target, double mirror);
#endif