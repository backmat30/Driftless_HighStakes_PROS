#ifndef __UTILITY_FUNCTIONS_HPP__
#define __UTILITY_FUNCTIONS_HPP__

#include <stdint.h>
#include <cmath>

#include "driftless/control/Point.hpp"

// returns the sign (- or +) of the input
int8_t sign(double value);

// binds the input radians between -PI and PI
double bindRadians(double radians);

// get the angle between 2 points
double angle(double x1, double y1, double x2, double y2);

// get the distance between 2 points
double distance(double x1, double y1, double x2, double y2);

// get the binomial coefficient (n choose k)
int16_t binomialCoefficient(int8_t n, int8_t k);

/// @brief Mirrors a target over a mirror
/// @param target The target value
/// @param mirror The mirror value
/// @return __double__ The mirrored value
double mirrorValue(double target, double mirror);
#endif