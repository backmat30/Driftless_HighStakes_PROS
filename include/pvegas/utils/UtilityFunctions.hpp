#ifndef __UTILITY_FUNCTIONS_HPP__
#define __UTILITY_FUNCTIONS_HPP__

#include <algorithm>
#include <cmath>

// returns the sign (- or +) of the input
int8_t sign(double value);

// binds the input radians between -PI and PI
double bindRadians(double radians);

// get the angle between 2 points
double angle(double x1, double y1, double x2, double y2);

// get the distance between 2 points
double distance(double x1, double y1, double x2, double y2);


#endif