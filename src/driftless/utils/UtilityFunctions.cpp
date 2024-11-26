#include "driftless/utils/UtilityFunctions.hpp"

int8_t sign(double value) { return value / std::abs(value); }

double bindRadians(double radians) {
  while (radians > M_PI) {
    radians -= M_PI * 2;
  }
  while (radians < -M_PI) {
    radians += M_PI * 2;
  }
  return radians;
}

double angle(double x1, double y1, double x2, double y2) {
  return std::atan2(y2 - y1, x2 - x1);
}

double distance(double x1, double y1, double x2, double y2) {
  return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
}

int16_t binomialCoefficient(int8_t n, int8_t k) {
  // https://en.cppreference.com/w/cpp/experimental/special_functions/beta
  // math is funky beta function goes over my head frfr
  return 1 / ((n + 1) * std::beta(n - k + 1, k + 1));
}

double mirrorValue(double target, double mirror) {
  double result{mirror * 2 - target};
  return result;
}