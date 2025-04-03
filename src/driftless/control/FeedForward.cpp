#include "driftless/control/FeedForward.hpp"

namespace driftless::control {
FeedForward::FeedForward(double kS, double kV)
    : m_kS(kS), m_kV(kV) {}

double FeedForward::getControlValue(double target_velocity) {
  int velocity_sign{target_velocity / std::abs(target_velocity)};

  double control_value{m_kS * velocity_sign + m_kV * target_velocity};

  return control_value;
}

FeedForward& FeedForward::operator=(const FeedForward& rhs) {
  m_kS = rhs.m_kS;
  m_kV = rhs.m_kV;

  return *this;
}
}  // namespace driftless::control