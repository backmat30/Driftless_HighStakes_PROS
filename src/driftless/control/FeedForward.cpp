#include "driftless/control/FeedForward.hpp"

namespace driftless::control {
FeedForward::FeedForward(double kS, double kV, double kA)
    : m_kS(kS), m_kV(kV), m_kA(kA) {}

double FeedForward::getControlValue(double target_velocity,
                                    double target_acceleration) {
  int velocity_sign{target_velocity / std::abs(target_velocity)};

  double control_value{m_kS * velocity_sign + m_kV * target_velocity +
                       m_kA * target_acceleration};

  return control_value;
}

FeedForward& FeedForward::operator=(const FeedForward& rhs) {
  m_kS = rhs.m_kS;
  m_kV = rhs.m_kV;
  m_kA = rhs.m_kA;

  return *this;
}
}  // namespace driftless::control