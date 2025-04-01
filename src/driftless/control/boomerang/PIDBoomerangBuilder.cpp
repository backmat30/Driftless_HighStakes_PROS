#include "driftless/control/boomerang/PIDBoomerangBuilder.hpp"

namespace driftless::control::boomerang {
PIDBoomerangBuilder* PIDBoomerangBuilder::withDelayer(
    const std::unique_ptr<rtos::IDelayer>& delayer) {
  m_delayer = delayer->clone();
  return this;
}

PIDBoomerangBuilder* PIDBoomerangBuilder::withMutex(
    std::unique_ptr<rtos::IMutex>& mutex) {
  m_mutex = std::move(mutex);
  return this;
}

PIDBoomerangBuilder* PIDBoomerangBuilder::withTask(
    std::unique_ptr<rtos::ITask>& task) {
  m_task = std::move(task);
  return this;
}

PIDBoomerangBuilder* PIDBoomerangBuilder::withLinearPID(PID linear_pid) {
  m_linear_pid = linear_pid;
  return this;
}

PIDBoomerangBuilder* PIDBoomerangBuilder::withRotationalPID(
    PID rotational_pid) {
  m_rotational_pid = rotational_pid;
  return this;
}

PIDBoomerangBuilder* PIDBoomerangBuilder::withLead(double lead) {
  m_lead = lead;
  return this;
}

PIDBoomerangBuilder* PIDBoomerangBuilder::withAimDistance(double aim_distance) {
  m_aim_distance = aim_distance;
  return this;
}

PIDBoomerangBuilder* PIDBoomerangBuilder::withTargetTolerance(
    double target_tolerance) {
  m_target_tolerance = target_tolerance;
  return this;
}

PIDBoomerangBuilder* PIDBoomerangBuilder::withTargetVelocity(
    double target_velocity) {
  m_target_velocity = target_velocity;
  return this;
}
}  // namespace driftless::control::boomerang