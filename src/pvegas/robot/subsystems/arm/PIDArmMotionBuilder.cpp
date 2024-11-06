#include "pvegas/robot/subsystems/arm/PIDArmMotionBuilder.hpp"

namespace pvegas {
namespace robot {
namespace subsystems {
namespace arm {
PIDArmMotionBuilder* PIDArmMotionBuilder::withDelayer(const std::unique_ptr<pvegas::rtos::IDelayer>& delayer) {
  m_delayer = delayer->clone();
  return this;
}

PIDArmMotionBuilder* PIDArmMotionBuilder::withMutex(std::unique_ptr<pvegas::rtos::IMutex>& mutex) {
  m_mutex = std::move(mutex);
  return this;
}

PIDArmMotionBuilder* PIDArmMotionBuilder::withTask(std::unique_ptr<pvegas::rtos::ITask>& task) {
  m_task = std::move(task);
  return this;
}

PIDArmMotionBuilder* PIDArmMotionBuilder::withRotationSensor(std::unique_ptr<pvegas::io::IRotationSensor>& rotation_sensor) {
  m_rotation_sensor = std::move(rotation_sensor);
  return this;
}

PIDArmMotionBuilder* PIDArmMotionBuilder::withPotentiometer(std::unique_ptr<pvegas::io::IPotentiometer>& potentiometer) {
  m_potentiometer = std::move(potentiometer);
  return this;
}

PIDArmMotionBuilder* PIDArmMotionBuilder::withRotationalMotor(std::unique_ptr<pvegas::io::IMotor>& motor) {
  m_rotation_motors.addMotor(motor);
  return this;
}

PIDArmMotionBuilder* PIDArmMotionBuilder::withLinearMotor(std::unique_ptr<pvegas::io::IMotor>& motor) {
  m_linear_motors.addMotor(motor);
  return this;
}

PIDArmMotionBuilder* PIDArmMotionBuilder::withRotationalPID(pvegas::control::PID rotational_pid) {
  m_rotational_pid = rotational_pid;
  return this;
}

PIDArmMotionBuilder* PIDArmMotionBuilder::withLinearPID(pvegas::control::PID linear_pid) {
  m_linear_pid = linear_pid;
  return this;
}

PIDArmMotionBuilder* PIDArmMotionBuilder::withRotationalNeutralPosition(double rotational_neutral_position) {
  m_rotational_neutral_position = rotational_neutral_position;
  return this;
}

PIDArmMotionBuilder* PIDArmMotionBuilder::withRotationalLoadPosition(double rotational_load_position) {
  m_rotational_load_position = rotational_load_position;
  return this;
}

PIDArmMotionBuilder* PIDArmMotionBuilder::withRotationalScorePosition(double rotational_score_position) {
  m_rotational_score_position = rotational_score_position;
  return this;
}

PIDArmMotionBuilder* PIDArmMotionBuilder::withRotationalTolerance(double rotational_tolerance) {
  m_rotational_tolerance = rotational_tolerance;
  return this;
}

PIDArmMotionBuilder* PIDArmMotionBuilder::withLinearNeutralPosition(double linear_neutral_position) {
  m_linear_neutral_position = linear_neutral_position;
  return this;
}

PIDArmMotionBuilder* PIDArmMotionBuilder::withLinearLoadPosition(double linear_load_position) {
  m_linear_load_position = linear_load_position;
  return this;
}

PIDArmMotionBuilder* PIDArmMotionBuilder::withLinearScorePosition(double linear_score_position) {
  m_linear_score_position = linear_score_position;
  return this;
}

PIDArmMotionBuilder* PIDArmMotionBuilder::withLinearTolerance(double linear_tolerance) {
  m_linear_tolerance = linear_tolerance;
  return this;
}
}
}
}
}