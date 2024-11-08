#ifndef __PID_ARM_MOTION_BUILDER_HPP__
#define __PID_ARM_MOTION_BUILDER_HPP__

#include <memory>

#include "driftless/robot/subsystems/arm/PIDArmMotion.hpp"

namespace driftless {
namespace robot {
namespace subsystems {
namespace arm {
class PIDArmMotionBuilder {
 private:
  // the delayer used by the subsystem
  std::unique_ptr<driftless::rtos::IDelayer> m_delayer{};

  // the mutex used for task-related functions
  std::unique_ptr<driftless::rtos::IMutex> m_mutex{};

  // the task used by the subsystem
  std::unique_ptr<driftless::rtos::ITask> m_task{};

  // rotation sensor for the arm rotation
  std::unique_ptr<driftless::io::IRotationSensor> m_rotation_sensor{};

  // potentiometer for arm rotation
  std::unique_ptr<driftless::io::IPotentiometer> m_potentiometer{};

  // the motors used to rotate the arm
  driftless::hal::MotorGroup m_rotation_motors{};

  // the motors used to control the length of the arm
  driftless::hal::MotorGroup m_linear_motors{};

  // the rotational PID controller
  driftless::control::PID m_rotational_pid{};

  // the linear PID controller
  driftless::control::PID m_linear_pid{};

  // the rotational position when neutral
  double m_rotational_neutral_position{};

  // the rotational position when loading
  double m_rotational_load_position{};

  // the rotational position when ready to score
  double m_rotational_ready_position{};

  // the rotational position when scoring
  double m_rotational_score_position{};

  // the rotational position tolerance
  double m_rotational_tolerance{};

  // the linear position when neutral
  double m_linear_neutral_position{};

  // the linear position when loading
  double m_linear_load_position{};

  // the linear position when ready to score
  double m_linear_ready_position{};

  // the linear position when scoring
  double m_linear_score_position{};

  // the linear position tolerance
  double m_linear_tolerance{};

 public:
  // add a delayer to the builder
  PIDArmMotionBuilder* withDelayer(
      const std::unique_ptr<driftless::rtos::IDelayer>& delayer);

  // add a mutex to the builder
  PIDArmMotionBuilder* withMutex(std::unique_ptr<driftless::rtos::IMutex>& mutex);

  // add a task to the builder
  PIDArmMotionBuilder* withTask(std::unique_ptr<driftless::rtos::ITask>& task);

  // add a rotation sensor to the builder
  PIDArmMotionBuilder* withRotationSensor(
      std::unique_ptr<driftless::io::IRotationSensor>& rotation_sensor);

  // add a potentiometer to the builder
  PIDArmMotionBuilder* withPotentiometer(
      std::unique_ptr<driftless::io::IPotentiometer>& potentiometer);

  // add a rotational motor to the builder
  PIDArmMotionBuilder* withRotationalMotor(
      std::unique_ptr<driftless::io::IMotor>& motor);

  // add a linear motor to the builder
  PIDArmMotionBuilder* withLinearMotor(
      std::unique_ptr<driftless::io::IMotor>& motor);

  // add a rotational PID controller to the builder
  PIDArmMotionBuilder* withRotationalPID(driftless::control::PID rotational_pid);

  // add a linear PID controller to the builder
  PIDArmMotionBuilder* withLinearPID(driftless::control::PID linear_pid);

  // add a rotational neutral position to the builder
  PIDArmMotionBuilder* withRotationalNeutralPosition(
      double rotational_neutral_position);

  // add a rotational load position to the builder
  PIDArmMotionBuilder* withRotationalLoadPosition(
      double rotational_load_position);

  // add a rotational ready position to the builder
  PIDArmMotionBuilder* withRotationalReadyPosition(
      double rotational_ready_position);

  // add a rotational score position to the builder
  PIDArmMotionBuilder* withRotationalScorePosition(
      double rotational_score_position);

  // add a rotational position tolerance to the builder
  PIDArmMotionBuilder* withRotationalTolerance(double rotational_tolerance);

  // add a linear neutral position to the builder
  PIDArmMotionBuilder* withLinearNeutralPosition(
      double linear_neutral_position);

  // add a linear load position to the builder
  PIDArmMotionBuilder* withLinearLoadPosition(double linear_load_position);

  // add a linear ready position to the builder
  PIDArmMotionBuilder* withLinearReadyPosition(double linear_ready_position);

  // add a linear score position to the builder
  PIDArmMotionBuilder* withLinearScorePosition(double linear_score_position);

  // add a linear position tolerance to the builder
  PIDArmMotionBuilder* withLinearTolerance(double linear_tolerance);

  // build a new arm motion controller
  std::unique_ptr<PIDArmMotion> build();
};
}  // namespace arm
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif