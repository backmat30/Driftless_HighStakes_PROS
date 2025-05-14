#include "driftless/robot/subsystems/arm/PIDArmMotionBuilder.hpp"

namespace driftless {
namespace robot {
namespace subsystems {
namespace arm {
PIDArmMotionBuilder* PIDArmMotionBuilder::withClock(
    const std::unique_ptr<driftless::rtos::IClock>& clock) {
  m_clock = clock->clone();
  return this;
}
PIDArmMotionBuilder* PIDArmMotionBuilder::withDelayer(
    const std::unique_ptr<driftless::rtos::IDelayer>& delayer) {
  m_delayer = delayer->clone();
  return this;
}

PIDArmMotionBuilder* PIDArmMotionBuilder::withMutex(
    std::unique_ptr<driftless::rtos::IMutex>& mutex) {
  m_mutex = std::move(mutex);
  return this;
}

PIDArmMotionBuilder* PIDArmMotionBuilder::withTask(
    std::unique_ptr<driftless::rtos::ITask>& task) {
  m_task = std::move(task);
  return this;
}

PIDArmMotionBuilder* PIDArmMotionBuilder::withRotationSensor(
    std::unique_ptr<driftless::io::IRotationSensor>& rotation_sensor) {
  m_rotation_sensor = std::move(rotation_sensor);
  return this;
}

PIDArmMotionBuilder* PIDArmMotionBuilder::withPotentiometer(
    std::unique_ptr<driftless::io::IPotentiometer>& potentiometer) {
  m_potentiometer = std::move(potentiometer);
  return this;
}

PIDArmMotionBuilder* PIDArmMotionBuilder::withRotationalMotor(
    std::unique_ptr<driftless::io::IMotor>& motor) {
  m_rotation_motors.addMotor(motor);
  return this;
}

PIDArmMotionBuilder* PIDArmMotionBuilder::withLinearMotor(
    std::unique_ptr<driftless::io::IMotor>& motor) {
  m_linear_motors.addMotor(motor);
  return this;
}

PIDArmMotionBuilder* PIDArmMotionBuilder::withRotationalPID(
    driftless::control::PID rotational_pid) {
  m_rotational_pid = rotational_pid;
  return this;
}

PIDArmMotionBuilder* PIDArmMotionBuilder::withLinearPID(
    driftless::control::PID linear_pid) {
  m_linear_pid = linear_pid;
  return this;
}

PIDArmMotionBuilder* PIDArmMotionBuilder::withRotationalNeutralPosition(
    double rotational_neutral_position) {
  m_rotational_neutral_position = rotational_neutral_position;
  return this;
}

PIDArmMotionBuilder* PIDArmMotionBuilder::withRotationalLoadPosition(
    double rotational_load_position) {
  m_rotational_load_position = rotational_load_position;
  return this;
}

PIDArmMotionBuilder* PIDArmMotionBuilder::withRotationalReadyPosition(
    double rotational_ready_position) {
  m_rotational_ready_position = rotational_ready_position;
  return this;
}

PIDArmMotionBuilder* PIDArmMotionBuilder::withRotationalScorePosition(
    double rotational_score_position) {
  m_rotational_score_position = rotational_score_position;
  return this;
}

PIDArmMotionBuilder* PIDArmMotionBuilder::withRotationalRushPosition(
    double rotational_rush_position) {
  m_rotational_rush_position = rotational_rush_position;
  return this;
}

PIDArmMotionBuilder* PIDArmMotionBuilder::withRotationalAllianceStakePosition(
    double rotational_alliance_stake_position) {
  m_rotational_alliance_stake_position = rotational_alliance_stake_position;
  return this;
}

PIDArmMotionBuilder* PIDArmMotionBuilder::withRotationalClimbPosition(
    double rotational_climb_position) {
  m_rotational_climb_position = rotational_climb_position;
  return this;
}

PIDArmMotionBuilder*
PIDArmMotionBuilder::withRotationalReadyIntermediatePosition(
    double rotational_ready_intermediate_position) {
  m_rotational_ready_intermediate_position =
      rotational_ready_intermediate_position;
  return this;
}

PIDArmMotionBuilder*
PIDArmMotionBuilder::withRotationalScoreIntermediatePosition(
    double rotational_score_intermediate_position) {
  m_rotational_score_intermediate_position =
      rotational_score_intermediate_position;
  return this;
}

PIDArmMotionBuilder*
PIDArmMotionBuilder::withRotationalRushIntermediatePosition(
    double rotational_rush_intermediate_position) {
  m_rotational_rush_intermediate_position =
      rotational_rush_intermediate_position;
  return this;
}

PIDArmMotionBuilder*
PIDArmMotionBuilder::withRotationalAllianceStakeIntermediatePosition(
    double rotational_alliance_stake_intermediate_position) {
  m_rotational_alliance_stake_intermediate_position =
      rotational_alliance_stake_intermediate_position;
  return this;
}

PIDArmMotionBuilder* PIDArmMotionBuilder::withRotationalTolerance(
    double rotational_tolerance) {
  m_rotational_tolerance = rotational_tolerance;
  return this;
}

PIDArmMotionBuilder* PIDArmMotionBuilder::withLinearNeutralPosition(
    double linear_neutral_position) {
  m_linear_neutral_position = linear_neutral_position;
  return this;
}

PIDArmMotionBuilder* PIDArmMotionBuilder::withLinearLoadPosition(
    double linear_load_position) {
  m_linear_load_position = linear_load_position;
  return this;
}

PIDArmMotionBuilder* PIDArmMotionBuilder::withLinearReadyPosition(
    double linear_ready_position) {
  m_linear_ready_position = linear_ready_position;
  return this;
}

PIDArmMotionBuilder* PIDArmMotionBuilder::withLinearScorePosition(
    double linear_score_position) {
  m_linear_score_position = linear_score_position;
  return this;
}

PIDArmMotionBuilder* PIDArmMotionBuilder::withLinearRushPosition(
    double linear_rush_position) {
  m_linear_rush_position = linear_rush_position;
  return this;
}

PIDArmMotionBuilder* PIDArmMotionBuilder::withLinearAllianceStakePosition(
    double linear_alliance_stake_position) {
  m_linear_alliance_stake_position = linear_alliance_stake_position;
  return this;
}

PIDArmMotionBuilder* PIDArmMotionBuilder::withLinearClimbReadyPosition(
    double linear_climb_ready_position) {
  m_linear_climb_ready_position = linear_climb_ready_position;
  return this;
}

PIDArmMotionBuilder* PIDArmMotionBuilder::withLinearClimbPosition(
    double linear_climb_position) {
  m_linear_climb_position = linear_climb_position;
  return this;
}

PIDArmMotionBuilder* PIDArmMotionBuilder::withLinearTolerance(
    double linear_tolerance) {
  m_linear_tolerance = linear_tolerance;
  return this;
}

std::unique_ptr<PIDArmMotion> PIDArmMotionBuilder::build() {
  std::unique_ptr<PIDArmMotion> pid_arm_motion{
      std::make_unique<PIDArmMotion>()};

  pid_arm_motion->setClock(m_clock);
  pid_arm_motion->setDelayer(m_delayer);
  pid_arm_motion->setMutex(m_mutex);
  pid_arm_motion->setTask(m_task);
  pid_arm_motion->setRotationSensor(m_rotation_sensor);
  pid_arm_motion->setPotentiometer(m_potentiometer);
  pid_arm_motion->setRotationMotors(m_rotation_motors);
  pid_arm_motion->setLinearMotors(m_linear_motors);
  pid_arm_motion->setRotationalPID(m_rotational_pid);
  pid_arm_motion->setLinearPID(m_linear_pid);
  pid_arm_motion->setRotationalNeutralPosition(m_rotational_neutral_position);
  pid_arm_motion->setRotationalLoadPosition(m_rotational_load_position);
  pid_arm_motion->setRotationalReadyPosition(m_rotational_ready_position);
  pid_arm_motion->setRotationalScorePosition(m_rotational_score_position);
  pid_arm_motion->setRotationalRushPosition(m_rotational_rush_position);
  pid_arm_motion->setRotationalAllianceStakePosition(
      m_rotational_alliance_stake_position);
  pid_arm_motion->setRotationalClimbPosition(m_rotational_climb_position);
  pid_arm_motion->setRotationalReadyIntermediatePosition(
      m_rotational_ready_intermediate_position);
  pid_arm_motion->setRotationalScoreIntermediatePosition(
      m_rotational_score_intermediate_position);
  pid_arm_motion->setRotationalRushIntermediatePosition(
      m_rotational_rush_intermediate_position);
  pid_arm_motion->setRotationalAllianceStakeIntermediatePosition(
      m_rotational_alliance_stake_intermediate_position);
  pid_arm_motion->setRotationalTolerance(m_rotational_tolerance);
  pid_arm_motion->setLinearNeutralPosition(m_linear_neutral_position);
  pid_arm_motion->setLinearLoadPosition(m_linear_load_position);
  pid_arm_motion->setLinearReadyPosition(m_linear_ready_position);
  pid_arm_motion->setLinearScorePosition(m_linear_score_position);
  pid_arm_motion->setLinearRushPosition(m_linear_rush_position);
  pid_arm_motion->setLinearAllianceStakePosition(
      m_linear_alliance_stake_position);
  pid_arm_motion->setLinearClimbReadyPosition(m_linear_climb_ready_position);
  pid_arm_motion->setLinearClimbPosition(m_linear_climb_position);
  pid_arm_motion->setLinearTolerance(m_linear_tolerance);

  return pid_arm_motion;
}
}  // namespace arm
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless