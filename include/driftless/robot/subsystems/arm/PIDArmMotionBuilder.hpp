#ifndef __PID_ARM_MOTION_BUILDER_HPP__
#define __PID_ARM_MOTION_BUILDER_HPP__

#include <memory>

#include "driftless/robot/subsystems/arm/PIDArmMotion.hpp"

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief The namespace for subsystems code
/// @author Matthew Backman
namespace subsystems {

/// @brief The namespace for arm subsystem code
/// @author Matthew Backman
namespace arm {

/// @brief The builder class for PIDArmMotion
/// @author Matthew Backman
class PIDArmMotionBuilder {
 private:
  // the clock used by the subsystem
  std::unique_ptr<driftless::rtos::IClock> m_clock{};

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

  // the rotational position when rushing
  double m_rotational_rush_position{};

  // The rotational position when scoring on alliance stake
  double m_rotational_alliance_stake_position{};

  double m_rotational_climb_position{};

  // the rotational intermediate position during ready motion
  double m_rotational_ready_intermediate_position{};

  // the rotational intermediate position during score motion
  double m_rotational_score_intermediate_position{};

  // the rotational intermediate position during the rush motion
  double m_rotational_rush_intermediate_position{};

  // the rotational intermediate position during the alliance stake motion
  double m_rotational_alliance_stake_intermediate_position{};

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

  // the linear position when rushing
  double m_linear_rush_position{};

  // the linear position when scoring on the alliance stake
  double m_linear_alliance_stake_position{};

  double m_linear_climb_ready_position{};

  double m_linear_climb_position{};

  // the linear position tolerance
  double m_linear_tolerance{};

 public:
  /// @brief Adds a clock to the builder
  /// @param clock The clock being used
  /// @return __PIDArmMotionBuilder*__ Pointer to the current builder
  PIDArmMotionBuilder* withClock(
      const std::unique_ptr<driftless::rtos::IClock>& clock);

  /// @brief Adds a delayer to the builder
  /// @param delayer The delayer being used
  /// @return __PIDArmMotionBuilder*__ Pointer to the current builder
  PIDArmMotionBuilder* withDelayer(
      const std::unique_ptr<driftless::rtos::IDelayer>& delayer);

  /// @brief Adds a mutex to the builder
  /// @param mutex The mutex being used
  /// @return __PIDArmMotionBuilder*__ Pointer to the current builder
  PIDArmMotionBuilder* withMutex(
      std::unique_ptr<driftless::rtos::IMutex>& mutex);

  /// @brief Adds a task to the builder
  /// @param task The task being used
  /// @return __PIDArmMotionBuilder*__ Pointer to the current builder
  PIDArmMotionBuilder* withTask(std::unique_ptr<driftless::rtos::ITask>& task);

  /// @brief Adds a rotation sensor to the builder
  /// @param rotation_sensor The rotation sensor being used
  /// @return __PIDArmMotionBuilder*__ Pointer to the current builder
  PIDArmMotionBuilder* withRotationSensor(
      std::unique_ptr<driftless::io::IRotationSensor>& rotation_sensor);

  /// @brief Adds a potentiometer to the builder
  /// @param potentiometer The potentiometer being used
  /// @return __PIDArmMotionBuilder*__ Pointer to the current builder
  PIDArmMotionBuilder* withPotentiometer(
      std::unique_ptr<driftless::io::IPotentiometer>& potentiometer);

  /// @brief Adds a rotational motor to the builder
  /// @param motor The motor being used
  /// @return __PIDArmMotionBuilder*__ Pointer to the current builder
  PIDArmMotionBuilder* withRotationalMotor(
      std::unique_ptr<driftless::io::IMotor>& motor);

  /// @brief Adds a linear motor to the builder
  /// @param motor The motor being used
  /// @return __PIDArmMotionBuilder*__ Pointer to the current builder
  PIDArmMotionBuilder* withLinearMotor(
      std::unique_ptr<driftless::io::IMotor>& motor);

  /// @brief Adds a rotational PID controller to the builder
  /// @param rotational_pid The PID controller being used
  /// @return __PIDArmMotionBuilder*__ Pointer to the current builder
  PIDArmMotionBuilder* withRotationalPID(
      driftless::control::PID rotational_pid);

  /// @brief Adds a linear PID controller to the builder
  /// @param linear_pid The PID controller being used
  /// @return __PIDArmMotionBuilder*__ Pointer to the current builder
  PIDArmMotionBuilder* withLinearPID(driftless::control::PID linear_pid);

  /// @brief Adds a rotational neutral position to the builder
  /// @param rotational_neutral_position The position being added
  /// @return __PIDArmMotionBuilder*__ Pointer to the current builder
  PIDArmMotionBuilder* withRotationalNeutralPosition(
      double rotational_neutral_position);

  /// @brief Adds a rotational load position to the builder
  /// @param rotational_load_position The position being added
  /// @return __PIDArmMotionBuilder*__ Pointer to the current builder
  PIDArmMotionBuilder* withRotationalLoadPosition(
      double rotational_load_position);

  /// @brief Adds a rotational ready position to the builder
  /// @param rotational_ready_position The position being added
  /// @return __PIDArmMotionBuilder*__ Pointer to the current builder
  PIDArmMotionBuilder* withRotationalReadyPosition(
      double rotational_ready_position);

  /// @brief Adds a rotational score position to the builder
  /// @param rotational_score_position The position being added
  /// @return __PIDArmMotionBuilder*__ Pointer to the current builder
  PIDArmMotionBuilder* withRotationalScorePosition(
      double rotational_score_position);

  /// @brief Adds a rotational rush position to the builder
  /// @param rotational_rush_position The position being added
  /// @return __PIDArmMotionBuilder*__ Pointer to the current builder
  PIDArmMotionBuilder* withRotationalRushPosition(
      double rotational_rush_position);

  /// @brief Adds a rotational alliance stake position to the builder
  /// @param rotational_alliance_stake_position The position being added
  /// @return __PIDArmMotionBuilder*__ Pointer to the current builder
  PIDArmMotionBuilder* withRotationalAllianceStakePosition(
      double rotational_alliance_stake_position);

  /// @brief Adds a rotational climb position to the builder
  /// @param rotational_climb_position The position being added
  /// @return __PIDArmMotionBuilder*__ Pointer to the current builder
  PIDArmMotionBuilder* withRotationalClimbPosition(
      double rotational_climb_position);

  /// @brief Adds a rotational ready intermediate position to the builder
  /// @param rotational_ready_intermediate_position The position being added
  /// @return __PIDArmMotionBuilder*__ Pointer to the current builder
  PIDArmMotionBuilder* withRotationalReadyIntermediatePosition(
      double rotational_ready_intermediate_position);

  /// @brief Adds a rotational score intermediate position to the builder
  /// @param rotational_score_intermediate_position The position being added
  /// @return __PIDArmMotionBuilder*__ Pointer to the current builder
  PIDArmMotionBuilder* withRotationalScoreIntermediatePosition(
      double rotational_score_intermediate_position);

  /// @brief Adds a rotational rush intermediate position to the builder
  /// @param rotational_rush_intermediate_position The position being added
  /// @return __PIDArmMotionBuilder*__ Pointer to the current builder
  PIDArmMotionBuilder* withRotationalRushIntermediatePosition(
      double rotational_rush_intermediate_position);

  /// @brief Adds a rotational alliance stake intermediate position to the
  /// builder
  /// @param rotational_alliance_stake_intermediate_position The position being
  /// added
  /// @return __PIDArmMotionBuilder*__ Pointer to the current builder
  PIDArmMotionBuilder* withRotationalAllianceStakeIntermediatePosition(
      double rotational_alliance_stake_intermediate_position);

  /// @brief Adds a rotational position tolerance to the builder
  /// @param rotational_tolerance The tolerance being added
  /// @return __PIDArmMotionBuilder*__ Pointer to the current builder
  PIDArmMotionBuilder* withRotationalTolerance(double rotational_tolerance);

  /// @brief Adds a linear neutral position to the builder
  /// @param linear_neutral_position The position being added
  /// @return __PIDArmMotionBuilder*__ Pointer to the current builder
  PIDArmMotionBuilder* withLinearNeutralPosition(
      double linear_neutral_position);

  /// @brief Adds a linear load position to the builder
  /// @param linear_load_position The position being added
  /// @return __PIDArmMotionBuilder*__ Pointer to the current builder
  PIDArmMotionBuilder* withLinearLoadPosition(double linear_load_position);

  /// @brief Adds a linear ready position to the builder
  /// @param linear_ready_position The position being added
  /// @return __PIDArmMotionBuilder*__ Pointer to the current builder
  PIDArmMotionBuilder* withLinearReadyPosition(double linear_ready_position);

  /// @brief Adds a linear score position to the builder
  /// @param linear_score_position The position being added
  /// @return __PIDArmMotionBuilder*__ Pointer to the current builder
  PIDArmMotionBuilder* withLinearScorePosition(double linear_score_position);

  /// @brief Adds a linear rush position to the builder
  /// @param linear_rush_position The position being added
  /// @return __PIDArmMotionBuilder*__ Pointer to the current builder
  PIDArmMotionBuilder* withLinearRushPosition(double linear_rush_position);

  /// @brief Adds a linear alliance stake position to the builder
  /// @param linear_alliance_stake_position The position being added
  /// @return __PIDArmMotionBuilder*__ Pointer to the current builder
  PIDArmMotionBuilder* withLinearAllianceStakePosition(
      double linear_alliance_stake_position);

  /// @brief Adds a linear climb ready position to the builder
  /// @param linear_climb_ready_position The position being added
  /// @return __PIDArmMotionBuilder*__ Pointer to the current builder
  PIDArmMotionBuilder* withLinearClimbReadyPosition(
      double linear_climb_ready_position);

  /// @brief Adds a linear climb position to the builder
  /// @param linear_climb_position The position being added
  /// @return __PIDArmMotionBuilder*__ Pointer to the current builder
  PIDArmMotionBuilder* withLinearClimbPosition(double linear_climb_position);

  /// @brief Adds a linear position tolerance to the builder
  /// @param linear_tolerance The tolerance being added
  /// @return __PIDArmMotionBuilder*__ Pointer to the current builder
  PIDArmMotionBuilder* withLinearTolerance(double linear_tolerance);

  /// @brief Builds a new arm motion controller
  /// @return __std::unique_ptr<PIDArmMotion>__ The built arm motion controller
  std::unique_ptr<PIDArmMotion> build();
};
}  // namespace arm
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif