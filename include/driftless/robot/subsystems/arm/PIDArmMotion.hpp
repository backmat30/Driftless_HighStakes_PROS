#ifndef __PID_ARM_MOTION_HPP__
#define __PID_ARM_MOTION_HPP__

#include <cmath>
#include <memory>

#include "driftless/control/PID.hpp"
#include "driftless/hal/MotorGroup.hpp"
#include "driftless/io/IPotentiometer.hpp"
#include "driftless/io/IRotationSensor.hpp"
#include "driftless/robot/subsystems/arm/IArmMotion.hpp"
#include "driftless/rtos/IDelayer.hpp"
#include "driftless/rtos/IMutex.hpp"
#include "driftless/rtos/ITask.hpp"

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

/// @brief The class for PID arm motion control
/// @author Matthew Backman
class PIDArmMotion : public IArmMotion {
 private:
  enum class EState {
    NEUTRAL,
    LOAD,
    READY,
    SCORE,
    RUSH,
    ALLIANCE_STAKE,
    CLIMB_READY,
    CLIMB,
    LOAD_INTERMEDIATE,
    READY_INTERMEDIATE,
    SCORE_INTERMEDIATE,
    RUSH_INTERMEDIATE,
    ALLIANCE_STAKE_INTERMEDIATE,
    NEUTRAL_MOTION,
    LOAD_MOTION,
    READY_MOTION,
    SCORE_MOTION,
    RUSH_MOTION,
    ALLIANCE_STAKE_MOTION,
    CLIMB_READY_MOTION,
    CLIMB_MOTION
  };
  // delay between task updates
  static constexpr uint8_t TASK_DELAY{10};

  // the conversion factor from motor rotations to arm rotations
  static constexpr double MOTOR_TO_ARM_ROTATIONS{1.0};

  // the conversion factor from potentiometer or rotation sensor rotations to
  // arm rotations
  static constexpr double SENSOR_TO_ARM_ROTATIONS{3.0 / 4.0};

  // runs task updates on loop
  static void taskLoop(void* params);

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

  // the rotational position when ready
  double m_rotational_ready_position{};

  // the rotational position when scoring
  double m_rotational_score_position{};

  // the rotational position when rushing
  double m_rotational_rush_position{};

  // the rotational position when scoring on the alliance stake
  double m_rotational_alliance_stake_position{};

  double m_rotational_climb_position{};

  // the rotational position of the intermediate load position
  double m_rotational_load_intermediate_position{};

  // the rotational position of the intermediate ready position
  double m_rotational_ready_intermediate_position{};

  // the rotational position of the intermediate
  double m_rotational_score_intermediate_position{};

  // the rotational position of the intermediate rush position
  double m_rotational_rush_intermediate_position{};

  // the rotational position of the intermediate alliance stake position
  double m_rotational_alliance_stake_intermediate_position{};

  // the rotational position tolerance
  double m_rotational_tolerance{};

  // the linear position when neutral
  double m_linear_neutral_position{};

  // the linear position when loading
  double m_linear_load_position{};

  // the linear position when ready
  double m_linear_ready_position{};

  // the linear position when scoring
  double m_linear_score_position{};

  // the linear position when rushing
  double m_linear_rush_position{};

  // the linear position when scoring on the alliance stake
  double m_linear_alliance_stake_position{};

  double m_linear_climb_ready_position{};

  double m_linear_climb_position{};

  // the linear position of the intermediate load position
  double m_linear_load_intermediate_position{};

  // the linear position tolerance
  double m_linear_tolerance{};

  // the current position of the arm
  EState state{EState::LOAD};

  // the previous position of the arm
  EState previous_state{EState::LOAD};

  // the target rotational position
  double rotational_target_position{};

  // the target linear position
  double linear_target_position{};

  // the last time calibrate was called
  uint32_t calibrate_time{};

  // whether the arm is calibrating or not
  bool calibrating{};

  // update all task-related features
  void taskUpdate();

  // update the state of the arm
  void updateState();

  /// @brief Updates the previous state of the arm
  void updatePreviousState();

  // update the position of the arm motors
  void updatePosition();

  // gets the rotational position
  double getRotationalPosition();

  // gets the linear position
  double getLinearPosition();

  /// @brief Gets the average efficiency of the rotation motors
  /// @return __double__ the torque of the rotational motors
  double getRotationalEfficiency();

  /// @brief Gets the average efficiency of the linear motors
  /// @return __double__ the torque of the linear motors
  double getLinearEfficiency();

 public:
  /// @brief Initializes the arm motion control
  void init() override;

  /// @brief Runs the arm motion control
  void run() override;

  /// @brief Finds the zero-position of the arm and calibrates
  void calibrate() override;

  /// @brief Goes to the neutral position
  void goNeutral() override;

  /// @brief Goes to the loading position
  void goLoad() override;

  /// @brief Goes to the ready position
  void goReady() override;

  /// @brief Goes to the score position
  void goScore() override;

  /// @brief Goes to the rush position
  void goRush() override;

  /// @brief Puts the arm at the alliance stake position
  void goAllianceStake() override;

  /// @brief Goes to the climb ready position
  void goClimbReady() override;

  /// @brief Goes to the climb position
  void goClimb() override;

  /// @brief Goes to the previous position
  void goPrevious() override;

  /// @brief Determines if the arm is in the neutral position
  /// @return __bool__ True if at neutral, false otherwise
  bool isAtNeutral() override;

  /// @brief Determines if the arm is going to the neutral position
  /// @return __bool__ True if going neutral, false otherwise
  bool isGoingNeutral() override;

  /// @brief Determines if the arm is in the loading position
  /// @return __bool__ True if at load, false otherwise
  bool isAtLoad() override;

  /// @brief Determines if the arm is going to the loading position
  /// @return __bool__ True if going load, false otherwise
  bool isGoingLoad() override;

  /// @brief Determines if the arm is in the ready position
  /// @return __bool__ True if at ready, false otherwise
  bool isAtReady() override;

  /// @brief Determines if the arm is going to the ready position
  /// @return __bool__ True if going ready, false otherwise
  bool isGoingReady() override;

  /// @brief Determines if the arm is in the score position
  /// @return __bool__ True if at score, false otherwise
  bool isAtScore() override;

  /// @brief Determines if the arm is going to the score position
  /// @return __bool__ True if going score, false otherwise
  bool isGoingScore() override;

  /// @brief Determines if the arm is at the rush position
  /// @return __bool__ True if at rush, false otherwise
  bool isAtRush() override;

  /// @brief Determines if the arm is going to the rush position
  /// @return __bool__ True if going rush, false otherwise
  bool isGoingRush() override;

  /// @brief Determines if the arm is at the alliance stake position
  /// @return __bool__ True if at alliance stake, false otherwise
  bool isAtAllianceStake() override;

  /// @brief Determines if the arm is going to the alliance stake position
  /// @return __bool__ True if going alliance stake, false otherwise
  bool isGoingAllianceStake() override;

  /// @brief Determines if the arm is at the climb ready position
  /// @return __bool__ True if at climb ready, false otherwise
  bool isAtClimbReady() override;

  /// @brief Determines if the arm is at the climb position
  /// @return __bool__ True if at climb, false otherwise
  bool isAtClimb() override;

  /// @brief Sets the internal clock of the system
  /// @param clock The new internal clock
  void setClock(const std::unique_ptr<driftless::rtos::IClock>& clock);

  /// @brief Sets the delayer
  /// @param delayer The new delayer
  void setDelayer(const std::unique_ptr<driftless::rtos::IDelayer>& delayer);

  /// @brief Sets the mutex
  /// @param mutex The new mutex
  void setMutex(std::unique_ptr<driftless::rtos::IMutex>& mutex);

  /// @brief Sets the task
  /// @param task The new task
  void setTask(std::unique_ptr<driftless::rtos::ITask>& task);

  /// @brief Sets the rotation sensor
  /// @param rotation_sensor The new rotation sensor
  void setRotationSensor(
      std::unique_ptr<driftless::io::IRotationSensor>& rotation_sensor);

  /// @brief Sets the potentiometer
  /// @param potentiometer The new potentiometer
  void setPotentiometer(
      std::unique_ptr<driftless::io::IPotentiometer>& potentiometer);

  /// @brief Sets the rotation motors
  /// @param rotation_motors The new rotation motors
  void setRotationMotors(driftless::hal::MotorGroup& rotation_motors);

  /// @brief Sets the linear motors
  /// @param linear_motors The new linear motors
  void setLinearMotors(driftless::hal::MotorGroup& linear_motors);

  /// @brief Sets the rotational PID controller
  /// @param rotational_pid The new rotational PID controller
  void setRotationalPID(driftless::control::PID rotational_pid);

  /// @brief Sets the linear PID controller
  /// @param linear_pid The new linear PID controller
  void setLinearPID(driftless::control::PID linear_pid);

  /// @brief Sets the rotational neutral position
  /// @param rotational_neutral_position The new rotational neutral position
  void setRotationalNeutralPosition(double rotational_neutral_position);

  /// @brief Sets the rotational loading position
  /// @param rotational_load_position The new rotational load position
  void setRotationalLoadPosition(double rotational_load_position);

  /// @brief Sets the rotational ready position
  /// @param rotational_ready_position The new rotational ready position
  void setRotationalReadyPosition(double rotational_ready_position);

  /// @brief Sets the rotational score position
  /// @param rotational_score_position The new rotational score position
  void setRotationalScorePosition(double rotational_score_position);

  /// @brief Sets the rotational rush position
  /// @param rotational_rush_position The new rotational rush position
  void setRotationalRushPosition(double rotational_rush_position);

  /// @brief Sets the rotational alliance stake position of the arm
  /// @param rotational_alliance_stake_position The new rotational alliance
  /// stake position
  void setRotationalAllianceStakePosition(
      double rotational_alliance_stake_position);

  /// @brief Sets the rotational climb ready position
  /// @param rotational_climb_position The new rotational climb position
  void setRotationalClimbPosition(double rotational_climb_position);

  /// @brief Sets the rotational ready intermediate position
  /// @param rotational_ready_intermediate_position The new position
  void setRotationalReadyIntermediatePosition(
      double rotational_ready_intermediate_position);

  /// @brief Sets the rotational score intermediate position
  /// @param rotational_score_intermediate_position The new position
  void setRotationalScoreIntermediatePosition(
      double rotational_score_intermediate_position);

  /// @brief Sets the rotational rush intermediate position
  /// @param rotational_rush_intermediate_position The new position
  void setRotationalRushIntermediatePosition(
      double rotational_rush_intermediate_position);

  /// @brief Sets the rotational alliance stake intermediate position
  /// @param rotational_alliance_stake_intermediate_position The new position
  void setRotationalAllianceStakeIntermediatePosition(
      double rotational_alliance_stake_intermediate_position);

  /// @brief Sets the rotational position tolerance
  /// @param rotational_tolerance The new tolerance
  void setRotationalTolerance(double rotational_tolerance);

  /// @brief Sets the linear neutral position
  /// @param linear_neutral_position The new linear neutral position
  void setLinearNeutralPosition(double linear_neutral_position);

  /// @brief Sets the linear loading position
  /// @param linear_load_position The new linear load position
  void setLinearLoadPosition(double linear_load_position);

  /// @brief Sets the linear ready position
  /// @param linear_ready_position The new linear ready position
  void setLinearReadyPosition(double linear_ready_position);

  /// @brief Sets the linear score position
  /// @param linear_score_position The new linear score position
  void setLinearScorePosition(double linear_score_position);

  /// @brief Sets the linear rush position
  /// @param linear_rush_position The new linear rush position
  void setLinearRushPosition(double linear_rush_position);

  /// @brief Sets the linear alliance stake position
  /// @param linear_alliance_stake_position The new position
  void setLinearAllianceStakePosition(double linear_alliance_stake_position);

  /// @brief Sets the linear climb ready position
  /// @param linear_climb_ready_position The new position
  void setLinearClimbReadyPosition(double linear_climb_ready_position);

  /// @brief Sets the linear climb position
  /// @param linear_climb_position The new position
  void setLinearClimbPosition(double linear_climb_position);

  // sets the linear position tolerance
  void setLinearTolerance(double linear_tolerance);
};
}  // namespace arm
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif