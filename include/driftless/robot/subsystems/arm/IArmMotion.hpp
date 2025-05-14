#ifndef __I_ARM_MOTION_HPP__
#define __I_ARM_MOTION_HPP__

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

/// @brief The interface for arm motion control
/// @author Matthew Backman
class IArmMotion {
 public:
  /// @brief Destroys the object
  virtual ~IArmMotion() = default;

  /// @brief Initializes the arm motion controller
  virtual void init() = 0;

  /// @brief Runs the arm motion controller
  virtual void run() = 0;

  /// @brief Finds the zero-position of the arm and calibrates it
  virtual void calibrate() = 0;

  /// @brief Goes to the neutral position
  virtual void goNeutral() = 0;

  /// @brief Goes to the loading position
  virtual void goLoad() = 0;

  /// @brief Goes to the ready position
  virtual void goReady() = 0;

  /// @brief Goes to the score position
  virtual void goScore() = 0;

  /// @brief Goes to the rush position
  virtual void goRush() = 0;

  /// @brief Goes to the alliance position
  virtual void goAllianceStake() = 0;

  /// @brief Goes to the ready position for climbing
  virtual void goClimbReady() = 0;

  /// @brief Goes to the climb position
  virtual void goClimb() = 0;

  /// @brief Goes to the previous position
  virtual void goPrevious() = 0;

  /// @brief Determines if the arm is in the neutral position
  /// @return __bool__ True if at neutral, false otherwise
  virtual bool isAtNeutral() = 0;

  /// @brief Determines if the arm is going to the neutral position
  /// @return __bool__ True if going neutral, false otherwise
  virtual bool isGoingNeutral() = 0;

  /// @brief Determines if the arm is in the loading position
  /// @return __bool__ True if at load, false otherwise
  virtual bool isAtLoad() = 0;

  /// @brief Determines if the arm is going to the loading position
  /// @return __bool__ True if going load, false otherwise
  virtual bool isGoingLoad() = 0;

  /// @brief Determines if the arm is in the ready position
  /// @return __bool__ True if at ready, false otherwise
  virtual bool isAtReady() = 0;

  /// @brief Determines if the arm is going to the ready position
  /// @return __bool__ True if going ready, false otherwise
  virtual bool isGoingReady() = 0;

  /// @brief Determines if the arm is in the score position
  /// @return __bool__ True if at score, false otherwise
  virtual bool isAtScore() = 0;

  /// @brief Determines if the arm is going to the score position
  /// @return __bool__ True if going score, false otherwise
  virtual bool isGoingScore() = 0;

  /// @brief Determines if the arm is at the rush position
  /// @return __bool__ True if at rush, false otherwise
  virtual bool isAtRush() = 0;

  /// @brief Determines if the arm is going to the rush position
  /// @return __bool__ True if going rush, false otherwise
  virtual bool isGoingRush() = 0;

  /// @brief Determines if the arm is at the alliance stake position
  /// @return __bool__ True if at alliance stake, false otherwise
  virtual bool isAtAllianceStake() = 0;

  /// @brief Determines if the arm is going to the alliance stake position
  /// @return __bool__ True if going alliance stake, false otherwise
  virtual bool isGoingAllianceStake() = 0;

  /// @brief Determines if the arm is at the climb ready position
  /// @return __bool__ True if at climb ready, false otherwise
  virtual bool isAtClimbReady() = 0;

  virtual bool isGoingClimbReady() = 0;

  /// @brief Determines if the arm is at the climb position
  /// @return __bool__ True if at climb, false otherwise
  virtual bool isAtClimb() = 0;
};
}  // namespace arm
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif