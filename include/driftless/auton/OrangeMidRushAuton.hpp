#ifndef __ORANGE_MID_RUSH_AUTON_HPP__
#define __ORANGE_MID_RUSH_AUTON_HPP__

#include "driftless/auton/IAuton.hpp"
#include "driftless/control/Point.hpp"
#include "driftless/control/motion/ETurnDirection.hpp"
#include "driftless/control/path/BezierCurveInterpolation.hpp"
#include "driftless/control/path/IPathFollower.hpp"
#include "driftless/robot/subsystems/odometry/Position.hpp"
#include "driftless/utils/UtilityFunctions.hpp"

namespace driftless {
namespace auton {
class OrangeMidRushAuton : public IAuton {
 private:
  static constexpr char AUTON_NAME[]{"ORANGE_MID_RUSH"};

  // MISC VALUES

  static constexpr uint8_t LOOP_DELAY{5};

  std::shared_ptr<rtos::IClock> m_clock{};

  std::unique_ptr<rtos::IDelayer> m_delayer{};

  std::shared_ptr<robot::Robot> m_robot{};

  std::shared_ptr<control::ControlSystem> m_control_system{};

  std::shared_ptr<processes::ProcessSystem> m_process_system{};

  std::shared_ptr<alliance::IAlliance> m_alliance{};

  /// @brief Starts the color sorter
  void startColorSort();

  /// @brief Calibrates the arm
  void calibrateArm();

  /// @brief Moves the arm to the neutral position
  void armGoNeutral();

  /// @brief Moves the arm to the load position
  void armGoLoad();

  /// @brief Moves the arm to the alliance stake position
  void armGoAllianceStake();

  /// @brief Sets the state of the clamp
  /// @param clamped __bool__ True if clamped, false if released
  void setClamp(bool clamped);

  /// @brief Sets the voltage of the elevator motors
  /// @param voltage __double__ The motor voltage
  void setElevatorVoltage(double voltage);

  /// @brief Delays until the ring sensor sees an alliance ring
  /// @param timeout __uint32_t__ The maximum time to delay for
  void waitForAllianceRing(uint32_t timeout);

  /// @brief Delays until the ring sensor sees an opposing ring
  /// @param timeout __uint32_t__ The maximum time to delay for
  void waitForOpposingRing(uint32_t timeout);

  /// @brief Sets the voltage of the intake motors
  /// @param voltage __double__ The motor voltage
  void setIntakeVoltage(double voltage);

  /// @brief sets the height of the intake
  /// @param high __bool__ True if raised, false if lowered
  void setIntakeHeight(bool high);

  /// @brief Sets the position of the odometry subsystem
  /// @param x __double__ The x coordinate
  /// @param y __double__ The y coordinate
  /// @param theta __double__ the heading of the robot
  void setOdomPosition(double x, double y, double theta);

  /// @brief Follows a given path using pure pursuit
  /// @param path __std::vector<control::Point>&__ The points along the desired
  /// path
  /// @param velocity __double__ The maximum velocity the robot should travel at
  void followPath(std::vector<control::Point>& path, double velocity);

  /// @brief Sets the max velocity of the robot while following a path
  /// @param velocity __double__ The maximum velocity the robot should travel at
  void setFollowPathVelocity(double velocity);

  /// @brief Goes to a given point on the field
  /// @param x __double__ The target x coordinate
  /// @param y __double__ The target y coordinate
  /// @param velocity __double__ The maximum velocity the robot should travel at
  void goToPoint(double x, double y, double velocity);

  /// @brief Sets the max velocity while going to a point
  /// @param velocity __double__ The maximum velocity the robot should travel at
  void setGoToPointVelocity(double velocity);

  /// @brief Delays the current task until the robot reaches the goToPoint
  /// target, or until a timeout has been reached
  /// @param target_x __double__ The target x coordinate of the goToPoint
  /// algorithm
  /// @param target_y __double__ The target y coordinate of the goToPoint
  /// algorithm
  /// @param timeout __uint32_t__ The maximum time to wait for the target to be
  /// reached
  /// @param tolerance __double__ The tolerance around the target position, in
  /// inches
  void waitForGoToPoint(double target_x, double target_y, uint32_t timeout,
                        double tolerance);

  /// @brief Turns to face a point of the field
  /// @param x __double__ The target x coordinate
  /// @param y __double__ The target y coordinate
  /// @param velocity __double__ The max velocity the robot should turn at
  /// @param direction __control::motion::ETurnDirection__ the direction to turn
  /// in
  void turnToPoint(double x, double y, double velocity,
                   control::motion::ETurnDirection direction);

  /// @brief Delays the current task until the robot faces the target point
  /// @param x __double__ The target x coordinate
  /// @param y __double__ The target y coordinate
  /// @param timeout __uint32_t__ The maximum time to wait for the target to be
  /// reached
  /// @param tolerance __double__ The tolerance around the target angle, in
  /// radians
  void waitForTurnToPoint(double x, double y, uint32_t timeout,
                          double tolerance);

  /// @brief Turns to face a given angle
  /// @param theta __double__ The angle to turn to in radians
  /// @param velocity __double__ The max velocity
  /// @param direction __control::motion::ETurnDirection__ The direction to turn
  /// in
  void turnToAngle(double theta, double velocity,
                   control::motion::ETurnDirection direction);

  /// @brief Delays the current task until the robot turns reaches the target
  /// heading
  /// @param theta __double__ The target angle in radians
  /// @param timeout __uint32_t__ The maximum time to delay for
  /// @param tolerance __double__ The tolerance around the target angle, in
  /// radians
  void waitForTurnToAngle(double theta, uint32_t timeout, double tolerance);

  /// @brief Drives forward in a given direction
  /// @param distance __double__ The distance to move in inches
  /// @param velocity __double__ The maximum velocity to travel at
  /// @param theta __double__ The heading to travel in, in inches
  void driveStraight(double distance, double velocity, double theta);

  /// @brief Delays the current task until the robot reaches the drive straight
  /// target
  /// @param target_distance __double__ The target distance to drive forward
  /// @param timeout __uint32_t__ The maximume time to delay for
  /// @param tolerance __double__ The tolerance around the target distance, in
  /// inches
  void waitForDriveStraight(double target_distance, uint32_t timeout,
                            double tolerance);

  /// @brief Delays the current task for a given time
  /// @param delay_time __uint32_t__ How long the task should be delayed, in
  /// milliseconds
  void delay(uint32_t delay_time);

  /// @brief Gets the current runtime
  /// @return __uint32_t__ The current time
  uint32_t getTime();

  /// @brief Gets the position from the odometry subsystem
  /// @return __robot::subsystems::odometry::Position__ The robot's current
  /// position
  robot::subsystems::odometry::Position getOdomPosition();

  /// @brief Gets the velocity from the odometry subsystem
  /// @return __double__ The robot's velocity, in in/s
  double getOdomVelocity();

  /// @brief Determines if the path follower has reached the target
  /// @return __bool__ True if target reached, false otherwise
  bool followPathTargetReached();

  /// @brief Determines if the goToPoint algorithm has reached the target
  /// @return __bool__ True if the target was reached, false otherwise
  bool goToPointTargetReached();

  /// @brief Determines if the turnToPoint algorithm has reached the target
  /// @return __bool__ True if the target was reached, false otherwise
  bool turnTargetReached();

  /// @brief Determines if the driveStraight algorithm has reached the target
  /// @return __bool__ True if the target was reached, false otherwise
  bool driveStraightTargetReached();

  /// @brief Determines if the ring sensor sees an alliance ring
  /// @return __bool__ True if there is an alliance ring, false otherwise
  bool hasAllianceRing();

  /// @brief Determines if the ring sensor sees an opposing ring
  /// @return __bool__ True if there is an opposing ring, false otherwise
  bool hasOpposingRing();

 public:
  /// @brief Gets the name of the auton
  /// @return __std::string__ The name
  std::string getName() override;

  /// @brief Initializes the auton
  /// @param robot __std::shared_ptr<robot::Robot>&__ The robot being controlled
  /// @param control_system __std::shared_ptr<control::ControlSystem>&__ The
  /// control system for the robot's movement
  /// @param process_system __std::shared_ptr<processes::ProcessSystem>&__ The
  /// process system to manage background processes
  void init(std::shared_ptr<robot::Robot>& robot,
            std::shared_ptr<control::ControlSystem>& control_system,
            std::shared_ptr<driftless::processes::ProcessSystem>&
                process_system) override;

  /// @brief Runs the auton
  /// @param robot __std::shared_ptr<robot::Robot>&__ The robot being controlled
  /// @param control_system __std::shared_ptr<control::ControlSystem>&__ The
  /// control system for the robot's movement
  /// @param process_system __std::shared_ptr<processes::ProcessSystem>&__ The
  /// process system to manage background processes
  /// @param alliance __std::shared_ptr<alliance::IAlliance>&__ The current
  /// alliance
  /// @param clock __std::shared_ptr<rtos::IClock>&__ The clock used
  /// @param delayer __std::shared_ptr<rtos::IDelayer>&__ The delayer used
  void run(std::shared_ptr<driftless::robot::Robot>& robot,
           std::shared_ptr<driftless::control::ControlSystem>& control_system,
           std::shared_ptr<driftless::processes::ProcessSystem>& process_system,
           std::shared_ptr<driftless::alliance::IAlliance>& alliance,
           std::shared_ptr<rtos::IClock>& clock,
           std::unique_ptr<rtos::IDelayer>& delayer) override;
};
}  // namespace auton
}  // namespace driftless
#endif