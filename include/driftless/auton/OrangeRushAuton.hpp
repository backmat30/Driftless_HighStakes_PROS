#ifndef __ORANGE_AUTON_HPP__
#define __ORANGE_AUTON_HPP__

#include "driftless/auton/IAuton.hpp"
#include "driftless/control/Point.hpp"
#include "driftless/control/motion/ETurnDirection.hpp"
#include "driftless/control/path/BezierCurveInterpolation.hpp"
#include "driftless/control/path/IPathFollower.hpp"
#include "driftless/robot/subsystems/odometry/Position.hpp"
#include "driftless/utils/UtilityFunctions.hpp"

namespace driftless {
namespace auton {
class OrangeRushAuton : public IAuton {
 private:
  static constexpr char AUTON_NAME[]{"ORANGE_RUSH"};

  // SUBSYSTEM NAMES

  static constexpr char ARM_SUBSYSTEM_NAME[]{"ARM"};

  static constexpr char CLAMP_SUBSYSTEM_NAME[]{"CLAMP"};

  static constexpr char DRIVE_SUBSYSTEM_NAME[]{"DIFFERENTIAL DRIVE"};

  static constexpr char ELEVATOR_SUBSYSTEM_NAME[]{"ELEVATOR"};

  static constexpr char INTAKE_SUBSYSTEM_NAME[]{"INTAKE"};

  static constexpr char ODOMETRY_SUBSYSTEM_NAME[]{"ODOMETRY"};

  static constexpr char RING_SORT_SUBSYSTEM_NAME[]{"RING SORT"};

  // CONTROL NAMES

  static constexpr char PATH_FOLLOWER_CONTROL_NAME[]{"PATH FOLLOWING"};

  static constexpr char MOTION_CONTROL_NAME[]{"MOTION"};

  // COMMAND NAMES

  // arm commands

  static constexpr char ARM_CALIBRATE_COMMAND[]{"CALIBRATE"};

  static constexpr char ARM_GO_ALLIANCE_COMMAND[]{"GO ALLIANCE STAKE"};

  static constexpr char ARM_GO_NEUTRAL_COMMAND[]{"GO NEUTRAL"};

  static constexpr char ARM_GO_LOAD_COMMAND[]{"GO LOAD"};

  static constexpr char ARM_GO_READY_COMMAND[]{"GO READY"};

  static constexpr char ARM_GO_SCORE_COMMAND[]{"GO SCORE"};

  static constexpr char ARM_GO_PREVIOUS_COMMAND[]{"GO PREVIOUS"};

  // clamp commands

  static constexpr char CLAMP_SET_STATE_COMMAND[]{"SET STATE"};

  // drive commands

  static constexpr char DRIVE_SET_VOLTAGE_COMMAND[]{"SET VOLTAGE"};

  // elevator commands

  static constexpr char ELEVATOR_SET_POSITION[]{"SET POSITION"};

  static constexpr char ELEVATOR_SET_VOLTAGE[]{"SET VOLTAGE"};

  static constexpr char ELEVATOR_DEPLOY_REJECTOR_COMMAND[]{"DEPLOY REJECTOR"};

  static constexpr char ELEVATOR_RETRACT_REJECTOR_COMMAND[]{"RETRACT REJECTOR"};

  // intake commands

  static constexpr char INTAKE_SPIN_COMMAND[]{"SPIN"};

  static constexpr char INTAKE_SET_HEIGHT_COMMAND[]{"SET HEIGHT"};

  // odometry commands

  static constexpr char ODOMETRY_SET_POSITION_COMMAND[]{"SET POSITION"};

  // path follower commands

  static constexpr char FOLLOW_PATH_COMMAND[]{"FOLLOW PATH"};

  static constexpr char SET_PATH_FOLLOWER_VELOCTY_COMMAND[]{"SET VELOCITY"};

  // motion commands

  static constexpr char GO_TO_POINT_COMMAND[]{"GO TO POINT"};

  static constexpr char SET_GO_TO_POINT_VELOCITY_COMMAND[]{
      "SET GO TO POINT VELOCITY"};

  static constexpr char TURN_TO_POINT_COMMAND[]{"TURN TO POINT"};

  static constexpr char TURN_TO_ANGLE_COMMAND[]{"TURN TO ANGLE"};

  static constexpr char DRIVE_STRAIGHT_COMMAND[]{"DRIVE STRAIGHT"};

  // STATE NAMES

  // arm states

  // clamp states

  // drive states

  // elevator states

  static constexpr char ELEVATOR_POSITION_STATE[]{"GET POSITION"};

  // intake states

  // odometry states

  static constexpr char ODOMETRY_GET_POSITION_STATE[]{"GET POSITION"};

  // ring sorter states

  static constexpr char RING_SORT_HAS_RING_STATE[]{"HAS RING"};

  static constexpr char RING_SORT_GET_RGB_STATE[]{"GET RGB"};

  static constexpr char RING_SORT_GET_DISTANCE_TO_END_STATE[]{
      "GET DISTANCE TO END"};

  // path follower states

  static constexpr char PATH_FOLLOWER_TARGET_REACHED_STATE[]{"TARGET REACHED"};

  // motion states

  static constexpr char GO_TO_POINT_TARGET_REACHED_STATE[]{
      "GO TO POINT TARGET REACHED"};

  static constexpr char TURN_TARGET_REACHED_STATE[]{"TURN TARGET REACHED"};

  static constexpr char DRIVE_STRAIGHT_TARGET_REACHED_STATE[]{
      "DRIVE STRAIGHT TARGET REACHED"};

  // MISC VALUES

  static constexpr uint8_t LOOP_DELAY{5};

  std::shared_ptr<rtos::IClock> m_clock{};

  std::unique_ptr<rtos::IDelayer> m_delayer{};

  std::shared_ptr<robot::Robot> m_robot{};

  std::shared_ptr<control::ControlSystem> m_control_system{};

  std::shared_ptr<alliance::IAlliance> m_alliance{};

  double ring_sort_latest_ring_pos{-__DBL_MAX__};

  void calibrateArm();

  void armGoNeutral();

  void armGoLoad();

  void armGoAllianceStake();

  void setClamp(bool clamped);

  void setElevatorVoltage(double voltage);

  void updateRingSort();

  void waitForAllianceRing(uint32_t timeout);

  void waitForOpposingRing(uint32_t timeout);

  void spinIntake(double voltage);

  void setIntakeHeight(bool high);

  void setOdomPosition(double x, double y, double theta);

  void followPath(std::vector<control::Point>& path, double velocity);

  void setFollowPathVelocity(double velocity);

  void goToPoint(double x, double y, double velocity);

  void setGoToPointVelocity(double velocity);

  void waitForGoToPoint(double target_x, double target_y, uint32_t timeout,
                        double tolerance);

  void turnToPoint(double x, double y, double velocity,
                   control::motion::ETurnDirection direction);

  void waitForTurnToPoint(double x, double y, uint32_t timeout,
                          double tolerance);

  void turnToAngle(double theta, double velocity,
                   control::motion::ETurnDirection direction);

  void waitForTurnToAngle(double theta, uint32_t timeout, double tolerance);

  void driveStraight(double distance, double velocity, double theta);

  void waitForDriveStraight(double target_distance, uint32_t timeout,
                            double tolerance);

  void delay(uint32_t delay_time);

  uint32_t getTime();

  robot::subsystems::odometry::Position getOdomPosition();

  double getOdomVelocity();

  bool followPathTargetReached();

  bool goToPointTargetReached();

  bool turnTargetReached();

  bool driveStraightTargetReached();

  bool hasAllianceRing();

  bool hasOpposingRing();

 public:
  std::string getName() override;

  void init(std::shared_ptr<robot::Robot>& robot,
            std::shared_ptr<control::ControlSystem>& control_system) override;

  void run(std::shared_ptr<driftless::robot::Robot>& robot,
           std::shared_ptr<driftless::control::ControlSystem>& control_system,
           std::shared_ptr<driftless::alliance::IAlliance>& alliance,
           std::shared_ptr<rtos::IClock>& clock,
           std::unique_ptr<rtos::IDelayer>& delayer) override;
};
}  // namespace auton
}  // namespace driftless
#endif