#ifndef __DEFAULT_CONFIG_HPP__
#define __DEFAULT_CONFIG_HPP__

#include <memory>

#include "pros/abstract_motor.hpp"
#include "pros/distance.hpp"
#include "pros/imu.hpp"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include "pvegas/config/IConfig.hpp"
#include "pvegas/control/ControlSystem.hpp"
#include "pvegas/control/motion/MotionControl.hpp"
#include "pvegas/control/motion/PIDDriveStraightBuilder.hpp"
#include "pvegas/control/motion/PIDGoToPointBuilder.hpp"
#include "pvegas/control/motion/PIDTurnBuilder.hpp"
#include "pvegas/control/path/PIDPathFollowerBuilder.hpp"
#include "pvegas/control/path/PathFollowerControl.hpp"
#include "pvegas/hal/TrackingWheel.hpp"
#include "pvegas/io/IController.hpp"
#include "pvegas/io/IDistanceSensor.hpp"
#include "pvegas/io/IDistanceTracker.hpp"
#include "pvegas/io/IInertialSensor.hpp"
#include "pvegas/io/IRotationSensor.hpp"
#include "pvegas/pros_adapters/ProsClock.hpp"
#include "pvegas/pros_adapters/ProsController.hpp"
#include "pvegas/pros_adapters/ProsDelayer.hpp"
#include "pvegas/pros_adapters/ProsDistanceSensor.hpp"
#include "pvegas/pros_adapters/ProsInertialSensor.hpp"
#include "pvegas/pros_adapters/ProsMutex.hpp"
#include "pvegas/pros_adapters/ProsRotationSensor.hpp"
#include "pvegas/pros_adapters/ProsTask.hpp"
#include "pvegas/pros_adapters/ProsV5Motor.hpp"
#include "pvegas/robot/Robot.hpp"
#include "pvegas/robot/subsystems/drivetrain/DirectDriveBuilder.hpp"
#include "pvegas/robot/subsystems/drivetrain/DrivetrainSubsystem.hpp"
#include "pvegas/robot/subsystems/drivetrain/IDriveTrain.hpp"
#include "pvegas/robot/subsystems/odometry/DistancePositionResetterBuilder.hpp"
#include "pvegas/robot/subsystems/odometry/InertialPositionTrackerBuilder.hpp"
#include "pvegas/robot/subsystems/odometry/OdometrySubsystem.hpp"
#include "pvegas/rtos/IClock.hpp"
#include "pvegas/rtos/IDelayer.hpp"
#include "pvegas/rtos/IMutex.hpp"
#include "pvegas/rtos/ITask.hpp"
namespace pvegas {
namespace config {
class DefaultConfig : public IConfig {
 private:
  static constexpr char CONFIG_NAME[]{"DEFAULT CONFIG"};

  // --CONTROL SYSTEM CONSTANTS--
  // motion control
  // drive straight

  // kp value for the drive straight linear pid controller
  static constexpr double PID_DRIVE_STRAIGHT_LINEAR_KP{12.0};
  // ki value for the drive straight linear pid controller
  static constexpr double PID_DRIVE_STRAIGHT_LINEAR_KI{0};
  // kd value for the drive straight linear pid controller
  static constexpr double PID_DRIVE_STRAIGHT_LINEAR_KD{800};
  // kp value for the drive straight rotational pid controller
  static constexpr double PID_DRIVE_STRAIGHT_ROTATIONAL_KP{128.0};
  // ki value for the drive straight rotational pid controller
  static constexpr double PID_DRIVE_STRAIGHT_ROTATIONAL_KI{0.001};
  // kd value for the drive straight rotational pid controller
  static constexpr double PID_DRIVE_STRAIGHT_ROTATIONAL_KD{800};
  // the target tolerance of the drive straight algorithm
  static constexpr double PID_DRIVE_STRAIGHT_TARGET_TOLERANCE{2.0};
  // the target velocity of the drive straight algorithm
  static constexpr double PID_DRIVE_STRAIGHT_TARGET_VELOCITY{1.0};

  // go to point

  // kp value for the go to point linear pid controller
  static constexpr double PID_GO_TO_POINT_LINEAR_KP{12.0};
  // ki value for the go to point linear pid controller
  static constexpr double PID_GO_TO_POINT_LINEAR_KI{0};
  // kd value of the go to point linear pid controller
  static constexpr double PID_GO_TO_POINT_LINEAR_KD{800};
  // kp value for the go to point rotational pid controller
  static constexpr double PID_GO_TO_POINT_ROTATIONAL_KP{128.0};
  // ki value for the go to point rotational pid controller
  static constexpr double PID_GO_TO_POINT_ROTATIONAL_KI{0.001};
  // kd value for the go to point rotational pid controller
  static constexpr double PID_GO_TO_POINT_ROTATIONAL_KD{800};
  // the target tolerance of the go to point algorithm
  static constexpr double PID_GO_TO_POINT_TARGET_TOLERANCE{2.0};
  // the target velocity of the go to point algorithm
  static constexpr double PID_GO_TO_POINT_TARGET_VELOCITY{1.0};

  // turn

  // kp value for the turn rotational pid controller
  static constexpr double PID_TURN_ROTATIONAL_KP{180.0};
  // ki value for the turn rotational pid controller
  static constexpr double PID_TURN_ROTATIONAL_KI{0.001};
  // kd value for the turn rotational pid controller
  static constexpr double PID_TURN_ROTATIONAL_KD{10000.0};
  // the target tolerance of the turn algorithm
  static constexpr double PID_TURN_TARGET_TOLERANCE{M_PI / 50};
  // the target velocity of the turn algorithm
  static constexpr double PID_TURN_TARGET_VELOCITY{M_PI / 200};

  // path follower control

  // kp value for the linear PID controller used in the path follower
  static constexpr double PID_PATH_FOLLOWER_LINEAR_KP{12.0};
  // ki value for the linear PID controller used in the path follower
  static constexpr double PID_PATH_FOLLOWER_LINEAR_KI{0.0};
  // kd value for the linear PID controller used in the path follower
  static constexpr double PID_PATH_FOLLOWER_LINEAR_KD{640.0};
  // kp value for the rotational PID controller used in the path follower
  static constexpr double PID_PATH_FOLLOWER_ROTATIONAL_KP{640.0};
  // ki value for the rotational PID controller used in the path follower
  static constexpr double PID_PATH_FOLLOWER_ROTATIONAL_KI{0.0};
  // kd value for the rotational PID controller used in the path follower
  static constexpr double PID_PATH_FOLLOWER_ROTATIONAL_KD{10000.0};
  // path follower follow distance
  static constexpr double PID_PATH_FOLLOWER_FOLLOW_DISTANCE{12.0};
  // path follower target tolerance
  static constexpr double PID_PATH_FOLLOWER_TARGET_TOLERANCE{3.0};
  // path follower target velocity
  static constexpr double PID_PATH_FOLLOWER_TARGET_VELOCITY{1.0};

  // -----PORT NUMBERS-----
  // DRIVE MOTORS

  // first left drive motor
  static constexpr int8_t DRIVE_LEFT_MOTOR_1{-4};
  // second left drive motor
  static constexpr int8_t DRIVE_LEFT_MOTOR_2{3};
  // third left drive motor
  static constexpr int8_t DRIVE_LEFT_MOTOR_3{-2};
  // fourth left drive motor
  static constexpr int8_t DRIVE_LEFT_MOTOR_4{1};
  // first right drive motor
  static constexpr int8_t DRIVE_RIGHT_MOTOR_1{11};
  // second right drive motor
  static constexpr int8_t DRIVE_RIGHT_MOTOR_2{-12};
  // third right drive motor
  static constexpr int8_t DRIVE_RIGHT_MOTOR_3{13};
  // fourth right drive motor
  static constexpr int8_t DRIVE_RIGHT_MOTOR_4{-14};

  // ODOMETRY PORTS

  // left tracking wheel
  static constexpr int8_t ODOMETRY_LEFT_TRACKING_WHEEL{0};
  // right tracking wheel
  static constexpr int8_t ODOMETRY_RIGHT_TRACKING_WHEEL{0};
  // inertial sensor
  static constexpr int8_t ODOMETRY_INERTIAL_SENSOR{0};
  // distance sensor
  static constexpr int8_t ODOMETRY_DISTANCE_SENSOR{0};

  // -----MISC VALUES-----

  // drive gearset
  static constexpr pros::MotorGearset DRIVE_GEARSET{pros::E_MOTOR_GEAR_BLUE};
  // drive ratio of motor voltage to velocity
  static constexpr double DRIVE_VELOCITY_TO_VOLTAGE{1.0};
  // radius of the robot
  static constexpr double ROBOT_RADIUS{7.25};
  // radius of the drive wheels
  static constexpr double DRIVE_WHEEL_RADIUS{1.25};
  // radius of the tracking wheels
  static constexpr double TRACKING_WHEEL_RADIUS{1.0};
  // left offset of the left tracking wheel
  static constexpr double LEFT_TRACKING_WHEEL_OFFSET{4.8};
  // left offset of the right tracking wheel
  static constexpr double RIGHT_TRACKING_WHEEL_OFFSET{8.69};
  // position resetter x offset
  static constexpr double RESETTER_LOCAL_X_OFFSET{7.5};
  // position resetter y offset
  static constexpr double RESETTER_LOCAL_Y_OFFSET{0.0};
  // position resetter angular offset
  static constexpr double RESETTER_LOCAL_THETA_OFFSET{0.0};

 public:
  std::string getName() override;

  std::shared_ptr<control::ControlSystem> buildControlSystem() override;

  std::shared_ptr<io::IController> buildController() override;

  std::shared_ptr<robot::Robot> buildRobot() override;
};
}  // namespace config
}  // namespace pvegas
#endif