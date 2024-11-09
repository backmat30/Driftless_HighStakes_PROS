#ifndef __DEFAULT_CONFIG_HPP__
#define __DEFAULT_CONFIG_HPP__

#include <memory>

// config interface
#include "driftless/config/IConfig.hpp"

// pros object includes
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/imu.hpp"
#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include "pros/rotation.hpp"

// control system includes
#include "driftless/control/ControlSystem.hpp"
#include "driftless/control/motion/MotionControl.hpp"
#include "driftless/control/motion/PIDDriveStraightBuilder.hpp"
#include "driftless/control/motion/PIDGoToPointBuilder.hpp"
#include "driftless/control/motion/PIDTurnBuilder.hpp"
#include "driftless/control/path/PIDPathFollowerBuilder.hpp"
#include "driftless/control/path/PathFollowerControl.hpp"

// hardware interface includes
#include "driftless/hal/TrackingWheel.hpp"
#include "driftless/io/IColorSensor.hpp"
#include "driftless/io/IController.hpp"
#include "driftless/io/IDistanceSensor.hpp"
#include "driftless/io/IDistanceTracker.hpp"
#include "driftless/io/IInertialSensor.hpp"
#include "driftless/io/IMotor.hpp"
#include "driftless/io/IPiston.hpp"
#include "driftless/io/IPotentiometer.hpp"
#include "driftless/io/IRotationSensor.hpp"

// pros adapter includes
#include "driftless/pros_adapters/ProsADIPotentiometer.hpp"
#include "driftless/pros_adapters/ProsClock.hpp"
#include "driftless/pros_adapters/ProsColorSensor.hpp"
#include "driftless/pros_adapters/ProsController.hpp"
#include "driftless/pros_adapters/ProsDelayer.hpp"
#include "driftless/pros_adapters/ProsDistanceSensor.hpp"
#include "driftless/pros_adapters/ProsInertialSensor.hpp"
#include "driftless/pros_adapters/ProsMutex.hpp"
#include "driftless/pros_adapters/ProsPiston.hpp"
#include "driftless/pros_adapters/ProsRotationSensor.hpp"
#include "driftless/pros_adapters/ProsTask.hpp"
#include "driftless/pros_adapters/ProsV5Motor.hpp"

// robot include
#include "driftless/robot/Robot.hpp"

// arm subsystem includes
#include "driftless/robot/subsystems/arm/ArmSubsystem.hpp"
#include "driftless/robot/subsystems/arm/ColorRingSensorBuilder.hpp"
#include "driftless/robot/subsystems/arm/PIDArmMotionBuilder.hpp"

// clamp subsystem includes
#include "driftless/robot/subsystems/clamp/ClampSubsystem.hpp"
#include "driftless/robot/subsystems/clamp/PistonClampBuilder.hpp"

// drivetrain subsystem includes
#include "driftless/robot/subsystems/drivetrain/DirectDriveBuilder.hpp"
#include "driftless/robot/subsystems/drivetrain/DrivetrainSubsystem.hpp"
#include "driftless/robot/subsystems/drivetrain/IDriveTrain.hpp"

// elevator subsystem includes
#include "driftless/robot/subsystems/elevator/ElevatorSubsystem.hpp"
#include "driftless/robot/subsystems/elevator/PIDElevatorBuilder.hpp"

// intake subsystem includes
#include "driftless/robot/subsystems/intake/DirectIntakeBuilder.hpp"
#include "driftless/robot/subsystems/intake/IntakeSubsystem.hpp"
#include "driftless/robot/subsystems/intake/PistonHeightControlBuilder.hpp"

// odometry subsystem includes
#include "driftless/robot/subsystems/odometry/DistancePositionResetterBuilder.hpp"
#include "driftless/robot/subsystems/odometry/InertialPositionTrackerBuilder.hpp"
#include "driftless/robot/subsystems/odometry/OdometrySubsystem.hpp"

// rtos includes
#include "driftless/rtos/IClock.hpp"
#include "driftless/rtos/IDelayer.hpp"
#include "driftless/rtos/IMutex.hpp"
#include "driftless/rtos/ITask.hpp"
/// @brief Namespace for driftless library code
namespace driftless {
/// @brief Namespace for robot configurations
namespace config {
/// @brief the default robot configuration
class DefaultConfig : public IConfig {
 private:
  /// @brief the name of the config
  static constexpr char CONFIG_NAME[]{"DEFAULT CONFIG"};

  // --CONTROL SYSTEM CONSTANTS--
  // motion control
  // drive straight

  /// @brief kp value for the drive straight linear pid controller
  static constexpr double PID_DRIVE_STRAIGHT_LINEAR_KP{12.0};
  /// @brief ki value for the drive straight linear pid controller
  static constexpr double PID_DRIVE_STRAIGHT_LINEAR_KI{0};
  /// @brief kd value for the drive straight linear pid controller
  static constexpr double PID_DRIVE_STRAIGHT_LINEAR_KD{800};
  /// @brief kp value for the drive straight rotational pid controller
  static constexpr double PID_DRIVE_STRAIGHT_ROTATIONAL_KP{128.0};
  /// @brief ki value for the drive straight rotational pid controller
  static constexpr double PID_DRIVE_STRAIGHT_ROTATIONAL_KI{0.001};
  /// @brief kd value for the drive straight rotational pid controller
  static constexpr double PID_DRIVE_STRAIGHT_ROTATIONAL_KD{800};
  /// @brief the target tolerance of the drive straight algorithm
  static constexpr double PID_DRIVE_STRAIGHT_TARGET_TOLERANCE{2.0};
  /// @brief the target velocity of the drive straight algorithm
  static constexpr double PID_DRIVE_STRAIGHT_TARGET_VELOCITY{1.0};

  // go to point

  /// @brief kp value for the go to point linear pid controller
  static constexpr double PID_GO_TO_POINT_LINEAR_KP{12.0};
  /// @brief ki value for the go to point linear pid controller
  static constexpr double PID_GO_TO_POINT_LINEAR_KI{0};
  /// @brief kd value of the go to point linear pid controller
  static constexpr double PID_GO_TO_POINT_LINEAR_KD{800};
  /// @brief kp value for the go to point rotational pid controller
  static constexpr double PID_GO_TO_POINT_ROTATIONAL_KP{128.0};
  /// @brief ki value for the go to point rotational pid controller
  static constexpr double PID_GO_TO_POINT_ROTATIONAL_KI{0.001};
  /// @brief kd value for the go to point rotational pid controller
  static constexpr double PID_GO_TO_POINT_ROTATIONAL_KD{800};
  /// @brief the target tolerance of the go to point algorithm
  static constexpr double PID_GO_TO_POINT_TARGET_TOLERANCE{2.0};
  /// @brief the target velocity of the go to point algorithm
  static constexpr double PID_GO_TO_POINT_TARGET_VELOCITY{1.0};

  // turn

  /// @brief kp value for the turn rotational pid controller
  static constexpr double PID_TURN_ROTATIONAL_KP{180.0};
  /// @brief ki value for the turn rotational pid controller
  static constexpr double PID_TURN_ROTATIONAL_KI{0.001};
  /// @brief kd value for the turn rotational pid controller
  static constexpr double PID_TURN_ROTATIONAL_KD{10000.0};
  /// @brief the target tolerance of the turn algorithm
  static constexpr double PID_TURN_TARGET_TOLERANCE{M_PI / 50};
  /// @brief the target velocity of the turn algorithm
  static constexpr double PID_TURN_TARGET_VELOCITY{M_PI / 200};

  // path follower control

  /// @brief kp value for the linear PID controller used in the path follower
  static constexpr double PID_PATH_FOLLOWER_LINEAR_KP{12.0};
  /// @brief ki value for the linear PID controller used in the path follower
  static constexpr double PID_PATH_FOLLOWER_LINEAR_KI{0.0};
  /// @brief kd value for the linear PID controller used in the path follower
  static constexpr double PID_PATH_FOLLOWER_LINEAR_KD{640.0};
  /// @brief kp value for the rotational PID controller used in the path follower
  static constexpr double PID_PATH_FOLLOWER_ROTATIONAL_KP{640.0};
  /// @brief ki value for the rotational PID controller used in the path follower
  static constexpr double PID_PATH_FOLLOWER_ROTATIONAL_KI{0.0};
  /// @brief kd value for the rotational PID controller used in the path follower
  static constexpr double PID_PATH_FOLLOWER_ROTATIONAL_KD{10000.0};
  /// @brief path follower follow distance
  static constexpr double PID_PATH_FOLLOWER_FOLLOW_DISTANCE{12.0};
  /// @brief path follower target tolerance
  static constexpr double PID_PATH_FOLLOWER_TARGET_TOLERANCE{3.0};
  /// @brief path follower target velocity
  static constexpr double PID_PATH_FOLLOWER_TARGET_VELOCITY{1.0};

  // -----PORT NUMBERS-----

  // use for undefined ports
  static constexpr int8_t UNDEFINED_PORT{};

  // DRIVE MOTORS

  /// @brief first left drive motor
  static constexpr int8_t DRIVE_LEFT_MOTOR_1{-1};
  /// @brief second left drive motor
  static constexpr int8_t DRIVE_LEFT_MOTOR_2{2};
  /// @brief third left drive motor
  static constexpr int8_t DRIVE_LEFT_MOTOR_3{-3};
  /// @brief fourth left drive motor
  static constexpr int8_t DRIVE_LEFT_MOTOR_4{-4};
  /// @brief first right drive motor
  static constexpr int8_t DRIVE_RIGHT_MOTOR_1{11};
  /// @brief second right drive motor
  static constexpr int8_t DRIVE_RIGHT_MOTOR_2{-12};
  /// @brief third right drive motor
  static constexpr int8_t DRIVE_RIGHT_MOTOR_3{13};
  /// @brief fourth right drive motor
  static constexpr int8_t DRIVE_RIGHT_MOTOR_4{14};

  // ARM PORTS

  /// @brief left side arm rotational motor
  static constexpr int8_t ARM_LEFT_ROTATION_MOTOR{2};

  /// @brief right side arm rotational motor
  static constexpr int8_t ARM_RIGHT_ROTATION_MOTOR{-1};

  /// @brief arm linear motor
  static constexpr int8_t ARM_LINEAR_MOTOR{3};

  /// @brief arm color sensor
  static constexpr int8_t ARM_COLOR_SENSOR{UNDEFINED_PORT};

  /// @brief arm potentiometer
  static constexpr int8_t ARM_POTENTIOMETER{UNDEFINED_PORT};

  // CLAMP PORTS

  /// @brief clamp piston controller
  static constexpr int8_t CLAMP_PISTON_1{UNDEFINED_PORT};

  // ELEVATOR PORTS

  /// @brief first elevator motor
  static constexpr int8_t ELEVATOR_MOTOR_1{UNDEFINED_PORT};
  /// @brief elevator rotational sensor
  static constexpr int8_t ELEVATOR_ROTATIONAL_SENSOR{UNDEFINED_PORT};

  // INTAKE PORTS

  /// @brief left intake piston
  static constexpr int8_t INTAKE_LEFT_PISTON{UNDEFINED_PORT};
  /// @brief right intake piston
  static constexpr int8_t INTAKE_RIGHT_PISTON{UNDEFINED_PORT};
  /// @brief intake motor
  static constexpr int8_t INTAKE_MOTOR{UNDEFINED_PORT};

  // ODOMETRY PORTS

  /// @brief left tracking wheel
  static constexpr int8_t ODOMETRY_LINEAR_TRACKING_WHEEL{UNDEFINED_PORT};
  /// @brief right tracking wheel
  static constexpr int8_t ODOMETRY_STRAFE_TRACKING_WHEEL{UNDEFINED_PORT};
  /// @brief inertial sensor
  static constexpr int8_t ODOMETRY_INERTIAL_SENSOR{UNDEFINED_PORT};
  /// @brief distance sensor
  static constexpr int8_t ODOMETRY_DISTANCE_SENSOR{UNDEFINED_PORT};

  // -----MISC VALUES-----

  // drive

  /// @brief drive gearset
  static constexpr pros::MotorGearset DRIVE_GEARSET{pros::E_MOTOR_GEAR_BLUE};
  /// @brief drive ratio of motor voltage to velocity
  static constexpr double DRIVE_VELOCITY_TO_VOLTAGE{1.0};
  /// @brief radius of the robot
  static constexpr double ROBOT_RADIUS{7.25};
  /// @brief radius of the drive wheels
  static constexpr double DRIVE_WHEEL_RADIUS{1.25};

  // arm

  /// @brief arm rotational pid controller kp value
  static constexpr double PID_ARM_ROTATIONAL_KP{24.0};
  /// @brief arm rotational pid controller ki value
  static constexpr double PID_ARM_ROTATIONAL_KI{0.0};
  /// @brief arm rotational pid controller kd value
  static constexpr double PID_ARM_ROTATIONAL_KD{256.0};
  /// @brief arm linear pid controller kp value
  static constexpr double PID_ARM_LINEAR_KP{24.0};
  /// @brief arm linear pid controller ki value
  static constexpr double PID_ARM_LINEAR_KI{0.0};
  /// @brief arm linear pid controller kd value
  static constexpr double PID_ARM_LINEAR_KD{256.0};
  /// @brief arm rotational neutral position
  static constexpr double ARM_ROTATIONAL_NEUTRAL_POSITION{0.0};
  /// @brief arm rotational load position
  static constexpr double ARM_ROTATIONAL_LOAD_POSITION{0.0};
  /// @brief arm rotational ready position
  static constexpr double ARM_ROTATIONAL_READY_POSITION{0.3375 * 2 * M_PI};
  /// @brief arm rotational score position
  static constexpr double ARM_ROTATIONAL_SCORE_POSITION{};
  /// @brief arm rotational position tolerance
  static constexpr double ARM_ROTATIONAL_TOLERANCE{0.5};
  /// @brief arm linear neutral position
  static constexpr double ARM_LINEAR_NEUTRAL_POSITION{0.6 * 2 * M_PI};
  /// @brief arm linear load position
  static constexpr double ARM_LINEAR_LOAD_POSITION{0.0};
  /// @brief arm linear ready position
  static constexpr double ARM_LINEAR_READY_POSITION{1.85 * 2 * M_PI};
  /// @brief arm linear score position
  static constexpr double ARM_LINEAR_SCORE_POSITION{};
  /// @brief arm linear position tolerance
  static constexpr double ARM_LINEAR_TOLERANCE{0.5};
  /// @brief arm ring proximity
  static constexpr uint32_t ARM_RING_PROXIMITY{0};

  // elevator

  /// @brief elevator pid controller kp value
  static constexpr double PID_ELEVATOR_KP{24.0};
  /// @brief elevator pid controller ki value
  static constexpr double PID_ELEVATOR_KI{0.0};
  /// @brief elevator pid controller kd value
  static constexpr double PID_ELEVATOR_KD{256.0};
  /// @brief elevator radians to inches travelled
  static constexpr double ELEVATOR_RADIANS_TO_INCHES{};

  // odometry

  /// @brief radius of the tracking wheels
  static constexpr double TRACKING_WHEEL_RADIUS{1.0};
  /// @brief left offset of the left tracking wheel
  static constexpr double LINEAR_TRACKING_WHEEL_OFFSET{4.8};
  /// @brief left offset of the right tracking wheel
  static constexpr double STRAFE_TRACKING_WHEEL_OFFSET{8.69};
  /// @brief position resetter x offset
  static constexpr double RESETTER_LOCAL_X_OFFSET{7.5};
  /// @brief position resetter y offset
  static constexpr double RESETTER_LOCAL_Y_OFFSET{0.0};
  /// @brief position resetter angular offset
  static constexpr double RESETTER_LOCAL_THETA_OFFSET{0.0};

 public:
  /// @brief Gets the name of the config
  /// @return The name of the config as a string
  std::string getName() override;

  /// @brief Builds a control system using the config values
  /// @return a new control system
  std::shared_ptr<control::ControlSystem> buildControlSystem() override;

  /// @brief Builds a controller object using the config values
  /// @return a new controller object
  std::shared_ptr<io::IController> buildController() override;

  /// @brief Builds a robot object using the config values
  /// @return a new robot object
  std::shared_ptr<robot::Robot> buildRobot() override;
};
}  // namespace config
}  // namespace driftless
#endif