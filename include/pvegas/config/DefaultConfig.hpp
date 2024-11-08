#ifndef __DEFAULT_CONFIG_HPP__
#define __DEFAULT_CONFIG_HPP__

#include <memory>

// config interface
#include "pvegas/config/IConfig.hpp"

// pros object includes
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/imu.hpp"
#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include "pros/rotation.hpp"

// control system includes
#include "pvegas/control/ControlSystem.hpp"
#include "pvegas/control/motion/MotionControl.hpp"
#include "pvegas/control/motion/PIDDriveStraightBuilder.hpp"
#include "pvegas/control/motion/PIDGoToPointBuilder.hpp"
#include "pvegas/control/motion/PIDTurnBuilder.hpp"
#include "pvegas/control/path/PIDPathFollowerBuilder.hpp"
#include "pvegas/control/path/PathFollowerControl.hpp"

// hardware interface includes
#include "pvegas/hal/TrackingWheel.hpp"
#include "pvegas/io/IColorSensor.hpp"
#include "pvegas/io/IController.hpp"
#include "pvegas/io/IDistanceSensor.hpp"
#include "pvegas/io/IDistanceTracker.hpp"
#include "pvegas/io/IInertialSensor.hpp"
#include "pvegas/io/IMotor.hpp"
#include "pvegas/io/IPiston.hpp"
#include "pvegas/io/IPotentiometer.hpp"
#include "pvegas/io/IRotationSensor.hpp"

// pros adapter includes
#include "pvegas/pros_adapters/ProsADIPotentiometer.hpp"
#include "pvegas/pros_adapters/ProsClock.hpp"
#include "pvegas/pros_adapters/ProsColorSensor.hpp"
#include "pvegas/pros_adapters/ProsController.hpp"
#include "pvegas/pros_adapters/ProsDelayer.hpp"
#include "pvegas/pros_adapters/ProsDistanceSensor.hpp"
#include "pvegas/pros_adapters/ProsInertialSensor.hpp"
#include "pvegas/pros_adapters/ProsMutex.hpp"
#include "pvegas/pros_adapters/ProsPiston.hpp"
#include "pvegas/pros_adapters/ProsRotationSensor.hpp"
#include "pvegas/pros_adapters/ProsTask.hpp"
#include "pvegas/pros_adapters/ProsV5Motor.hpp"

// robot include
#include "pvegas/robot/Robot.hpp"

// arm subsystem includes
#include "pvegas/robot/subsystems/arm/ArmSubsystem.hpp"
#include "pvegas/robot/subsystems/arm/ColorRingSensorBuilder.hpp"
#include "pvegas/robot/subsystems/arm/PIDArmMotionBuilder.hpp"

// clamp subsystem includes
#include "pvegas/robot/subsystems/clamp/ClampSubsystem.hpp"
#include "pvegas/robot/subsystems/clamp/PistonClampBuilder.hpp"

// drivetrain subsystem includes
#include "pvegas/robot/subsystems/drivetrain/DirectDriveBuilder.hpp"
#include "pvegas/robot/subsystems/drivetrain/DrivetrainSubsystem.hpp"
#include "pvegas/robot/subsystems/drivetrain/IDriveTrain.hpp"

// elevator subsystem includes
#include "pvegas/robot/subsystems/elevator/ElevatorSubsystem.hpp"
#include "pvegas/robot/subsystems/elevator/PIDElevatorBuilder.hpp"

// intake subsystem includes
#include "pvegas/robot/subsystems/intake/DirectIntakeBuilder.hpp"
#include "pvegas/robot/subsystems/intake/IntakeSubsystem.hpp"
#include "pvegas/robot/subsystems/intake/PistonHeightControlBuilder.hpp"

// odometry subsystem includes
#include "pvegas/robot/subsystems/odometry/DistancePositionResetterBuilder.hpp"
#include "pvegas/robot/subsystems/odometry/InertialPositionTrackerBuilder.hpp"
#include "pvegas/robot/subsystems/odometry/OdometrySubsystem.hpp"

// rtos includes
#include "pvegas/rtos/IClock.hpp"
#include "pvegas/rtos/IDelayer.hpp"
#include "pvegas/rtos/IMutex.hpp"
#include "pvegas/rtos/ITask.hpp"

namespace driftless {
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

  // use for undefined ports
  static constexpr int8_t UNDEFINED_PORT{};

  // DRIVE MOTORS

  // first left drive motor
  static constexpr int8_t DRIVE_LEFT_MOTOR_1{-UNDEFINED_PORT};
  // second left drive motor
  static constexpr int8_t DRIVE_LEFT_MOTOR_2{UNDEFINED_PORT};
  // third left drive motor
  static constexpr int8_t DRIVE_LEFT_MOTOR_3{-UNDEFINED_PORT};
  // fourth left drive motor
  static constexpr int8_t DRIVE_LEFT_MOTOR_4{UNDEFINED_PORT};
  // first right drive motor
  static constexpr int8_t DRIVE_RIGHT_MOTOR_1{UNDEFINED_PORT};
  // second right drive motor
  static constexpr int8_t DRIVE_RIGHT_MOTOR_2{-UNDEFINED_PORT};
  // third right drive motor
  static constexpr int8_t DRIVE_RIGHT_MOTOR_3{UNDEFINED_PORT};
  // fourth right drive motor
  static constexpr int8_t DRIVE_RIGHT_MOTOR_4{-UNDEFINED_PORT};

  // ARM PORTS

  // left side arm rotational motor
  static constexpr int8_t ARM_LEFT_ROTATION_MOTOR{2};

  // right side arm rotational motor
  static constexpr int8_t ARM_RIGHT_ROTATION_MOTOR{-1};

  // arm linear motor
  static constexpr int8_t ARM_LINEAR_MOTOR{3};

  // arm color sensor
  static constexpr int8_t ARM_COLOR_SENSOR{UNDEFINED_PORT};

  // arm potentiometer
  static constexpr int8_t ARM_POTENTIOMETER{UNDEFINED_PORT};

  // CLAMP PORTS

  // clamp piston controller
  static constexpr int8_t CLAMP_PISTON_1{UNDEFINED_PORT};

  // ELEVATOR PORTS

  // first elevator motor
  static constexpr int8_t ELEVATOR_MOTOR_1{UNDEFINED_PORT};
  // elevator rotational sensor
  static constexpr int8_t ELEVATOR_ROTATIONAL_SENSOR{UNDEFINED_PORT};

  // INTAKE PORTS
  // left intake piston
  static constexpr int8_t INTAKE_LEFT_PISTON{UNDEFINED_PORT};
  // right intake piston
  static constexpr int8_t INTAKE_RIGHT_PISTON{UNDEFINED_PORT};
  // intake motor
  static constexpr int8_t INTAKE_MOTOR{UNDEFINED_PORT};

  // ODOMETRY PORTS

  // left tracking wheel
  static constexpr int8_t ODOMETRY_LINEAR_TRACKING_WHEEL{UNDEFINED_PORT};
  // right tracking wheel
  static constexpr int8_t ODOMETRY_STRAFE_TRACKING_WHEEL{UNDEFINED_PORT};
  // inertial sensor
  static constexpr int8_t ODOMETRY_INERTIAL_SENSOR{UNDEFINED_PORT};
  // distance sensor
  static constexpr int8_t ODOMETRY_DISTANCE_SENSOR{UNDEFINED_PORT};

  // -----MISC VALUES-----

  // drive

  // drive gearset
  static constexpr pros::MotorGearset DRIVE_GEARSET{pros::E_MOTOR_GEAR_BLUE};
  // drive ratio of motor voltage to velocity
  static constexpr double DRIVE_VELOCITY_TO_VOLTAGE{1.0};
  // radius of the robot
  static constexpr double ROBOT_RADIUS{7.25};
  // radius of the drive wheels
  static constexpr double DRIVE_WHEEL_RADIUS{1.25};

  // arm

  // arm rotational pid controller kp value
  static constexpr double PID_ARM_ROTATIONAL_KP{24.0};
  // arm rotational pid controller ki value
  static constexpr double PID_ARM_ROTATIONAL_KI{0.0};
  // arm rotational pid controller kd value
  static constexpr double PID_ARM_ROTATIONAL_KD{256.0};
  // arm linear pid controller kp value
  static constexpr double PID_ARM_LINEAR_KP{24.0};
  // arm linear pid controller ki value
  static constexpr double PID_ARM_LINEAR_KI{0.0};
  // arm linear pid controller kd value
  static constexpr double PID_ARM_LINEAR_KD{256.0};
  // arm rotational neutral position
  static constexpr double ARM_ROTATIONAL_NEUTRAL_POSITION{0.0};
  // arm rotational load position
  static constexpr double ARM_ROTATIONAL_LOAD_POSITION{0.0};
  // arm rotational ready position
  static constexpr double ARM_ROTATIONAL_READY_POSITION{0.3375 * 2 * M_PI};
  // arm rotational score position
  static constexpr double ARM_ROTATIONAL_SCORE_POSITION{};
  // arm rotational position tolerance
  static constexpr double ARM_ROTATIONAL_TOLERANCE{0.5};
  // arm linear neutral position
  static constexpr double ARM_LINEAR_NEUTRAL_POSITION{0.6 * 2 * M_PI};
  // arm linear load position
  static constexpr double ARM_LINEAR_LOAD_POSITION{0.0};
  // arm linear ready position
  static constexpr double ARM_LINEAR_READY_POSITION{1.85 * 2 * M_PI};
  // arm linear score position
  static constexpr double ARM_LINEAR_SCORE_POSITION{};
  // arm linear position tolerance
  static constexpr double ARM_LINEAR_TOLERANCE{0.5};
  // arm ring proximity
  static constexpr uint32_t ARM_RING_PROXIMITY{0};

  // elevator

  // elevator pid controller kp value
  static constexpr double PID_ELEVATOR_KP{24.0};
  // elevator pid controller ki value
  static constexpr double PID_ELEVATOR_KI{0.0};
  // elevator pid controller kd value
  static constexpr double PID_ELEVATOR_KD{256.0};
  // elevator radians to inches travelled
  static constexpr double ELEVATOR_RADIANS_TO_INCHES{};

  // odometry

  // radius of the tracking wheels
  static constexpr double TRACKING_WHEEL_RADIUS{1.0};
  // left offset of the left tracking wheel
  static constexpr double LINEAR_TRACKING_WHEEL_OFFSET{4.8};
  // left offset of the right tracking wheel
  static constexpr double STRAFE_TRACKING_WHEEL_OFFSET{8.69};
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