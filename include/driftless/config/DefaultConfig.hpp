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
#include "driftless/hal/SparkfunOTOS.hpp"
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
#include "driftless/pros_adapters/ProsSerialDevice.hpp"
#include "driftless/pros_adapters/ProsTask.hpp"
#include "driftless/pros_adapters/ProsV5Motor.hpp"

// robot include
#include "driftless/robot/Robot.hpp"

// arm subsystem includes
#include "driftless/robot/subsystems/arm/ArmSubsystem.hpp"
#include "driftless/robot/subsystems/arm/PIDArmMotionBuilder.hpp"

// clamp subsystem includes
#include "driftless/robot/subsystems/clamp/ClampSubsystem.hpp"
#include "driftless/robot/subsystems/clamp/PistonClampBuilder.hpp"

// climb subsystem includes
#include "driftless/robot/subsystems/climb/ClimbSubsystem.hpp"
#include "driftless/robot/subsystems/climb/PneumaticClimbBuilder.hpp"

// drivetrain subsystem includes
#include "driftless/robot/subsystems/drivetrain/DirectDriveBuilder.hpp"
#include "driftless/robot/subsystems/drivetrain/DrivetrainSubsystem.hpp"
#include "driftless/robot/subsystems/drivetrain/IDriveTrain.hpp"

// elevator subsystem includes
#include "driftless/robot/subsystems/elevator/ElevatorSubsystem.hpp"
#include "driftless/robot/subsystems/elevator/PIDElevatorBuilder.hpp"
#include "driftless/robot/subsystems/elevator/PistonRingRejectionBuilder.hpp"

// intake subsystem includes
#include "driftless/robot/subsystems/intake/DirectIntakeBuilder.hpp"
#include "driftless/robot/subsystems/intake/IntakeSubsystem.hpp"
#include "driftless/robot/subsystems/intake/PistonHeightControlBuilder.hpp"

// odometry subsystem includes
#include "driftless/robot/subsystems/odometry/DistancePositionResetterBuilder.hpp"
#include "driftless/robot/subsystems/odometry/InertialPositionTrackerBuilder.hpp"
#include "driftless/robot/subsystems/odometry/OdometrySubsystem.hpp"
#include "driftless/robot/subsystems/odometry/SparkFunPositionTrackerBuilder.hpp"

// ring sort subsystem includes
#include "driftless/robot/subsystems/ring_sort/ColorRingSortBuilder.hpp"
#include "driftless/robot/subsystems/ring_sort/RingSortSubsystem.hpp"

// rtos includes
#include "driftless/rtos/IClock.hpp"
#include "driftless/rtos/IDelayer.hpp"
#include "driftless/rtos/IMutex.hpp"
#include "driftless/rtos/ITask.hpp"

// ring rejection process includes
#include "driftless/processes/auto_ring_rejection/AutoRingRejectionProcess.hpp"
#include "driftless/processes/auto_ring_rejection/ElevatorAutoRingRejectorBuilder.hpp"

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for robot configurations
/// @author Matthew Backman
namespace config {

/// @brief the default robot configuration
/// @author Matthew Backman
class DefaultConfig : public IConfig {
 private:
  /// @brief the name of the config
  static constexpr char CONFIG_NAME[]{"DEFAULT_CONFIG"};

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
  static constexpr double PID_DRIVE_STRAIGHT_ROTATIONAL_KP{100.0};
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
  static constexpr double PID_GO_TO_POINT_LINEAR_KP{6.5};
  /// @brief ki value for the go to point linear pid controller
  static constexpr double PID_GO_TO_POINT_LINEAR_KI{0};
  /// @brief kd value of the go to point linear pid controller
  static constexpr double PID_GO_TO_POINT_LINEAR_KD{280};
  /// @brief kp value for the go to point rotational pid controller
  static constexpr double PID_GO_TO_POINT_ROTATIONAL_KP{2.9};
  /// @brief ki value for the go to point rotational pid controller
  static constexpr double PID_GO_TO_POINT_ROTATIONAL_KI{0};
  /// @brief kd value for the go to point rotational pid controller
  static constexpr double PID_GO_TO_POINT_ROTATIONAL_KD{230};
  /// @brief the target tolerance of the go to point algorithm
  static constexpr double PID_GO_TO_POINT_TARGET_TOLERANCE{0.5};
  /// @brief the target velocity of the go to point algorithm
  static constexpr double PID_GO_TO_POINT_TARGET_VELOCITY{5.0};

  // turn

  /// @brief kp value for the turn rotational pid controller
  static constexpr double PID_TURN_ROTATIONAL_KP{32.0};
  /// @brief ki value for the turn rotational pid controller
  static constexpr double PID_TURN_ROTATIONAL_KI{0.001};
  /// @brief kd value for the turn rotational pid controller
  static constexpr double PID_TURN_ROTATIONAL_KD{1250.0};
  /// @brief the target tolerance of the turn algorithm
  static constexpr double PID_TURN_TARGET_TOLERANCE{M_PI / 50.0};
  /// @brief the target velocity of the turn algorithm
  static constexpr double PID_TURN_TARGET_VELOCITY{M_PI / 200.0};

  // path follower control

  /// @brief kp value for the linear PID controller used in the path follower
  static constexpr double PID_PATH_FOLLOWER_LINEAR_KP{12.0};
  /// @brief ki value for the linear PID controller used in the path follower
  static constexpr double PID_PATH_FOLLOWER_LINEAR_KI{0.0};
  /// @brief kd value for the linear PID controller used in the path follower
  static constexpr double PID_PATH_FOLLOWER_LINEAR_KD{640.0};
  /// @brief kp value for the rotational PID controller used in the path
  /// follower
  static constexpr double PID_PATH_FOLLOWER_ROTATIONAL_KP{100.0};
  /// @brief ki value for the rotational PID controller used in the path
  /// follower
  static constexpr double PID_PATH_FOLLOWER_ROTATIONAL_KI{0.0};
  /// @brief kd value for the rotational PID controller used in the path
  /// follower
  static constexpr double PID_PATH_FOLLOWER_ROTATIONAL_KD{1000.0};
  /// @brief path follower follow distance
  static constexpr double PID_PATH_FOLLOWER_FOLLOW_DISTANCE{12.0};
  /// @brief path follower target tolerance
  static constexpr double PID_PATH_FOLLOWER_TARGET_TOLERANCE{3.0};
  /// @brief path follower target velocity
  static constexpr double PID_PATH_FOLLOWER_TARGET_VELOCITY{12.0};

  // -----PORT NUMBERS-----

  // use for undefined ports
  static constexpr int8_t UNDEFINED_PORT{};

  // DRIVE MOTORS

  /// @brief first left drive motor
  static constexpr int8_t DRIVE_LEFT_MOTOR_1{-10};
  /// @brief second left drive motor
  static constexpr int8_t DRIVE_LEFT_MOTOR_2{9};
  /// @brief third left drive motor
  static constexpr int8_t DRIVE_LEFT_MOTOR_3{-8};
  /// @brief fourth left drive motor
  static constexpr int8_t DRIVE_LEFT_MOTOR_4{-7};

  static constexpr int8_t DRIVE_LEFT_MOTOR_5{6};
  /// @brief first right drive motor
  static constexpr int8_t DRIVE_RIGHT_MOTOR_1{1};
  /// @brief second right drive motor
  static constexpr int8_t DRIVE_RIGHT_MOTOR_2{-2};
  /// @brief third right drive motor
  static constexpr int8_t DRIVE_RIGHT_MOTOR_3{3};
  /// @brief fourth right drive motor
  static constexpr int8_t DRIVE_RIGHT_MOTOR_4{4};

  static constexpr int8_t DRIVE_RIGHT_MOTOR_5{-5};

  // ARM PORTS

  /// @brief left side arm rotational motor
  static constexpr int8_t ARM_LEFT_ROTATION_MOTOR{15};

  /// @brief arm linear motor
  static constexpr int8_t ARM_LINEAR_MOTOR{-18};

  // CLAMP PORTS

  /// @brief clamp piston controller
  static constexpr int8_t CLAMP_PISTON_1{3};

  static constexpr int8_t CLAMP_DISTANCE_SENSOR{14};

  // CLIMB PORTS

  static constexpr int8_t CLIMB_STILT_PISTON{6};

  static constexpr int8_t CLIMB_CLIMBER_PISTON{5};

  static constexpr int8_t CLIMB_PASSIVE_PISTON{2};

  // ELEVATOR PORTS

  /// @brief first elevator motor
  static constexpr int8_t ELEVATOR_MOTOR_1{-16};
  /// @brief elevator rotational sensor
  static constexpr int8_t ELEVATOR_ROTATIONAL_SENSOR{UNDEFINED_PORT};

  static constexpr int8_t ELEVATOR_REJECTION_LEFT_PISTON{4};

  static constexpr int8_t ELEVATOR_REJECTION_RIGHT_PISTON{8};

  // INTAKE PORTS

  /// @brief intake piston
  static constexpr int8_t INTAKE_STAGE1_PISTON{1};

  static constexpr int8_t INTAKE_STAGE2_PISTON{7};
  /// @brief intake motor
  static constexpr int8_t INTAKE_MOTOR{17};

  // ODOMETRY PORTS

  /// @brief left tracking wheel
  static constexpr int8_t ODOMETRY_LINEAR_TRACKING_WHEEL{UNDEFINED_PORT};
  /// @brief right tracking wheel
  static constexpr int8_t ODOMETRY_STRAFE_TRACKING_WHEEL{UNDEFINED_PORT};
  /// @brief inertial sensor
  static constexpr int8_t ODOMETRY_INERTIAL_SENSOR{UNDEFINED_PORT};
  /// @brief distance sensor
  static constexpr int8_t ODOMETRY_DISTANCE_SENSOR{13};

  static constexpr int8_t ODOMETRY_ARDUINO{13};

  // RING SORT PORTS

  static constexpr int8_t RING_SORT_COLOR_SENSOR{19};

  // -----MISC VALUES-----

  // drive

  /// @brief drive gearset
  static constexpr pros::MotorGearset DRIVE_GEARSET{pros::E_MOTOR_GEAR_BLUE};
  /// @brief drive ratio of motor voltage to velocity
  static constexpr double DRIVE_VELOCITY_TO_VOLTAGE{12.0 / 83.0};
  /// @brief radius of the robot
  static constexpr double ROBOT_RADIUS{5.5};
  /// @brief radius of the drive wheels
  static constexpr double DRIVE_WHEEL_RADIUS{1.25};

  // arm

  /// @brief whether the arm potentiometer is reversed or not
  static constexpr bool ARM_POTENTIOMETER_REVERSED{true};
  /// @brief the gearset on the arm linear motor
  static constexpr pros::MotorGearset ARM_LINEAR_GEARSET{
      pros::E_MOTOR_GEAR_200};
  /// @brief the gearset on the arm rotation motors
  static constexpr pros::MotorGearset ARM_ROTATIONAL_GEARSET{
      pros::E_MOTOR_GEAR_200};
  /// @brief arm rotational pid controller kp value
  static constexpr double PID_ARM_ROTATIONAL_KP{18.0};
  /// @brief arm rotational pid controller ki value
  static constexpr double PID_ARM_ROTATIONAL_KI{0.0};
  /// @brief arm rotational pid controller kd value
  static constexpr double PID_ARM_ROTATIONAL_KD{500.0};
  /// @brief arm linear pid controller kp value
  static constexpr double PID_ARM_LINEAR_KP{24.0};
  /// @brief arm linear pid controller ki value
  static constexpr double PID_ARM_LINEAR_KI{0.0};
  /// @brief arm linear pid controller kd value
  static constexpr double PID_ARM_LINEAR_KD{200.0};
  /// @brief arm rotational neutral position
  static constexpr double ARM_ROTATIONAL_NEUTRAL_POSITION{0.15 * 2.0 * M_PI};
  /// @brief arm rotational load position
  static constexpr double ARM_ROTATIONAL_LOAD_POSITION{0.15 * 2.0 * M_PI};
  /// @brief arm rotational ready position
  static constexpr double ARM_ROTATIONAL_READY_POSITION{1.2 * 2.0 * M_PI};
  /// @brief arm rotational score position
  static constexpr double ARM_ROTATIONAL_SCORE_POSITION{1.45 * 2.0 * M_PI};
  /// @brief arm rotational rush position
  static constexpr double ARM_ROTATIONAL_RUSH_POSITION{2.25 * 2.0 * M_PI};

  static constexpr double ARM_ROTATIONAL_ALLIANCE_STAKE_POSITION{1.71 * 2.0 *
                                                                 M_PI};

  static constexpr double ARM_ROTATIONAL_CLIMB_POSITION{1.38 * 2.0 * M_PI};

  /// @brief The intermediate position on the rotation towards the arm ready
  /// position
  static constexpr double ARM_ROTATIONAL_READY_INTERMEDIATE_POSITION{0.75 * 2.0 *
                                                                     M_PI};
  /// @brief The intermediate position on the rotation towards the arm score
  /// position
  static constexpr double ARM_ROTATIONAL_SCORE_INTERMEDIATE_POSITION{1.4 * 2.0 *
                                                                     M_PI};
  /// @brief The intermediate position on the rotation towards the arm rush
  /// position
  static constexpr double ARM_ROTATIONAL_RUSH_INTERMEDIATE_POSITION{0.75 * 2.0 *
                                                                    M_PI};

  static constexpr double ARM_ROTATIONAL_ALLIANCE_STAKE_INTERMEDIATE_POSITION{
      0.5 * 2.0 * M_PI};
  /// @brief arm rotational position tolerance
  static constexpr double ARM_ROTATIONAL_TOLERANCE{0.1};
  /// @brief arm linear neutral position
  static constexpr double ARM_LINEAR_NEUTRAL_POSITION{0.5 * 2.0 * M_PI};
  /// @brief arm linear load position
  static constexpr double ARM_LINEAR_LOAD_POSITION{0.08 * 2.0 * M_PI};
  /// @brief arm linear ready position
  static constexpr double ARM_LINEAR_READY_POSITION{0.965 * 2 * M_PI};
  /// @brief arm linear score position
  static constexpr double ARM_LINEAR_SCORE_POSITION{0.485 * 2 * M_PI};
  /// @brief Arm linear rush position
  static constexpr double ARM_LINEAR_RUSH_POSITION{0.530 * 2 * M_PI};

  static constexpr double ARM_LINEAR_ALLIANCE_STAKE_POSITION{0.175 * 2 * M_PI};

  static constexpr double ARM_LINEAR_CLIMB_READY_POSITION{0.4 * 2 * M_PI};

  static constexpr double ARM_LINEAR_CLIMB_POSITION{0.16 * 2 * M_PI};

  /// @brief arm linear position tolerance
  static constexpr double ARM_LINEAR_TOLERANCE{0.1};

  // clamp

  static constexpr double CLAMP_GOAL_DISTANCE{80.0 / 25.4};

  // elevator

  /// @brief elevator pid controller kp value
  static constexpr double PID_ELEVATOR_KP{24.0};
  /// @brief elevator pid controller ki value
  static constexpr double PID_ELEVATOR_KI{0.0};
  /// @brief elevator pid controller kd value
  static constexpr double PID_ELEVATOR_KD{256.0};
  /// @brief elevator radians to inches travelled
  static constexpr double ELEVATOR_RADIANS_TO_INCHES{2.35 / (2.0 * M_PI)};
  /// @brief The max distance a ring can be from the color sensor
  static constexpr uint8_t ELEVATOR_MAX_RING_DISTANCE{50};

  // odometry

  /// @brief radius of the tracking wheels
  static constexpr double TRACKING_WHEEL_RADIUS{1.125};
  /// @brief left offset of the linear tracking wheel
  static constexpr double LINEAR_TRACKING_WHEEL_OFFSET{0.0};
  /// @brief forwards offset of the strafe tracking wheel
  static constexpr double STRAFE_TRACKING_WHEEL_OFFSET{-2.25};
  /// @brief position resetter x offset
  static constexpr double RESETTER_LOCAL_X_OFFSET{3.33159158};
  /// @brief position resetter y offset
  static constexpr double RESETTER_LOCAL_Y_OFFSET{0.0};
  /// @brief position resetter angular offset
  static constexpr double RESETTER_LOCAL_THETA_OFFSET{0.0};

  static constexpr double ODOMETRY_SENSOR_LOCAL_X_OFFSET{0.0};

  static constexpr double ODOMETRY_SENSOR_LOCAL_Y_OFFSET{-1.125};

  static constexpr double ODOMETRY_SENSOR_LOCAL_THETA_OFFSET{M_PI / 2.0};

  static constexpr int ODOMETRY_BAUD_RATE{74880};

  // ring sensor

  /// @brief The distance from the color sensor to the end of the elevator
  static constexpr double RING_SORT_COLOR_SENSOR_TO_END{1.8};
  /// @brief The minimum proximity value to be considered a ring
  static constexpr uint8_t RING_SORT_MIN_RING_PROXIMITY{50};

 public:
  /// @brief Gets the name of the config
  /// @return __std::string__ The name of the config as a string
  std::string getName() override;

  /// @brief Builds a control system using the config values
  /// @return __std::shared_ptr<control::ControlSystem>__ The new control system
  std::shared_ptr<control::ControlSystem> buildControlSystem() override;

  /// @brief Builds a controller object using the config values
  /// @return __std::shared_ptr<io::IController>__ The new controller object
  std::shared_ptr<io::IController> buildController() override;

  /// @brief Builds a robot object using the config values
  /// @return __std::shared_ptr<robot::Robot>__ The new robot object
  std::shared_ptr<robot::Robot> buildRobot() override;

  /// @brief Builds a process system using config values
  /// @return __shared_ptr<ProcessSystem>__ The new process system
  std::shared_ptr<processes::ProcessSystem> buildProcessSystem() override;
};
}  // namespace config
}  // namespace driftless
#endif