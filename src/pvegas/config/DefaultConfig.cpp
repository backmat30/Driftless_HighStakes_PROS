#include "pvegas/config/DefaultConfig.hpp"

namespace pvegas {
namespace config {
std::string DefaultConfig::getName() { return CONFIG_NAME; }

std::shared_ptr<control::ControlSystem> DefaultConfig::buildControlSystem() {
  // creates a new ControlSystem object through the copy constructor
  std::shared_ptr<control::ControlSystem> control_system{
      std::make_shared<control::ControlSystem>()};

  // TODO - add controls as needed

  return control_system;
}

std::shared_ptr<io::IController> DefaultConfig::buildController() {
  // creates a default pros controller
  std::unique_ptr<pros::Controller> pros_controller{
      std::make_unique<pros::Controller>(pros::E_CONTROLLER_MASTER)};
  // adapts the pros controller to work as an IController object
  std::shared_ptr<io::IController> adapted_controller{
      std::make_shared<pros_adapters::ProsController>(pros_controller)};
  // send back a new adapted controller
  return adapted_controller;
}

std::shared_ptr<robot::Robot> DefaultConfig::buildRobot() {
  // create the robot
  std::shared_ptr<robot::Robot> robot{std::make_shared<robot::Robot>()};

  // DRIVETRAIN
  // creates the factory used to build the drivetrain
  robot::subsystems::drivetrain::DirectDriveBuilder drive_factory{};
  // pros objects
  // left motors
  std::unique_ptr<pros::Motor> left_temp_motor_1{
      std::make_unique<pros::Motor>(DRIVE_LEFT_MOTOR_1, DRIVE_GEARSET)};
  std::unique_ptr<pros::Motor> left_temp_motor_2{
      std::make_unique<pros::Motor>(DRIVE_LEFT_MOTOR_2, DRIVE_GEARSET)};
  std::unique_ptr<pros::Motor> left_temp_motor_3{
      std::make_unique<pros::Motor>(DRIVE_LEFT_MOTOR_3, DRIVE_GEARSET)};
  std::unique_ptr<pros::Motor> left_temp_motor_4{
      std::make_unique<pros::Motor>(DRIVE_LEFT_MOTOR_4, DRIVE_GEARSET)};
  // right motors
  std::unique_ptr<pros::Motor> right_temp_motor_1{
      std::make_unique<pros::Motor>(DRIVE_RIGHT_MOTOR_1, DRIVE_GEARSET)};
  std::unique_ptr<pros::Motor> right_temp_motor_2{
      std::make_unique<pros::Motor>(DRIVE_RIGHT_MOTOR_2, DRIVE_GEARSET)};
  std::unique_ptr<pros::Motor> right_temp_motor_3{
      std::make_unique<pros::Motor>(DRIVE_RIGHT_MOTOR_3, DRIVE_GEARSET)};
  std::unique_ptr<pros::Motor> right_temp_motor_4{
      std::make_unique<pros::Motor>(DRIVE_RIGHT_MOTOR_4, DRIVE_GEARSET)};

  // pros adapters
  // left motors
  std::unique_ptr<io::IMotor> left_motor_1{
      std::make_unique<pros_adapters::ProsV5Motor>(left_temp_motor_1)};
  std::unique_ptr<io::IMotor> left_motor_2{
      std::make_unique<pros_adapters::ProsV5Motor>(left_temp_motor_2)};
  std::unique_ptr<io::IMotor> left_motor_3{
      std::make_unique<pros_adapters::ProsV5Motor>(left_temp_motor_3)};
  std::unique_ptr<io::IMotor> left_motor_4{
      std::make_unique<pros_adapters::ProsV5Motor>(left_temp_motor_4)};
  // right motors
  std::unique_ptr<io::IMotor> right_motor_1{
      std::make_unique<pros_adapters::ProsV5Motor>(right_temp_motor_1)};
  std::unique_ptr<io::IMotor> right_motor_2{
      std::make_unique<pros_adapters::ProsV5Motor>(right_temp_motor_2)};
  std::unique_ptr<io::IMotor> right_motor_3{
      std::make_unique<pros_adapters::ProsV5Motor>(right_temp_motor_3)};
  std::unique_ptr<io::IMotor> right_motor_4{
      std::make_unique<pros_adapters::ProsV5Motor>(right_temp_motor_4)};

  // assembling the drive train
  std::unique_ptr<robot::subsystems::drivetrain::IDrivetrain> drivetrain{
      // call the factory and add all necessary items for the drivetrain
      drive_factory.withLeftMotor(left_motor_1)
          ->withLeftMotor(left_motor_2)
          ->withLeftMotor(left_motor_3)
          ->withLeftMotor(left_motor_4)
          ->withRightMotor(right_motor_1)
          ->withRightMotor(right_motor_2)
          ->withRightMotor(right_motor_3)
          ->withRightMotor(right_motor_4)
          ->withVelocityToVoltage(DRIVE_VELOCITY_TO_VOLTAGE)
          ->withDriveRadius(ROBOT_RADIUS)
          ->withWheelRadius(DRIVE_WHEEL_RADIUS)
          ->build()};
  // create the subsystem for the drivetrain
  std::unique_ptr<robot::ASubsystem> drivetrain_subsystem{
      std::make_unique<robot::subsystems::drivetrain::DrivetrainSubsystem>(
          drivetrain)};
  // add the new subsystem to the robot
  robot->addSubsystem(drivetrain_subsystem);

  // objects for the odometry subsystem
  // pros objects
  std::unique_ptr<pros::Rotation> temp_left_rotation_sensor{
      std::make_unique<pros::Rotation>(ODOMETRY_LEFT_TRACKING_WHEEL)};
  std::unique_ptr<pros::Rotation> temp_right_rotation_sensor{
      std::make_unique<pros::Rotation>(ODOMETRY_RIGHT_TRACKING_WHEEL)};
  std::unique_ptr<pros::IMU> temp_inertial_sensor{
      std::make_unique<pros::IMU>(ODOMETRY_INERTIAL_SENSOR)};
  std::unique_ptr<pros::Distance> temp_distance_sensor{
      std::make_unique<pros::Distance>(ODOMETRY_DISTANCE_SENSOR)};

  // adapted objects
  // left tracking wheel
  std::unique_ptr<pvegas::io::IRotationSensor> left_rotation_sensor{
      std::make_unique<pvegas::pros_adapters::ProsRotationSensor>(
          temp_left_rotation_sensor)};
  std::unique_ptr<pvegas::io::IDistanceTracker> left_tracking_wheel{
      std::make_unique<pvegas::hal::TrackingWheel>(left_rotation_sensor, TRACKING_WHEEL_RADIUS)};
  // right tracking wheel
  std::unique_ptr<pvegas::io::IRotationSensor> right_rotation_sensor{
      std::make_unique<pvegas::pros_adapters::ProsRotationSensor>(
          temp_right_rotation_sensor)};
  std::unique_ptr<pvegas::io::IDistanceTracker> right_tracking_wheel{
      std::make_unique<pvegas::hal::TrackingWheel>(right_rotation_sensor, TRACKING_WHEEL_RADIUS)};
  // inertial sensor
  std::unique_ptr<pvegas::io::IInertialSensor> inertial_sensor{
      std::make_unique<pvegas::pros_adapters::ProsInertialSensor>(
          temp_inertial_sensor)};
  // distance sensor
  std::unique_ptr<pvegas::io::IDistanceSensor> distance_sensor{
      std::make_unique<pvegas::pros_adapters::ProsDistanceSensor>(
          temp_distance_sensor)};
  // odometry rtos
  std::unique_ptr<pvegas::rtos::IClock> odometry_clock{
      std::make_unique<pvegas::pros_adapters::ProsClock>()};
  std::unique_ptr<pvegas::rtos::IDelayer> odometry_delayer{
      std::make_unique<pvegas::pros_adapters::ProsDelayer>()};
  std::unique_ptr<pvegas::rtos::IMutex> odometry_mutex{
      std::make_unique<pvegas::pros_adapters::ProsMutex>()};
  std::unique_ptr<pvegas::rtos::ITask> odometry_task{
      std::make_unique<pvegas::pros_adapters::ProsTask>()};
  // position tracker
  pvegas::robot::subsystems::odometry::InertialPositionTrackerBuilder
      inertial_position_tracker_builder{};
  std::unique_ptr<pvegas::robot::subsystems::odometry::IPositionTracker>
      inertial_position_tracker{
          inertial_position_tracker_builder.withClock(odometry_clock)
              ->withDelayer(odometry_delayer)
              ->withMutex(odometry_mutex)
              ->withTask(odometry_task)
              ->withInertialSensor(inertial_sensor)
              ->withLeftDistanceTracker(left_tracking_wheel)
              ->withLeftDistanceTrackerOffset(LEFT_TRACKING_WHEEL_OFFSET)
              ->withRightDistanceTracker(right_tracking_wheel)
              ->withRightDistanceTrackerOffset(RIGHT_TRACKING_WHEEL_OFFSET)
              ->build()};
  // position resetter
  pvegas::robot::subsystems::odometry::DistancePositionResetterBuilder
      distance_position_resetter_builder{};
  std::unique_ptr<pvegas::robot::subsystems::odometry::IPositionResetter>
      distance_position_resetter{
          distance_position_resetter_builder
              .withDistanceSensor(distance_sensor)
              ->withLocalX(RESETTER_LOCAL_X_OFFSET)
              ->withLocalY(RESETTER_LOCAL_Y_OFFSET)
              ->withLocalTheta(RESETTER_LOCAL_THETA_OFFSET)
              ->build()};

  // create the subsystem
  std::unique_ptr<pvegas::robot::ASubsystem> odometry_subsystem{
      std::make_unique<pvegas::robot::subsystems::odometry::OdometrySubsystem>(
          inertial_position_tracker, distance_position_resetter)};
  robot->addSubsystem(odometry_subsystem);
  return robot;
}
}  // namespace config
}  // namespace pvegas