#include "pvegas/config/DefaultConfig.hpp"
#include "pvegas/robot/subsystems/drivetrain/DrivetrainSubsystem.hpp"
#include "pvegas/robot/subsystems/drivetrain/IDriveTrain.hpp"
#include <memory>

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

  // DRIVE TRAIN SETUP
  // creates the factory used to build the drivetrain
  robot::subsystems::drivetrain::DirectDriveBuilder drive_factory{};

  // Objects needed for drivetrain
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
    //call the factory and add all necessary items for the drivetrain
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

  return robot;
}
} // namespace config
} // namespace pvegas