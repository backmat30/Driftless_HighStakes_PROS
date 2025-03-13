#include "driftless/config/DefaultConfig.hpp"

namespace driftless {
namespace config {
std::string DefaultConfig::getName() { return CONFIG_NAME; }

std::shared_ptr<control::ControlSystem> DefaultConfig::buildControlSystem() {
  // creates a new ControlSystem object through the copy constructor
  std::shared_ptr<control::ControlSystem> control_system{
      std::make_shared<control::ControlSystem>()};

  // delayer and clock passed to all controls
  std::unique_ptr<driftless::rtos::IClock> clock{
      std::make_unique<driftless::pros_adapters::ProsClock>()};
  std::unique_ptr<driftless::rtos::IDelayer> delayer{
      std::make_unique<driftless::pros_adapters::ProsDelayer>()};

  // MOTION CONTROL
  driftless::control::motion::PIDDriveStraightBuilder
      pid_drive_straight_builder{};
  // objects needed for drive straight algorithm
  std::unique_ptr<driftless::rtos::IMutex> pid_drive_straight_mutex{
      std::make_unique<driftless::pros_adapters::ProsMutex>()};
  std::unique_ptr<driftless::rtos::ITask> pid_drive_straight_task{
      std::make_unique<driftless::pros_adapters::ProsTask>()};
  driftless::control::PID pid_drive_straight_linear_pid{
      clock, PID_DRIVE_STRAIGHT_LINEAR_KP, PID_DRIVE_STRAIGHT_LINEAR_KI,
      PID_DRIVE_STRAIGHT_LINEAR_KD};
  driftless::control::PID pid_drive_straight_rotational_pid{
      clock, PID_DRIVE_STRAIGHT_ROTATIONAL_KP, PID_DRIVE_STRAIGHT_ROTATIONAL_KI,
      PID_DRIVE_STRAIGHT_ROTATIONAL_KD};

  // assemble the drive straight object
  std::unique_ptr<driftless::control::motion::IDriveStraight> drive_straight{
      pid_drive_straight_builder.withDelayer(delayer)
          ->withMutex(pid_drive_straight_mutex)
          ->withTask(pid_drive_straight_task)
          ->withLinearPID(pid_drive_straight_linear_pid)
          ->withRotationalPID(pid_drive_straight_rotational_pid)
          ->withTargetTolerance(PID_DRIVE_STRAIGHT_TARGET_TOLERANCE)
          ->withTargetVelocity(PID_DRIVE_STRAIGHT_TARGET_VELOCITY)
          ->build()};

  driftless::control::motion::PIDGoToPointBuilder pid_go_to_point_builder{};
  // objects needed for go to point algorithm
  std::unique_ptr<driftless::rtos::IMutex> pid_go_to_point_mutex{
      std::make_unique<driftless::pros_adapters::ProsMutex>()};
  std::unique_ptr<driftless::rtos::ITask> pid_go_to_point_task{
      std::make_unique<driftless::pros_adapters::ProsTask>()};
  driftless::control::PID pid_go_to_point_linear_pid{
      clock, PID_GO_TO_POINT_LINEAR_KP, PID_GO_TO_POINT_LINEAR_KI,
      PID_GO_TO_POINT_LINEAR_KD};
  driftless::control::PID pid_go_to_point_rotational_pid{
      clock, PID_GO_TO_POINT_ROTATIONAL_KP, PID_GO_TO_POINT_ROTATIONAL_KI,
      PID_GO_TO_POINT_ROTATIONAL_KD};

  // assemble the go to point object
  std::unique_ptr<driftless::control::motion::IGoToPoint> go_to_point{
      pid_go_to_point_builder.withDelayer(delayer)
          ->withMutex(pid_go_to_point_mutex)
          ->withTask(pid_go_to_point_task)
          ->withLinearPID(pid_go_to_point_linear_pid)
          ->withRotationalPID(pid_go_to_point_rotational_pid)
          ->withTargetTolerance(PID_GO_TO_POINT_TARGET_TOLERANCE)
          ->withTargetVelocity(PID_GO_TO_POINT_TARGET_VELOCITY)
          ->build()};

  driftless::control::motion::PIDTurnBuilder pid_turn_builder{};
  // objects needed for turn algorithm
  std::unique_ptr<driftless::rtos::IMutex> pid_turn_mutex{
      std::make_unique<driftless::pros_adapters::ProsMutex>()};
  std::unique_ptr<driftless::rtos::ITask> pid_turn_task{
      std::make_unique<driftless::pros_adapters::ProsTask>()};
  driftless::control::PID pid_turn_rotational_pid{clock, PID_TURN_ROTATIONAL_KP,
                                                  PID_TURN_ROTATIONAL_KI,
                                                  PID_TURN_ROTATIONAL_KD};

  // assemble the turn object
  std::unique_ptr<driftless::control::motion::ITurn> turn{
      pid_turn_builder.withDelayer(delayer)
          ->withMutex(pid_turn_mutex)
          ->withTask(pid_turn_task)
          ->withRotationalPID(pid_turn_rotational_pid)
          ->withTargetTolerance(PID_TURN_TARGET_TOLERANCE)
          ->withTargetVelocity(PID_TURN_TARGET_VELOCITY)
          ->build()};

  // create and add the motion control
  std::unique_ptr<driftless::control::AControl> motion_control{
      std::make_unique<driftless::control::motion::MotionControl>(
          drive_straight, go_to_point, turn)};
  control_system->addControl(motion_control);

  // PATH FOLLOWER CONTROL
  driftless::control::path::PIDPathFollowerBuilder pid_path_follower_builder{};
  // needed objects for the path follower
  std::unique_ptr<driftless::rtos::IMutex> pid_path_follower_mutex{
      std::make_unique<driftless::pros_adapters::ProsMutex>()};
  std::unique_ptr<driftless::rtos::ITask> pid_path_follower_task{
      std::make_unique<driftless::pros_adapters::ProsTask>()};
  driftless::control::PID pid_path_follower_linear_pid{
      clock, PID_PATH_FOLLOWER_LINEAR_KP, PID_PATH_FOLLOWER_LINEAR_KI,
      PID_PATH_FOLLOWER_LINEAR_KD};
  driftless::control::PID pid_path_follower_rotational_pid{
      clock, PID_PATH_FOLLOWER_ROTATIONAL_KP, PID_PATH_FOLLOWER_ROTATIONAL_KI,
      PID_PATH_FOLLOWER_ROTATIONAL_KD};

  // assemble the path follower
  std::unique_ptr<driftless::control::path::IPathFollower> pid_path_follower{
      pid_path_follower_builder.withDelayer(delayer)
          ->withMutex(pid_path_follower_mutex)
          ->withTask(pid_path_follower_task)
          ->withLinearPID(pid_path_follower_linear_pid)
          ->withRotationalPID(pid_path_follower_rotational_pid)
          ->withFollowDistance(PID_PATH_FOLLOWER_FOLLOW_DISTANCE)
          ->withTargetTolerance(PID_PATH_FOLLOWER_TARGET_TOLERANCE)
          ->withTargetVelocity(PID_PATH_FOLLOWER_TARGET_VELOCITY)
          ->build()};

  // put the path follower in a control object
  std::unique_ptr<driftless::control::AControl> path_follower_control{
      std::make_unique<driftless::control::path::PathFollowerControl>(
          pid_path_follower)};
  // insert the path follower control into the control manager
  control_system->addControl(path_follower_control);

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
  std::unique_ptr<robot::subsystems::ASubsystem> drivetrain_subsystem{
      std::make_unique<robot::subsystems::drivetrain::DrivetrainSubsystem>(
          drivetrain)};
  // add the new subsystem to the robot
  robot->addSubsystem(drivetrain_subsystem);

  // ARM

  // rtos
  std::unique_ptr<driftless::rtos::IClock> arm_clock{
      std::make_unique<driftless::pros_adapters::ProsClock>()};
  std::unique_ptr<driftless::rtos::IDelayer> arm_delayer{
      std::make_unique<driftless::pros_adapters::ProsDelayer>()};
  std::unique_ptr<driftless::rtos::IMutex> arm_mutex{
      std::make_unique<driftless::pros_adapters::ProsMutex>()};
  std::unique_ptr<driftless::rtos::ITask> arm_task{
      std::make_unique<driftless::pros_adapters::ProsTask>()};

  // pros objects
  std::unique_ptr<pros::Motor> temp_arm_left_rotation_motor{
      std::make_unique<pros::Motor>(ARM_LEFT_ROTATION_MOTOR,
                                    ARM_ROTATIONAL_GEARSET)};
  std::unique_ptr<pros::Motor> temp_arm_right_rotation_motor{
      std::make_unique<pros::Motor>(ARM_RIGHT_ROTATION_MOTOR,
                                    ARM_ROTATIONAL_GEARSET)};
  std::unique_ptr<pros::Motor> temp_arm_linear_motor{
      std::make_unique<pros::Motor>(ARM_LINEAR_MOTOR, ARM_LINEAR_GEARSET)};

  // adapted objects
  std::unique_ptr<driftless::io::IMotor> arm_left_rotation_motor{
      std::make_unique<driftless::pros_adapters::ProsV5Motor>(
          temp_arm_left_rotation_motor)};
  std::unique_ptr<driftless::io::IMotor> arm_right_rotation_motor{
      std::make_unique<driftless::pros_adapters::ProsV5Motor>(
          temp_arm_right_rotation_motor)};
  std::unique_ptr<driftless::io::IMotor> arm_linear_motor{
      std::make_unique<driftless::pros_adapters::ProsV5Motor>(
          temp_arm_linear_motor)};

  driftless::control::PID arm_rotational_pid{arm_clock, PID_ARM_ROTATIONAL_KP,
                                             PID_ARM_ROTATIONAL_KI,
                                             PID_ARM_ROTATIONAL_KD};
  driftless::control::PID arm_linear_pid{arm_clock, PID_ARM_LINEAR_KP,
                                         PID_ARM_LINEAR_KI, PID_ARM_LINEAR_KD};

  // assemble the subsystem
  driftless::robot::subsystems::arm::PIDArmMotionBuilder
      pid_arm_motion_builder{};

  std::unique_ptr<driftless::robot::subsystems::arm::IArmMotion> arm_motion{
      pid_arm_motion_builder.withClock(arm_clock)
          ->withDelayer(arm_delayer)
          ->withMutex(arm_mutex)
          ->withTask(arm_task)
          ->withRotationalMotor(arm_left_rotation_motor)
          ->withRotationalMotor(arm_right_rotation_motor)
          ->withLinearMotor(arm_linear_motor)
          ->withRotationalPID(arm_rotational_pid)
          ->withLinearPID(arm_linear_pid)
          ->withRotationalNeutralPosition(ARM_ROTATIONAL_NEUTRAL_POSITION)
          ->withRotationalLoadPosition(ARM_ROTATIONAL_LOAD_POSITION)
          ->withRotationalReadyPosition(ARM_ROTATIONAL_READY_POSITION)
          ->withRotationalScorePosition(ARM_ROTATIONAL_SCORE_POSITION)
          ->withRotationalRushPosition(ARM_ROTATIONAL_RUSH_POSITION)
          ->withRotationalAllianceStakePosition(
              ARM_ROTATIONAL_ALLIANCE_STAKE_POSITION)
          ->withRotationalReadyIntermediatePosition(
              ARM_ROTATIONAL_READY_INTERMEDIATE_POSITION)
          ->withRotationalScoreIntermediatePosition(
              ARM_ROTATIONAL_SCORE_INTERMEDIATE_POSITION)
          ->withRotationalRushIntermediatePosition(
              ARM_ROTATIONAL_RUSH_INTERMEDIATE_POSITION)
          ->withRotationalTolerance(ARM_ROTATIONAL_TOLERANCE)
          ->withLinearNeutralPosition(ARM_LINEAR_NEUTRAL_POSITION)
          ->withLinearLoadPosition(ARM_LINEAR_LOAD_POSITION)
          ->withLinearReadyPosition(ARM_LINEAR_READY_POSITION)
          ->withLinearScorePosition(ARM_LINEAR_SCORE_POSITION)
          ->withLinearRushPosition(ARM_LINEAR_RUSH_POSITION)
          ->withLinearAllianceStakePosition(ARM_LINEAR_ALLIANCE_STAKE_POSITION)
          ->withLinearTolerance(ARM_LINEAR_TOLERANCE)
          ->build()};

  std::unique_ptr<driftless::robot::subsystems::ASubsystem> arm_subsystem{
      std::make_unique<driftless::robot::subsystems::arm::ArmSubsystem>(
          arm_motion)};

  robot->addSubsystem(arm_subsystem);

  // CLAMP

  // pros objects
  std::unique_ptr<pros::adi::DigitalOut> temp_clamp_left_piston{
      std::make_unique<pros::adi::DigitalOut>(CLAMP_PISTON_1)};

  // adapted objects
  std::unique_ptr<driftless::io::IPiston> adapted_clamp_left_piston{
      std::make_unique<driftless::pros_adapters::ProsPiston>(
          temp_clamp_left_piston)};

  // build the clamp
  driftless::robot::subsystems::clamp::PistonClampBuilder
      piston_clamp_builder{};

  std::unique_ptr<driftless::robot::subsystems::clamp::IClamp> piston_clamp{
      piston_clamp_builder.withPiston(adapted_clamp_left_piston)->build()};

  std::unique_ptr<driftless::robot::subsystems::ASubsystem> clamp_subsystem{
      std::make_unique<driftless::robot::subsystems::clamp::ClampSubsystem>(
          piston_clamp)};
  robot->addSubsystem(clamp_subsystem);

  // ELEVATOR

  // rtos
  std::unique_ptr<driftless::rtos::IClock> elevator_clock{
      std::make_unique<driftless::pros_adapters::ProsClock>()};
  std::unique_ptr<driftless::rtos::IDelayer> elevator_delayer{
      std::make_unique<driftless::pros_adapters::ProsDelayer>()};
  std::unique_ptr<driftless::rtos::IMutex> elevator_mutex{
      std::make_unique<driftless::pros_adapters::ProsMutex>()};
  std::unique_ptr<driftless::rtos::ITask> elevator_task{
      std::make_unique<driftless::pros_adapters::ProsTask>()};
  // pros objects
  std::unique_ptr<pros::Motor> temp_elevator_motor_1{
      std::make_unique<pros::Motor>(ELEVATOR_MOTOR_1)};
  std::unique_ptr<pros::adi::DigitalOut> temp_elevator_rejection_piston{
      std::make_unique<pros::adi::DigitalOut>(ELEVATOR_REJECTION_PISTON)};

  // adapted objects
  std::unique_ptr<driftless::io::IMotor> adapted_elevator_motor_1{
      std::make_unique<driftless::pros_adapters::ProsV5Motor>(
          temp_elevator_motor_1)};
  std::unique_ptr<driftless::io::IPiston> adapted_elevator_rejection_piston{
      std::make_unique<driftless::pros_adapters::ProsPiston>(
          temp_elevator_rejection_piston)};

  driftless::control::PID elevator_pid{elevator_clock, PID_ELEVATOR_KP,
                                       PID_ELEVATOR_KI, PID_ELEVATOR_KD};

  // build the elevator
  driftless::robot::subsystems::elevator::PIDElevatorBuilder
      pid_elevator_builder{};

  std::unique_ptr<driftless::robot::subsystems::elevator::IElevator> elevator{
      pid_elevator_builder.withClock(elevator_clock)
          ->withDelayer(elevator_delayer)
          ->withMutex(elevator_mutex)
          ->withTask(elevator_task)
          ->withMotor(adapted_elevator_motor_1)
          ->withPID(elevator_pid)
          ->withRadiansToInches(ELEVATOR_RADIANS_TO_INCHES)
          ->build()};

  driftless::robot::subsystems::elevator::PistonRingRejectionBuilder
      piston_ring_rejection_builder{};

  std::unique_ptr<robot::subsystems::elevator::IRingRejection> ring_rejection{
      piston_ring_rejection_builder
          .withPiston(adapted_elevator_rejection_piston)
          ->build()};

  std::unique_ptr<driftless::robot::subsystems::ASubsystem> elevator_subsystem{
      std::make_unique<
          driftless::robot::subsystems::elevator::ElevatorSubsystem>(
          elevator, ring_rejection)};
  robot->addSubsystem(elevator_subsystem);

  // INTAKE

  // pros objects
  std::unique_ptr<pros::Motor> temp_intake_motor_1{
      std::make_unique<pros::Motor>(INTAKE_MOTOR)};
  std::unique_ptr<pros::adi::DigitalOut> temp_intake_piston{
      std::make_unique<pros::adi::DigitalOut>(INTAKE_PISTON)};

  // adapted objects
  std::unique_ptr<driftless::io::IMotor> intake_motor_1{
      std::make_unique<driftless::pros_adapters::ProsV5Motor>(
          temp_intake_motor_1)};
  std::unique_ptr<driftless::io::IPiston> intake_piston{
      std::make_unique<driftless::pros_adapters::ProsPiston>(
          temp_intake_piston)};

  // build the intake
  driftless::robot::subsystems::intake::DirectIntakeBuilder
      direct_intake_builder{};
  driftless::robot::subsystems::intake::PistonHeightControlBuilder
      piston_height_control_builder{};

  std::unique_ptr<driftless::robot::subsystems::intake::IIntake> direct_intake{
      direct_intake_builder.withMotor(intake_motor_1)->build()};
  std::unique_ptr<driftless::robot::subsystems::intake::IHeightControl>
      piston_height_control{
          piston_height_control_builder.withPiston(intake_piston)->build()};

  // create and add the intake subsystem to the robot
  std::unique_ptr<driftless::robot::subsystems::ASubsystem> intake_subsystem{
      std::make_unique<driftless::robot::subsystems::intake::IntakeSubsystem>(
          direct_intake, piston_height_control)};
  robot->addSubsystem(intake_subsystem);

  // ODOMETRY

  // pros objects
  std::unique_ptr<pros::Rotation> temp_linear_rotation_sensor{
      std::make_unique<pros::Rotation>(ODOMETRY_LINEAR_TRACKING_WHEEL)};
  std::unique_ptr<pros::Rotation> temp_strafe_rotation_sensor{
      std::make_unique<pros::Rotation>(ODOMETRY_STRAFE_TRACKING_WHEEL)};
  std::unique_ptr<pros::IMU> temp_inertial_sensor{
      std::make_unique<pros::IMU>(ODOMETRY_INERTIAL_SENSOR)};
  std::unique_ptr<pros::Distance> temp_distance_sensor{
      std::make_unique<pros::Distance>(ODOMETRY_DISTANCE_SENSOR)};

  // adapted objects
  // linear tracking wheel
  std::unique_ptr<driftless::io::IRotationSensor> linear_rotation_sensor{
      std::make_unique<driftless::pros_adapters::ProsRotationSensor>(
          temp_linear_rotation_sensor)};
  std::unique_ptr<driftless::io::IDistanceTracker> linear_tracking_wheel{
      std::make_unique<driftless::hal::TrackingWheel>(linear_rotation_sensor,
                                                      TRACKING_WHEEL_RADIUS)};
  // strafe tracking wheel
  std::unique_ptr<driftless::io::IRotationSensor> strafe_rotation_sensor{
      std::make_unique<driftless::pros_adapters::ProsRotationSensor>(
          temp_strafe_rotation_sensor)};
  std::unique_ptr<driftless::io::IDistanceTracker> strafe_tracking_wheel{
      std::make_unique<driftless::hal::TrackingWheel>(strafe_rotation_sensor,
                                                      TRACKING_WHEEL_RADIUS)};
  // inertial sensor
  std::unique_ptr<driftless::io::IInertialSensor> inertial_sensor{
      std::make_unique<driftless::pros_adapters::ProsInertialSensor>(
          temp_inertial_sensor)};
  // distance sensor
  std::unique_ptr<driftless::io::IDistanceSensor> distance_sensor{
      std::make_unique<driftless::pros_adapters::ProsDistanceSensor>(
          temp_distance_sensor)};
  // odometry rtos
  std::unique_ptr<driftless::rtos::IClock> odometry_clock{
      std::make_unique<driftless::pros_adapters::ProsClock>()};
  std::unique_ptr<driftless::rtos::IDelayer> odometry_delayer{
      std::make_unique<driftless::pros_adapters::ProsDelayer>()};
  std::unique_ptr<driftless::rtos::IMutex> odometry_mutex{
      std::make_unique<driftless::pros_adapters::ProsMutex>()};
  std::unique_ptr<driftless::rtos::ITask> odometry_task{
      std::make_unique<driftless::pros_adapters::ProsTask>()};
  // position tracker
  driftless::robot::subsystems::odometry::InertialPositionTrackerBuilder
      inertial_position_tracker_builder{};
  std::unique_ptr<driftless::robot::subsystems::odometry::IPositionTracker>
      inertial_position_tracker{
          inertial_position_tracker_builder.withClock(odometry_clock)
              ->withDelayer(odometry_delayer)
              ->withMutex(odometry_mutex)
              ->withTask(odometry_task)
              ->withInertialSensor(inertial_sensor)
              ->withLinearDistanceTracker(linear_tracking_wheel)
              ->withLinearDistanceTrackerOffset(LINEAR_TRACKING_WHEEL_OFFSET)
              ->withStrafeDistanceTracker(strafe_tracking_wheel)
              ->withStrafeDistanceTrackerOffset(STRAFE_TRACKING_WHEEL_OFFSET)
              ->build()};
  // position resetter
  driftless::robot::subsystems::odometry::DistancePositionResetterBuilder
      distance_position_resetter_builder{};
  std::unique_ptr<driftless::robot::subsystems::odometry::IPositionResetter>
      distance_position_resetter{
          distance_position_resetter_builder
              .withDistanceSensor(distance_sensor)
              ->withLocalX(RESETTER_LOCAL_X_OFFSET)
              ->withLocalY(RESETTER_LOCAL_Y_OFFSET)
              ->withLocalTheta(RESETTER_LOCAL_THETA_OFFSET)
              ->build()};

  // create and add the subsystem
  std::unique_ptr<driftless::robot::subsystems::ASubsystem> odometry_subsystem{
      std::make_unique<
          driftless::robot::subsystems::odometry::OdometrySubsystem>(
          inertial_position_tracker, distance_position_resetter)};
  robot->addSubsystem(odometry_subsystem);

  // RING SORT

  std::unique_ptr<pros::Optical> ring_sort_temp_optical{
      std::make_unique<pros::Optical>(RING_SORT_COLOR_SENSOR)};

  std::unique_ptr<io::IColorSensor> ring_sort_adapted_color_sensor{
      std::make_unique<pros_adapters::ProsColorSensor>(ring_sort_temp_optical)};

  robot::subsystems::ring_sort::ColorRingSortBuilder color_ring_sort_builder{};

  std::unique_ptr<robot::subsystems::ring_sort::IRingSort> ring_sort{
      color_ring_sort_builder.withColorSensor(ring_sort_adapted_color_sensor)
          ->withMaxRingDistance(RING_SORT_MIN_RING_PROXIMITY)
          ->withDistanceToElevatorEnd(RING_SORT_COLOR_SENSOR_TO_END)
          ->build()};

  std::unique_ptr<robot::subsystems::ASubsystem> ring_sort_subsystem{
      std::make_unique<robot::subsystems::ring_sort::RingSortSubsystem>(
          ring_sort)};

  robot->addSubsystem(ring_sort_subsystem);

  // send out the finalized robot
  return robot;
}

std::shared_ptr<processes::ProcessSystem> DefaultConfig::buildProcessSystem() {
  std::shared_ptr<processes::ProcessSystem> process_system{
      std::make_shared<processes::ProcessSystem>()};

  std::unique_ptr<rtos::IDelayer> ring_rejector_delayer{
      std::make_unique<pros_adapters::ProsDelayer>()};
  std::unique_ptr<rtos::IMutex> ring_rejector_mutex{
      std::make_unique<pros_adapters::ProsMutex>()};
  std::unique_ptr<rtos::ITask> ring_rejector_task{
      std::make_unique<pros_adapters::ProsTask>()};

  processes::auto_ring_rejection::ElevatorAutoRingRejectorBuilder
      elevator_auto_ring_rejector_builder{};

  std::unique_ptr<processes::auto_ring_rejection::IAutoRingRejector>
      auto_ring_rejector{elevator_auto_ring_rejector_builder
                             .withDelayer(ring_rejector_delayer)
                             ->withMutex(ring_rejector_mutex)
                             ->withTask(ring_rejector_task)
                             ->build()};

  std::unique_ptr<processes::AProcess> auto_ring_rejection_process{
      std::make_unique<
          processes::auto_ring_rejection::AutoRingRejectionProcess>(
          auto_ring_rejector)};
  process_system->addProcess(auto_ring_rejection_process);

  return process_system;
}
}  // namespace config
}  // namespace driftless