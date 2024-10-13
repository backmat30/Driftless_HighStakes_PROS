#ifndef __DEFAULT_CONFIG_HPP__
#define __DEFAULT_CONFIG_HPP__

#include <memory>

#include "pros/abstract_motor.hpp"
#include "pros/motors.h"
#include "pvegas/config/IConfig.hpp"
#include "pvegas/control/ControlSystem.hpp"
#include "pvegas/io/IController.hpp"
#include "pvegas/pros_adapters/ProsClock.hpp"
#include "pvegas/pros_adapters/ProsController.hpp"
#include "pvegas/pros_adapters/ProsDelayer.hpp"
#include "pvegas/pros_adapters/ProsMutex.hpp"
#include "pvegas/pros_adapters/ProsTask.hpp"
#include "pvegas/pros_adapters/ProsV5Motor.hpp"
#include "pvegas/robot/Robot.hpp"
#include "pvegas/robot/subsystems/drivetrain/DirectDriveBuilder.hpp"
#include "pvegas/rtos/IClock.hpp"
#include "pvegas/rtos/IDelayer.hpp"
#include "pvegas/rtos/IMutex.hpp"
#include "pvegas/rtos/ITask.hpp"
namespace pvegas {
namespace config {
class DefaultConfig : public IConfig {
 private:
  static constexpr char CONFIG_NAME[]{"DEFAULT CONFIG"};

  // -----PORT NUMBERS-----
  // DRIVE MOTORS
  // first left drive motor
  static constexpr int8_t DRIVE_LEFT_MOTOR_1{12};
  // second left drive motor
  static constexpr int8_t DRIVE_LEFT_MOTOR_2{13};
  // third left drive motor
  static constexpr int8_t DRIVE_LEFT_MOTOR_3{-14};
  // fourth left drive motor
  static constexpr int8_t DRIVE_LEFT_MOTOR_4{-6};
  // first right drive motor
  static constexpr int8_t DRIVE_RIGHT_MOTOR_1{-1};
  // second right drive motor
  static constexpr int8_t DRIVE_RIGHT_MOTOR_2{-1};
  // third right drive motor
  static constexpr int8_t DRIVE_RIGHT_MOTOR_3{-1};
  // fourth right drive motor
  static constexpr int8_t DRIVE_RIGHT_MOTOR_4{-1};

  // -----MISC VALUES-----
  // drive gearset
  static constexpr pros::MotorGearset DRIVE_GEARSET{pros::E_MOTOR_GEAR_BLUE};
  // drive ratio of motor voltage to velocity
  static constexpr double DRIVE_VELOCITY_TO_VOLTAGE{1.0};
  // radius of the robot
  static constexpr double ROBOT_RADIUS{7.25};
  // radius of the drive wheels
  static constexpr double DRIVE_WHEEL_RADIUS{1.25};

 public:
  std::string getName() override;

  std::shared_ptr<control::ControlSystem> buildControlSystem() override;

  std::shared_ptr<io::IController> buildController() override;

  std::shared_ptr<robot::Robot> buildRobot() override;
};
}  // namespace config
}  // namespace pvegas
#endif