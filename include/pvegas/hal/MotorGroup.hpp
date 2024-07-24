#ifndef __MOTOR_GROUP_HPP__
#define __MOTOR_GROUP_HPP__
#include "pvegas/pros_adapters/ProsV5Motor.hpp"
#include <memory>
#include <vector>
namespace pvegas {
namespace hal {
class MotorGroup {
private:
  std::vector<std::unique_ptr<pros_adapters::ProsV5Motor>> motors{};

public:
  void addMotor(std::unique_ptr<pros_adapters::ProsV5Motor> &motor);

  void init();

  double getTorqueConstant();

  double getResistance();
  
  double getAngularVelocityConstant();

  double getGearRatio();

  double getAngularVelocity();

  double getPosition();

  void setVoltage(double voltage);

  void setPosition(double position);

  MotorGroup& operator=(MotorGroup& rhs);
};
} // namespace hal
} // namespace pvegas
#endif