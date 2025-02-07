#ifndef __MOTOR_GROUP_HPP__
#define __MOTOR_GROUP_HPP__
#include <memory>
#include <vector>

#include "driftless/io/IMotor.hpp"
/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for the hardware abstraction layer
/// @author Matthew Backman
namespace hal {

/// @brief Class for a group of related motors
/// @author Matthew Backman
class MotorGroup {
 private:
  /// @brief The motors in the group
  std::vector<std::unique_ptr<io::IMotor>> motors{};

 public:
  /// @brief Adds a motor to the group
  /// @param motor __std::unique_ptr<io::IMotor>&__ The motor to add
  void addMotor(std::unique_ptr<io::IMotor>& motor);

  /// @brief Initializes the motor group
  void init();

  /// @brief Gets the torque constant of the motors in the group
  /// @return __double__ the torque constant
  double getTorqueConstant();

  /// @brief Gets the average resistance of the motors in the group
  /// @return __double__ the resistance
  double getResistance();

  /// @brief Gets the average angular velocity constant of the motors in the
  /// group
  /// @return __double__ the angular velocity constant
  double getAngularVelocityConstant();

  /// @brief Gets the average gear ratio of the motors in the group
  /// @return __double__ the gear ratio
  double getGearRatio();

  /// @brief Gets the average angular velocity of the motors in the group
  /// @return __double__ The angular velocity
  double getAngularVelocity();

  /// @brief Gets the average position of the motors in the group
  /// @return __double__ The position
  double getPosition();

  /// @brief Gets the average efficiency of the motors in the group
  /// @return __double__ The efficiency
  double getEfficiency();

  /// @brief Sets the voltage of all motors in the group
  /// @param voltage __double__ The new voltage
  void setVoltage(double voltage);

  /// @brief Sets the encoder position of all motors in the group
  /// @param position __double__ The new position
  void setPosition(double position);

  /// @brief Copies a given motor group
  /// @param rhs __MotorGroup&__ Reference to the desired motor group
  /// @return __MotorGroup&__ Reference to the copy of the motor group
  MotorGroup& operator=(MotorGroup& rhs);
};
}  // namespace hal
}  // namespace driftless
#endif