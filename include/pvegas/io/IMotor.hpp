#ifndef __I_MOTOR_HPP__
#define __I_MOTOR_HPP__

namespace driftless {
namespace io {
class IMotor {
 public:
  virtual ~IMotor() = default;

  virtual void initialize() = 0;

  virtual double getTorqueConstant() = 0;

  virtual double getResistance() = 0;

  virtual double getAngularVelocityConstant() = 0;

  virtual double getGearRatio() = 0;

  virtual double getAngularVelocity() = 0;

  virtual double getPosition() = 0;

  virtual void setVoltage(double volts) = 0;

  virtual void setPosition(double position) = 0;
};
}  // namespace io
}  // namespace pvegas
#endif