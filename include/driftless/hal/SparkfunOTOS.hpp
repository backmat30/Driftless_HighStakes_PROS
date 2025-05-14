#ifndef __SPARKFUN_OTOS_HPP__
#define __SPARKFUN_OTOS_HPP__

#include <cmath>
#include <memory>
#include <string>

#include "driftless/io/IPositionSensor.hpp"
#include "driftless/io/ISerialDevice.hpp"

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for the hardware abstraction layer
/// @author Matthew Backman
namespace hal {

class SparkfunOTOS : public io::IPositionSensor {
 private:
  std::unique_ptr<io::ISerialDevice> m_serial_device{};

  robot::subsystems::odometry::Position latest_position{};

  std::string arduino_buffer{};

  int empty_buffers{};

  /// @brief Looks for new data from the serial device and updates the latest
  /// position
  void updatePosition();

 public:
  /// @brief Constructs a new SparkfunOTOS object
  /// @param serialDevice __std::unique_ptr<io::ISerialDevice>&__ The serial
  /// device used for the Sparkfun OTOS
  SparkfunOTOS(std::unique_ptr<io::ISerialDevice>& serialDevice);

  /// @brief Initializes the SparkfunOTOS object
  void init() override;

  /// @brief Gets the position of the Sparkfun OTOS sensor
  /// @return __robot::subsystems::odometry::Position__ The position
  robot::subsystems::odometry::Position getPosition() override;

  /// @brief Sets the local offset of the OTOS sensor
  /// @param x_offset __double__ The local x-offset
  /// @param y_offset __double__ The local y-offset
  /// @param theta_offset __double The local theta-offset
  void setLocalOffset(double x_offset, double y_offset, double theta_offset) override;
};
}  // namespace hal
}  // namespace driftless
#endif