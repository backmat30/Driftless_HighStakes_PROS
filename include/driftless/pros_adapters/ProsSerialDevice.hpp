#ifndef __PROS_SERIAL_DEVICE_HPP__
#define __PROS_SERIAL_DEVICE_HPP__

#include <cstring>
#include <memory>

#include "driftless/io/ISerialDevice.hpp"
#include "pros/serial.hpp"

namespace driftless {

namespace pros_adapters {

/// @brief Class to adapt a pros serial device to the serial device interface
class ProsSerialDevice : public io::ISerialDevice {
 private:
  std::unique_ptr<pros::Serial> m_serial_device{};

 public:
  /// @brief Constructs a new pros serial device
  /// @param serial_device __std::unique_ptr<pros::Serial>&__ The serial device
  /// being adapted
  ProsSerialDevice(std::unique_ptr<pros::Serial>& serial_device);

  /// @brief Initializes the pros serial device
  void initialize() override;

  /// @brief Reads a double from the input stream
  /// @return __double__ The first item from the input stream as a double
  double readDouble() override;

  /// @brief Writes to the output stream
  /// @param buffer __uint8_t*__ Array of bytes to be sent
  /// @param length __int__ The length of the output array
  void write(uint8_t* buffer, int length) override;

  /// @brief Clears both input and output streams
  void flush() override;

  /// @brief Gets the number of bytes in the input stream
  /// @return __int__ The number of bytes in the input stream
  int getInputBytes();
};
}  // namespace pros_adapters
}  // namespace driftless
#endif