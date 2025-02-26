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

  /// @brief Reads the next byte in the input stream
  /// @return __uint8_t__ The next available byte
  uint8_t readByte() override;

  /// @brief Checks the next byte in the input stream without removing it
  /// @return __uint8_t__ The next byte in the input stream
  uint8_t peekByte() override;

  /// @brief Reads from the input stream into a buffer
  /// @param buffer __uint8_t*__ Array of bytes to store read values
  /// @param length __int__ Number of bytes to read
  void read(uint8_t* buffer, int length) override;

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