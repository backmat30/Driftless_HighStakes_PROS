#ifndef __I_SERIAL_DEVICE_HPP__
#define __I_SERIAL_DEVICE_HPP__

#include <stdint.h>

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for IO device interfaces
/// @author Matthew Backman
namespace io {

/// @brief Interface representing a generic serial device
class ISerialDevice {
 public:
  /// @brief Destroys the serial device object
  virtual ~ISerialDevice() = default;

  /// @brief Initializes the serial device
  virtual void initialize() = 0;

  /// @brief Reads a double from the input stream
  /// @return __double__ The first item in the byte stream, as a double
  virtual double readDouble() = 0;

  /// @brief Writes to the output stream
  /// @param output_bytes __uint8_t*__ Array of bytes
  /// @param length __int__ The length of the output array
  virtual void write(uint8_t* output_bytes, int length) = 0;

  /// @brief Clears both input and output streams
  virtual void flush() = 0;

  /// @brief Gets the bytes in the input stream
  /// @return __int__ The number of bytes in the input stream
  virtual int getInputBytes() = 0;
};
}  // namespace io
}  // namespace driftless
#endif