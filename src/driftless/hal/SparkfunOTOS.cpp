#include "driftless/hal/SparkfunOTOS.hpp"

#include "pros/screen.hpp"

namespace driftless::hal {

void SparkfunOTOS::updatePosition() {
  // ammends any new data from the serial device to the buffer string
  if (m_serial_device) {
    while (m_serial_device->getInputBytes()) {
      arduino_buffer += static_cast<char>(m_serial_device->readByte());
    }
    pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 5, "%s", arduino_buffer);
  }

  if (arduino_buffer.find('/') != std::string::npos) {
    arduino_buffer = arduino_buffer.substr(arduino_buffer.find('/'));

    while (arduino_buffer.find(';') != std::string::npos) {
      arduino_buffer = arduino_buffer.substr(arduino_buffer.find('/') + 1);

      char current_key{static_cast<char>(arduino_buffer.at(0))};

      int value_start{2};
      uint32_t value_end{arduino_buffer.find(';') - 1};
      std::string value{
          arduino_buffer.substr(value_start, value_end - value_start)};

      double value_as_double{};
      try {
        value_as_double = std::stod(value);
      } catch (std::invalid_argument& e) {
        arduino_buffer = "";
        return;
      }

      switch (current_key) {
        case 'X':
          latest_position.x = value_as_double;
          break;
        case 'Y':
          latest_position.y = value_as_double;
          break;
        case 'H':
          latest_position.theta = value_as_double * M_PI / 180;
          break;
      }

      if (arduino_buffer.find('/') != std::string::npos) {
        arduino_buffer = arduino_buffer.substr(arduino_buffer.find('/'));
      } else {
        arduino_buffer = "";
      }
    }
  }
}

SparkfunOTOS::SparkfunOTOS(std::unique_ptr<io::ISerialDevice>& serialDevice)
    : m_serial_device{std::move(serialDevice)} {}

void SparkfunOTOS::init() {}

robot::subsystems::odometry::Position SparkfunOTOS::getPosition() {
  updatePosition();
  return latest_position;
}

void SparkfunOTOS::setLocalOffset(double x_offset, double y_offset,
                                  double theta_offset) {
  if (m_serial_device) {
    m_serial_device->flush();

    std::string output_string{"X:" + std::to_string(x_offset) + ";" +
                              "Y:" + std::to_string(y_offset) + ";" +
                              "H:" + std::to_string(theta_offset) + ";"};

    uint8_t output_bytes[output_string.length()];

    std::copy(output_string.begin(), output_string.end(), output_bytes);

    m_serial_device->write(output_bytes, output_string.length());
  }
}
}  // namespace driftless::hal