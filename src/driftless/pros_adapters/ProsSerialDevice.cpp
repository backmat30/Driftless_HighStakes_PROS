#include "driftless/pros_adapters/ProsSerialDevice.hpp"

namespace driftless::pros_adapters {
ProsSerialDevice::ProsSerialDevice(std::unique_ptr<pros::Serial>& serial_device)
    : m_serial_device{std::move(serial_device)} {}

void ProsSerialDevice::initialize() { m_serial_device->flush(); }

uint8_t ProsSerialDevice::readByte() {
  uint8_t value{};

  if (m_serial_device && m_serial_device->get_read_avail()) {
    value = m_serial_device->read_byte();
  }

  return value;
}

uint8_t ProsSerialDevice::peekByte() {
  uint8_t value{};
  if (m_serial_device && m_serial_device->get_read_avail()) {
    value = m_serial_device->peek_byte();
  }

  return value;
}

void ProsSerialDevice::read(uint8_t* buffer, int length) {
  if (m_serial_device) {
    m_serial_device->read(buffer, length);
  }
}

void ProsSerialDevice::write(uint8_t* buffer, int length) {
  if (m_serial_device) m_serial_device->write(buffer, length);
}

void ProsSerialDevice::flush() {
  if (m_serial_device) m_serial_device->flush();
}

int ProsSerialDevice::getInputBytes() {
  int bytes{};

  if (m_serial_device) bytes = m_serial_device->get_read_avail();

  return bytes;
}
}  // namespace driftless::pros_adapters