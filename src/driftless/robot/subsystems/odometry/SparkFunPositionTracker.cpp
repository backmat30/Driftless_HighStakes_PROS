#include "driftless/robot/subsystems/odometry/SparkFunPositionTracker.hpp"

namespace driftless::robot::subsystems::odometry {
void SparkFunPositionTracker::taskLoop(void* params) {
  SparkFunPositionTracker* instance{
      static_cast<SparkFunPositionTracker*>(params)};

  while (true) {
    instance->taskUpdate();
  }
}

void SparkFunPositionTracker::taskUpdate() {
  uint64_t current_time{m_clock->getTime()};

  updatePosition();

  m_delayer->delayUntil(current_time + TASK_DELAY);
}

void SparkFunPositionTracker::updatePosition() {
  if (m_mutex) {
    m_mutex->take();
  }

  Position raw_position{fetchRawPosition()};
  uint32_t current_time{m_clock->getTime()};

  double x_pos{};
  double y_pos{};
  double heading{};

  x_pos = raw_position.x * std::cos(global_theta_offset) -
          raw_position.y * std::sin(global_theta_offset);
  y_pos = raw_position.x * std::sin(global_theta_offset) +
          raw_position.y * std::cos(global_theta_offset);

  x_pos += global_x_offset;
  y_pos += global_y_offset;
  heading += global_theta_offset;

  double time_change{current_time - latest_time};
  double x_change{x_pos - current_position.x};
  double y_change{y_pos - current_position.y};
  double theta_change{heading - current_position.theta};

  if (time_change) {
    current_position.xV = x_change / (time_change * 0.001);
    current_position.yV = y_change / (time_change * 0.001);
    current_position.thetaV = theta_change / (time_change * 0.001);
  }
  current_position.x = x_pos;
  current_position.y = y_pos;
  current_position.theta = heading;

  latest_time = current_time;

  if (m_mutex) {
    m_mutex->give();
  }
}

Position SparkFunPositionTracker::fetchRawPosition() {
  Position raw_position{};

  if (m_serial_device) {
    for (int i = 0; i < 3; i++) {
      InputKey current_key{static_cast<InputKey>(m_serial_device->readByte())};

      if (static_cast<char>(m_serial_device->readByte()) != ':') {
        m_serial_device->flush();
        return;
      }

      std::string value{""};

      // Turn the byte stream back into a readable string
      while (static_cast<char>(m_serial_device->peekByte()) != ';') {
        value += static_cast<char>(m_serial_device->readByte());
      }
      m_serial_device->readByte();

      switch (current_key) {
        case InputKey::GET_X:
          raw_position.x = std::stod(value);
          break;
        case InputKey::GET_Y:
          raw_position.y = std::stod(value);
          break;
        case InputKey::GET_HEADING:
          raw_position.theta = std::stod(value);
          break;
      }
    }
  }
  m_serial_device->flush();
  return raw_position;
}

void SparkFunPositionTracker::sendLocalOffset() {
  if (m_mutex) {
    m_mutex->take();
  }

  if (m_serial_device) {
    m_serial_device->flush();

    std::string output_string{"X:" + std::to_string(m_local_x_offset) + ";" +
                              "Y:" + std::to_string(m_local_y_offset) + ";" +
                              "H:" + std::to_string(m_local_theta_offset) +
                              ";"};

    uint8_t output_bytes[output_string.length()];

    std::copy(output_string.begin(), output_string.end(), output_bytes);

    m_serial_device->write(output_bytes, output_string.length());
  }

  if (m_mutex) {
    m_mutex->give();
  }
}

void SparkFunPositionTracker::init() { sendLocalOffset(); }

void SparkFunPositionTracker::run() { m_task->start(taskLoop, this); }

void SparkFunPositionTracker::setPosition(Position position) {
  setX(position.x);
  setY(position.y);
  setTheta(position.theta);
}

void SparkFunPositionTracker::setX(double x) {
  global_x_offset += x - current_position.x;
}

void SparkFunPositionTracker::setY(double y) {
  global_y_offset += y - current_position.y;
}

void SparkFunPositionTracker::setTheta(double theta) {
  global_theta_offset += theta - current_position.theta;
}

void SparkFunPositionTracker::setClock(std::unique_ptr<rtos::IClock>& clock) {
  m_clock = std::move(clock);
}

void SparkFunPositionTracker::setDelayer(
    std::unique_ptr<rtos::IDelayer>& delayer) {
  m_delayer = std::move(delayer);
}

void SparkFunPositionTracker::setMutex(std::unique_ptr<rtos::IMutex>& mutex) {
  m_mutex = std::move(mutex);
}

void SparkFunPositionTracker::setTask(std::unique_ptr<rtos::ITask>& task) {
  m_task = std::move(task);
}

void SparkFunPositionTracker::setSerialDevice(
    std::unique_ptr<io::ISerialDevice>& serial_device) {
  m_serial_device = std::move(serial_device);
}

void SparkFunPositionTracker::setLocalXOffset(double local_x_offset) {
  m_local_x_offset = local_x_offset;
}

void SparkFunPositionTracker::setLocalYOffset(double local_y_offset) {
  m_local_y_offset = local_y_offset;
}

void SparkFunPositionTracker::setLocalThetaOffset(double local_theta_offset) {
  m_local_theta_offset = local_theta_offset;
}
}  // namespace driftless::robot::subsystems::odometry