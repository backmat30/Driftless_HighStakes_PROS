#include "driftless/robot/subsystems/odometry/SparkFunPositionTrackerBuilder.hpp"

namespace driftless::robot::subsystems::odometry {
SparkFunPositionTrackerBuilder* SparkFunPositionTrackerBuilder::withClock(
    std::unique_ptr<rtos::IClock>& clock) {
  m_clock = std::move(clock);
  return this;
}

SparkFunPositionTrackerBuilder* SparkFunPositionTrackerBuilder::withDelayer(
    std::unique_ptr<rtos::IDelayer>& delayer) {
  m_delayer = std::move(delayer);
  return this;
}

SparkFunPositionTrackerBuilder* SparkFunPositionTrackerBuilder::withMutex(
    std::unique_ptr<rtos::IMutex>& mutex) {
  m_mutex = std::move(mutex);
  return this;
}

SparkFunPositionTrackerBuilder* SparkFunPositionTrackerBuilder::withTask(
    std::unique_ptr<rtos::ITask>& task) {
  m_task = std::move(task);
  return this;
}

SparkFunPositionTrackerBuilder* SparkFunPositionTrackerBuilder::withSerialDevice(
    std::unique_ptr<io::ISerialDevice>& serial_device) {
  m_serial_device = std::move(serial_device);
  return this;
}

SparkFunPositionTrackerBuilder* SparkFunPositionTrackerBuilder::withLocalXOffset(
    double local_x_offset) {
  m_local_x_offset = local_x_offset;
  return this;
}

SparkFunPositionTrackerBuilder* SparkFunPositionTrackerBuilder::withLocalYOffset(
    double local_y_offset) {
  m_local_y_offset = local_y_offset;
  return this;
}

SparkFunPositionTrackerBuilder*
SparkFunPositionTrackerBuilder::withLocalThetaOffset(double local_theta_offset) {
  m_local_theta_offset = local_theta_offset;
  return this;
}

std::unique_ptr<IPositionTracker> SparkFunPositionTrackerBuilder::build() {
  std::unique_ptr<SparkFunPositionTracker> position_tracker{
      std::make_unique<SparkFunPositionTracker>()};
  position_tracker->setClock(m_clock);
  position_tracker->setDelayer(m_delayer);
  position_tracker->setMutex(m_mutex);
  position_tracker->setTask(m_task);
  position_tracker->setSerialDevice(m_serial_device);
  position_tracker->setLocalXOffset(m_local_x_offset);
  position_tracker->setLocalYOffset(m_local_y_offset);
  position_tracker->setLocalThetaOffset(m_local_theta_offset);

  return position_tracker;
}
}  // namespace driftless::robot::subsystems::odometry