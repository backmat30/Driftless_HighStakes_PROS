#include "driftless/robot/subsystems/odometry/InertialPositionTrackerBuilder.hpp"

namespace driftless {
namespace robot {
namespace subsystems {
namespace odometry {
InertialPositionTrackerBuilder* InertialPositionTrackerBuilder::withClock(
    std::unique_ptr<driftless::rtos::IClock>& clock) {
  m_clock = std::move(clock);
  return this;
}

InertialPositionTrackerBuilder* InertialPositionTrackerBuilder::withDelayer(
    std::unique_ptr<driftless::rtos::IDelayer>& delayer) {
  m_delayer = std::move(delayer);
  return this;
}

InertialPositionTrackerBuilder* InertialPositionTrackerBuilder::withMutex(
    std::unique_ptr<driftless::rtos::IMutex>& mutex) {
  m_mutex = std::move(mutex);
  return this;
}

InertialPositionTrackerBuilder* InertialPositionTrackerBuilder::withTask(
    std::unique_ptr<driftless::rtos::ITask>& task) {
  m_task = std::move(task);
  return this;
}

InertialPositionTrackerBuilder*
InertialPositionTrackerBuilder::withInertialSensor(
    std::unique_ptr<driftless::io::IInertialSensor>& inertial_sensor) {
  m_inertial_sensor = std::move(inertial_sensor);
  return this;
}

InertialPositionTrackerBuilder*
InertialPositionTrackerBuilder::withLinearDistanceTracker(
    std::unique_ptr<driftless::io::IDistanceTracker>& linear_distance_tracker) {
  m_linear_distance_tracker = std::move(linear_distance_tracker);
  return this;
}

InertialPositionTrackerBuilder*
InertialPositionTrackerBuilder::withLinearDistanceTrackerOffset(
    double linear_distance_tracker_offset) {
  m_linear_distance_tracker_offset = linear_distance_tracker_offset;
  return this;
}

InertialPositionTrackerBuilder*
InertialPositionTrackerBuilder::withStrafeDistanceTracker(
    std::unique_ptr<driftless::io::IDistanceTracker>& strafe_distance_tracker) {
  m_strafe_distance_tracker = std::move(strafe_distance_tracker);
  return this;
}

InertialPositionTrackerBuilder*
InertialPositionTrackerBuilder::withStrafeDistanceTrackerOffset(
    double strafe_distance_tracker_offset) {
  m_strafe_distance_tracker_offset = strafe_distance_tracker_offset;
  return this;
}

std::unique_ptr<IPositionTracker>
InertialPositionTrackerBuilder::build() {
  std::unique_ptr<InertialPositionTracker> position_tracker{
      std::make_unique<InertialPositionTracker>()};
  position_tracker->setClock(m_clock);
  position_tracker->setDelayer(m_delayer);
  position_tracker->setMutex(m_mutex);
  position_tracker->setTask(m_task);
  position_tracker->setInertialSensor(m_inertial_sensor);
  position_tracker->setLinearDistanceTracker(m_linear_distance_tracker);
  position_tracker->setLinearDIstanceTrackerOffset(
      m_linear_distance_tracker_offset);
  position_tracker->setStrafeDistanceTracker(m_strafe_distance_tracker);
  position_tracker->setStrafeDistanceTrackerOffset(
      m_strafe_distance_tracker_offset);

  return position_tracker;
}
}  // namespace odometry
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless