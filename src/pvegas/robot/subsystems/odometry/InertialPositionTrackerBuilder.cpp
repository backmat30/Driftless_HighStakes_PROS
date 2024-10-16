#include "pvegas/robot/subsystems/odometry/InertialPositionTrackerBuilder.hpp"

namespace pvegas {
namespace robot {
namespace subsystems {
namespace odometry {
InertialPositionTrackerBuilder* InertialPositionTrackerBuilder::withClock(
    std::unique_ptr<pvegas::rtos::IClock>& clock) {
  m_clock = std::move(clock);
  return this;
}

InertialPositionTrackerBuilder* InertialPositionTrackerBuilder::withDelayer(
    std::unique_ptr<pvegas::rtos::IDelayer>& delayer) {
  m_delayer = std::move(delayer);
  return this;
}

InertialPositionTrackerBuilder* InertialPositionTrackerBuilder::withMutex(
    std::unique_ptr<pvegas::rtos::IMutex>& mutex) {
  m_mutex = std::move(mutex);
  return this;
}

InertialPositionTrackerBuilder* InertialPositionTrackerBuilder::withTask(
    std::unique_ptr<pvegas::rtos::ITask>& task) {
  m_task = std::move(task);
  return this;
}

InertialPositionTrackerBuilder*
InertialPositionTrackerBuilder::withInertialSensor(
    std::unique_ptr<pvegas::io::IInertialSensor>& inertial_sensor) {
  m_inertial_sensor = std::move(inertial_sensor);
  return this;
}

InertialPositionTrackerBuilder*
InertialPositionTrackerBuilder::withLeftDistanceTracker(
    std::unique_ptr<pvegas::io::IDistanceTracker>& left_distance_tracker) {
  m_left_distance_tracker = std::move(left_distance_tracker);
  return this;
}

InertialPositionTrackerBuilder*
InertialPositionTrackerBuilder::withLeftDistanceTrackerOffset(
    double left_distance_tracker_offset) {
  m_left_distance_tracker_offset = left_distance_tracker_offset;
  return this;
}

InertialPositionTrackerBuilder*
InertialPositionTrackerBuilder::withRightDistanceTracker(
    std::unique_ptr<pvegas::io::IDistanceTracker>& right_distance_tracker) {
  m_right_distance_tracker = std::move(right_distance_tracker);
  return this;
}

InertialPositionTrackerBuilder*
InertialPositionTrackerBuilder::withRightDistanceTrackerOffset(
    double right_distance_tracker_offset) {
  m_right_distance_tracker_offset = right_distance_tracker_offset;
  return this;
}

std::unique_ptr<InertialPositionTracker>
InertialPositionTrackerBuilder::build() {
  std::unique_ptr<InertialPositionTracker> position_tracker{
      std::make_unique<InertialPositionTracker>()};
  position_tracker->setClock(m_clock);
  position_tracker->setDelayer(m_delayer);
  position_tracker->setMutex(m_mutex);
  position_tracker->setTask(m_task);
  position_tracker->setInertialSensor(m_inertial_sensor);
  position_tracker->setLeftDistanceTracker(m_left_distance_tracker);
  position_tracker->setLeftDIstanceTrackerOffset(
      m_left_distance_tracker_offset);
  position_tracker->setRightDistanceTracker(m_right_distance_tracker);
  position_tracker->setRightDistanceTrackerOffset(
      m_right_distance_tracker_offset);

  return position_tracker;
}
}  // namespace odometry
}  // namespace subsystems
}  // namespace robot
}  // namespace pvegas