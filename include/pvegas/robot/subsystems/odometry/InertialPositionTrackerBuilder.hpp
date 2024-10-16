#ifndef __INERTIAL_POSITION_TRACKER_BUILDER_HPP__
#define __INERTIAL_POSITION_TRACKER_BUILDER_HPP__

#include <memory>

#include "pvegas/robot/subsystems/odometry/InertialPositionTracker.hpp"

namespace pvegas {
namespace robot {
namespace subsystems {
namespace odometry {
class InertialPositionTrackerBuilder {
 private:
  // the clock used to build the position tracker
  std::unique_ptr<pvegas::rtos::IClock> m_clock{};

  // the delayer used to build the position tracker
  std::unique_ptr<pvegas::rtos::IDelayer> m_delayer{};

  // the mutex used to build the position tracker
  std::unique_ptr<pvegas::rtos::IMutex> m_mutex{};

  // the task used to build the position tracker
  std::unique_ptr<pvegas::rtos::ITask> m_task{};

  // the inertial sensor used to build the position tracker
  std::unique_ptr<pvegas::io::IInertialSensor> m_inertial_sensor{};

  // the left distance tracker used to build the position tracker
  std::unique_ptr<pvegas::io::IDistanceTracker> m_left_distance_tracker{};

  // the offset of the left distance tracker used to build the position tracker
  double m_left_distance_tracker_offset{};

  // the right distance tracker used to build the position tracker
  std::unique_ptr<pvegas::io::IDistanceTracker> m_right_distance_tracker{};

  // the offset of the right distance tracker used to build the position tracker
  double m_right_distance_tracker_offset{};

 public:
  // add a clock to the builder
  InertialPositionTrackerBuilder* withClock(
      std::unique_ptr<pvegas::rtos::IClock>& clock);

  // add a delayer to the builder
  InertialPositionTrackerBuilder* withDelayer(
      std::unique_ptr<pvegas::rtos::IDelayer>& delayer);

  // add a mutex to the builder
  InertialPositionTrackerBuilder* withMutex(
      std::unique_ptr<pvegas::rtos::IMutex>& mutex);

  // add a task to the builder
  InertialPositionTrackerBuilder* withTask(
      std::unique_ptr<pvegas::rtos::ITask>& task);

  // add an inertial sensor to the builder
  InertialPositionTrackerBuilder* withInertialSensor(
      std::unique_ptr<pvegas::io::IInertialSensor>& inertial_sensor);

  // add a left distance tracker to the builder
  InertialPositionTrackerBuilder* withLeftDistanceTracker(
      std::unique_ptr<pvegas::io::IDistanceTracker>& left_distance_tracker);

  // add a left distance tracker offset to the builder
  InertialPositionTrackerBuilder* withLeftDistanceTrackerOffset(
      double left_distance_tracker_offset);

  // add a right distance tracker to the builder
  InertialPositionTrackerBuilder* withRightDistanceTracker(
      std::unique_ptr<pvegas::io::IDistanceTracker>& right_distance_tracker);

  // add a right distance tracker offset to the builder
  InertialPositionTrackerBuilder* withRightDistanceTrackerOffset(
      double right_distance_tracker_offset);

  std::unique_ptr<InertialPositionTracker> build();
};
}  // namespace odometry
}  // namespace subsystems
}  // namespace robot
}  // namespace pvegas
#endif