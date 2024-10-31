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

  // the linear distance tracker used to build the position tracker
  std::unique_ptr<pvegas::io::IDistanceTracker> m_linear_distance_tracker{};

  // the offset of the linear distance tracker used to build the position tracker
  double m_linear_distance_tracker_offset{};

  // the strafe distance tracker used to build the position tracker
  std::unique_ptr<pvegas::io::IDistanceTracker> m_strafe_distance_tracker{};

  // the offset of the strafe distance tracker used to build the position tracker
  double m_strafe_distance_tracker_offset{};

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

  // add a linear distance tracker to the builder
  InertialPositionTrackerBuilder* withLinearDistanceTracker(
      std::unique_ptr<pvegas::io::IDistanceTracker>& linear_distance_tracker);

  // add a linear distance tracker offset to the builder
  InertialPositionTrackerBuilder* withLinearDistanceTrackerOffset(
      double linear_distance_tracker_offset);

  // add a strafe distance tracker to the builder
  InertialPositionTrackerBuilder* withStrafeDistanceTracker(
      std::unique_ptr<pvegas::io::IDistanceTracker>& strafe_distance_tracker);

  // add a strafe distance tracker offset to the builder
  InertialPositionTrackerBuilder* withStrafeDistanceTrackerOffset(
      double strafe_distance_tracker_offset);

  std::unique_ptr<IPositionTracker> build();
};
}  // namespace odometry
}  // namespace subsystems
}  // namespace robot
}  // namespace pvegas
#endif