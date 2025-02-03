#ifndef __INERTIAL_POSITION_TRACKER_BUILDER_HPP__
#define __INERTIAL_POSITION_TRACKER_BUILDER_HPP__

#include <memory>

#include "driftless/robot/subsystems/odometry/InertialPositionTracker.hpp"

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief The namespace for subsystems code
/// @author Matthew Backman
namespace subsystems {

/// @brief The namespace for odometry code
/// @author Matthew Backman
namespace odometry {

/// @brief Builder class for creating an InertialPositionTracker
/// @author Matthew Backman
class InertialPositionTrackerBuilder {
 private:
  // the clock used to build the position tracker
  std::unique_ptr<driftless::rtos::IClock> m_clock{};

  // the delayer used to build the position tracker
  std::unique_ptr<driftless::rtos::IDelayer> m_delayer{};

  // the mutex used to build the position tracker
  std::unique_ptr<driftless::rtos::IMutex> m_mutex{};

  // the task used to build the position tracker
  std::unique_ptr<driftless::rtos::ITask> m_task{};

  // the inertial sensor used to build the position tracker
  std::unique_ptr<driftless::io::IInertialSensor> m_inertial_sensor{};

  // the linear distance tracker used to build the position tracker
  std::unique_ptr<driftless::io::IDistanceTracker> m_linear_distance_tracker{};

  // the offset of the linear distance tracker used to build the position tracker
  double m_linear_distance_tracker_offset{};

  // the strafe distance tracker used to build the position tracker
  std::unique_ptr<driftless::io::IDistanceTracker> m_strafe_distance_tracker{};

  // the offset of the strafe distance tracker used to build the position tracker
  double m_strafe_distance_tracker_offset{};

 public:
  /// @brief Adds a clock to the builder
  /// @param clock __std::unique_ptr<driftless::rtos::IClock>&__ The clock to add
  /// @return __InertialPositionTrackerBuilder*__ The builder instance
  InertialPositionTrackerBuilder* withClock(
      std::unique_ptr<driftless::rtos::IClock>& clock);

  /// @brief Adds a delayer to the builder
  /// @param delayer __std::unique_ptr<driftless::rtos::IDelayer>&__ The delayer to add
  /// @return __InertialPositionTrackerBuilder*__ The builder instance
  InertialPositionTrackerBuilder* withDelayer(
      std::unique_ptr<driftless::rtos::IDelayer>& delayer);

  /// @brief Adds a mutex to the builder
  /// @param mutex __std::unique_ptr<driftless::rtos::IMutex>&__ The mutex to add
  /// @return __InertialPositionTrackerBuilder*__ The builder instance
  InertialPositionTrackerBuilder* withMutex(
      std::unique_ptr<driftless::rtos::IMutex>& mutex);

  /// @brief Adds a task to the builder
  /// @param task __std::unique_ptr<driftless::rtos::ITask>&__ The task to add
  /// @return __InertialPositionTrackerBuilder*__ The builder instance
  InertialPositionTrackerBuilder* withTask(
      std::unique_ptr<driftless::rtos::ITask>& task);

  /// @brief Adds an inertial sensor to the builder
  /// @param inertial_sensor __std::unique_ptr<driftless::io::IInertialSensor>&__ The inertial sensor to add
  /// @return __InertialPositionTrackerBuilder*__ The builder instance
  InertialPositionTrackerBuilder* withInertialSensor(
      std::unique_ptr<driftless::io::IInertialSensor>& inertial_sensor);

  /// @brief Adds a linear distance tracker to the builder
  /// @param linear_distance_tracker __std::unique_ptr<driftless::io::IDistanceTracker>&__ The linear distance tracker to add
  /// @return __InertialPositionTrackerBuilder*__ The builder instance
  InertialPositionTrackerBuilder* withLinearDistanceTracker(
      std::unique_ptr<driftless::io::IDistanceTracker>& linear_distance_tracker);

  /// @brief Adds a linear distance tracker offset to the builder
  /// @param linear_distance_tracker_offset __double__ The offset to add
  /// @return __InertialPositionTrackerBuilder*__ The builder instance
  InertialPositionTrackerBuilder* withLinearDistanceTrackerOffset(
      double linear_distance_tracker_offset);

  /// @brief Adds a strafe distance tracker to the builder
  /// @param strafe_distance_tracker __std::unique_ptr<driftless::io::IDistanceTracker>&__ The strafe distance tracker to add
  /// @return __InertialPositionTrackerBuilder*__ The builder instance
  InertialPositionTrackerBuilder* withStrafeDistanceTracker(
      std::unique_ptr<driftless::io::IDistanceTracker>& strafe_distance_tracker);

  /// @brief Adds a strafe distance tracker offset to the builder
  /// @param strafe_distance_tracker_offset __double__ The offset to add
  /// @return __InertialPositionTrackerBuilder*__ The builder instance
  InertialPositionTrackerBuilder* withStrafeDistanceTrackerOffset(
      double strafe_distance_tracker_offset);

  /// @brief Builds the InertialPositionTracker
  /// @return __std::unique_ptr<IPositionTracker>__ The built position tracker
  std::unique_ptr<IPositionTracker> build();
};

}  // namespace odometry
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif