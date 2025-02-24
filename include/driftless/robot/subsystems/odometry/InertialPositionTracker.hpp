#ifndef __INERTIAL_POSITION_TRACKER_HPP__
#define __INERTIAL_POSITION_TRACKER_HPP__

#include <cmath>
#include <memory>

#include "driftless/io/IDistanceTracker.hpp"
#include "driftless/io/IInertialSensor.hpp"
#include "driftless/robot/subsystems/odometry/IPositionTracker.hpp"
#include "driftless/rtos/IClock.hpp"
#include "driftless/rtos/IDelayer.hpp"
#include "driftless/rtos/IMutex.hpp"
#include "driftless/rtos/ITask.hpp"

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

/// @brief Class for tracking the position of the robot using inertial sensors
/// @author Matthew Backman
class InertialPositionTracker : public IPositionTracker {
 private:
  // the delay between task loops
  static constexpr uint8_t TASK_DELAY{10};

  // conversion factor between milliseconds and seconds
  static constexpr double MS_TO_SECONDS{1.0 / 1000.0};

  // task loop to update provided tracker
  static void taskLoop(void* params);

  // clock object
  std::unique_ptr<driftless::rtos::IClock> m_clock{};

  // task delayer
  std::unique_ptr<driftless::rtos::IDelayer> m_delayer{};

  // mutex
  std::unique_ptr<driftless::rtos::IMutex> m_mutex{};

  // task for updates
  std::unique_ptr<driftless::rtos::ITask> m_task{};

  // inertial sensor to track rotation
  std::unique_ptr<driftless::io::IInertialSensor> m_inertial_sensor{};

  // distance tracker on the linear side of the robot
  std::unique_ptr<driftless::io::IDistanceTracker> m_linear_distance_tracker{};

  // the offset, from the linear side of the robot, of the linear distance tracker
  double m_linear_distance_tracker_offset{};

  // distance tracker on the strafe side of the robot
  std::unique_ptr<driftless::io::IDistanceTracker> m_strafe_distance_tracker{};

  // the offset, from the linear side of the robot, of the strafe distance tracker
  double m_strafe_distance_tracker_offset{};

  // the position on the field
  Position m_position{};

  // the heading offset
  double m_heading_offset{};

  // the latest heading value
  double last_heading{};

  // the latest distance recorded on the linear tracking wheel
  double last_linear_distance{};

  // the latest distance recorded on the strafe tracking wheel
  double last_strafe_distance{};

  // the system time of the latest update
  uint8_t last_time{};

  // run all updates in the system
  void taskUpdate();

  // update the position of the system
  void updatePosition();

 public:
  /// @brief Initializes the system
  void init() override;

  /// @brief Runs the system
  void run() override;

  /// @brief Sets the position of the system
  /// @param position __Position__ The position to set
  void setPosition(Position position) override;

  /// @brief Gets the position of the system
  /// @return __Position__ The current position of the system
  Position getPosition() override;

  /// @brief Sets the x position
  /// @param x __double__ The x position to set
  void setX(double x) override;

  /// @brief Sets the y position
  /// @param y __double__ The y position to set
  void setY(double y) override;

  /// @brief Sets the angular offset of the system
  /// @param theta __double__ The angular offset to set
  void setTheta(double theta);

  /// @brief Sets the clock
  /// @param clock __std::unique_ptr<driftless::rtos::IClock>&__ The clock to set
  void setClock(std::unique_ptr<driftless::rtos::IClock>& clock);

  /// @brief Sets the delayer
  /// @param delayer __std::unique_ptr<driftless::rtos::IDelayer>&__ The delayer to set
  void setDelayer(std::unique_ptr<driftless::rtos::IDelayer>& delayer);

  /// @brief Sets the mutex
  /// @param mutex __std::unique_ptr<driftless::rtos::IMutex>&__ The mutex to set
  void setMutex(std::unique_ptr<driftless::rtos::IMutex>& mutex);

  /// @brief Sets the task
  /// @param task __std::unique_ptr<driftless::rtos::ITask>&__ The task to set
  void setTask(std::unique_ptr<driftless::rtos::ITask>& task);

  /// @brief Sets the inertial sensor
  /// @param inertial_sensor __std::unique_ptr<driftless::io::IInertialSensor>&__ The inertial sensor to set
  void setInertialSensor(
      std::unique_ptr<driftless::io::IInertialSensor>& inertial_sensor);

  /// @brief Sets the linear distance tracker
  /// @param linear_distance_tracker __std::unique_ptr<driftless::io::IDistanceTracker>&__ The linear distance tracker to set
  void setLinearDistanceTracker(
      std::unique_ptr<driftless::io::IDistanceTracker>& linear_distance_tracker);

  /// @brief Sets the offset of the linear distance tracker
  /// @param linear_distance_tracker_offset __double__ The offset to set
  void setLinearDIstanceTrackerOffset(double linear_distance_tracker_offset);

  /// @brief Sets the strafe distance tracker
  /// @param strafe_distance_tracker __std::unique_ptr<driftless::io::IDistanceTracker>&__ The strafe distance tracker to set
  void setStrafeDistanceTracker(
      std::unique_ptr<driftless::io::IDistanceTracker>& strafe_distance_tracker);

  /// @brief Sets the offset of the strafe distance tracker
  /// @param strafe_distance_tracker_offset __double__ The offset to set
  void setStrafeDistanceTrackerOffset(double strafe_distance_tracker_offset);
};

}  // namespace odometry
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif