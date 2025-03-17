#ifndef __SPARK_FUN_POSITION_TRACKER_BUILDER_HPP__
#define __SPARK_FUN_POSITION_TRACKER_BUILDER_HPP__

#include "driftless/robot/subsystems/odometry/SparkFunPositionTracker.hpp"

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief Namespace for subsystems code
/// @author Matthew Backman
namespace subsystems {

/// @brief Namespace for odometry code
/// @author Matthew Backman
namespace odometry {

/// @brief Builder class for the SparkFun position tracker
/// @author Matthew Backman
class SparkFunPositionTrackerBuilder {
 private:
  std::unique_ptr<rtos::IClock> m_clock{};

  std::unique_ptr<rtos::IDelayer> m_delayer{};

  std::unique_ptr<rtos::IMutex> m_mutex{};

  std::unique_ptr<rtos::ITask> m_task{};

  std::unique_ptr<io::IPositionSensor> m_position_sensor{};

  double m_local_x_offset{};
  double m_local_y_offset{};
  double m_local_theta_offset{};
 
 public:
  /// @brief Adds a clock to the builder
  /// @param clock __std::unique_ptr<rtos::IClock>&__ The clock to add
  /// @return __SparkFunPositionTrackerBuilder*__ Pointer to the current builder
  SparkFunPositionTrackerBuilder* withClock(std::unique_ptr<rtos::IClock>& clock);

  /// @brief Adds a delayer to the builder
  /// @param delayer __std::unique_ptr<rtos::IDelayer>&__ The delayer to add
  /// @return __SparkFunPositionTrackerBuilder*__ Pointer to the current builder
  SparkFunPositionTrackerBuilder* withDelayer(std::unique_ptr<rtos::IDelayer>& delayer);

  /// @brief Adds a mutex to the builder
  /// @param mutex __std::unique_ptr<rtos::IMutex>&__ The mutex to add
  /// @return __SparkFunPositionTrackerBuilder*__ Pointer to the current builder
  SparkFunPositionTrackerBuilder* withMutex(std::unique_ptr<rtos::IMutex>& mutex);

  /// @brief Adds a task to the builder
  /// @param task __std::unique_ptr<rtos::ITask>&__ The task to add
  /// @return __SparkFunPositionTrackerBuilder*__ Pointer to the current builder
  SparkFunPositionTrackerBuilder* withTask(std::unique_ptr<rtos::ITask>& task);

  /// @brief Adds a serial device to the builder
  /// @param serial_device __std::unique_ptr<io::IPositionSensor>&__ The position sensor to add
  /// @return __SparkFunPositionTrackerBuilder*__ Pointer to the current builder
  SparkFunPositionTrackerBuilder* withPositionSensor(std::unique_ptr<io::IPositionSensor>& position_sensor);

  /// @brief Adds a local x offset to the builder
  /// @param local_x_offset __double__ The local x offset to add
  /// @return __SparkFunPositionTrackerBuilder*__ Pointer to the current builder
  SparkFunPositionTrackerBuilder* withLocalXOffset(double local_x_offset);

  /// @brief Adds a local y offset to the builder
  /// @param local_y_offset __double__ The local y offset to add
  /// @return __SparkFunPositionTrackerBuilder*__ Pointer to the current builder
  SparkFunPositionTrackerBuilder* withLocalYOffset(double local_y_offset);

  /// @brief Adds a local theta offset to the builder
  /// @param local_theta_offset __double__ The local theta offset to add
  /// @return __SparkFunPositionTrackerBuilder*__ Pointer to the current builder
  SparkFunPositionTrackerBuilder* withLocalThetaOffset(double local_theta_offset);

  /// @brief Builds a new SparkFun position tracker
  /// @return __std::unique_ptr<IPositionTracker>__ A pointer to the new SparkFun position tracker
  std::unique_ptr<IPositionTracker> build();
};
}  // namespace odometry
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif