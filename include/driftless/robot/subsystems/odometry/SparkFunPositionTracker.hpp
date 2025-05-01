#ifndef __SPARK_FUN_POSITION_TRACKER_HPP__
#define __SPARK_FUN_POSITION_TRACKER_HPP__

#include <cmath>
#include <cstring>
#include <memory>
#include <string>

#include "driftless/io/ISerialDevice.hpp"
#include "driftless/robot/subsystems/odometry/IPositionTracker.hpp"
#include "driftless/robot/subsystems/odometry/Position.hpp"
#include "driftless/rtos/IClock.hpp"
#include "driftless/rtos/IDelayer.hpp"
#include "driftless/rtos/IMutex.hpp"
#include "driftless/rtos/ITask.hpp"
#include "driftless/io/IPositionSensor.hpp"

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief Namespace for subsystems code
/// @author Matthew Backman
namespace subsystems {

/// @brief The namespace for odometry code
/// @author Matthew Backman
namespace odometry {

/// @brief Class representing a position tracker using the SparkFun Odometry
/// sensor
class SparkFunPositionTracker : public IPositionTracker {
 private:
  static constexpr uint8_t TASK_DELAY{10};

  /// @brief Task loop to update the given position tracker
  /// @param params __void*__ The position tracker being updated
  static void taskLoop(void* params);

  std::unique_ptr<rtos::IClock> m_clock{};

  std::unique_ptr<rtos::IDelayer> m_delayer{};

  std::unique_ptr<rtos::IMutex> m_mutex{};

  std::unique_ptr<rtos::ITask> m_task{};

  std::unique_ptr<io::IPositionSensor> m_position_sensor{};

  double m_local_x_offset{};
  double m_local_y_offset{};
  double m_local_theta_offset{};

  double global_x_offset{};
  double global_y_offset{};
  double global_theta_offset{};

  Position current_position{};

  uint32_t latest_time{};

  /// @brief Runs all updates related to the position tracker
  void taskUpdate();

  /// @brief Updates the internal position
  void updatePosition();

  /// @brief Gathers the raw position from the odometry sensor
  /// @return __Position__ The position the sensor thinks it's at
  Position fetchRawPosition();

  /// @brief Sends the local offsets to the arduino
  void sendLocalOffset();

 public:
  /// @brief Initializes the position tracker
  void init() override;

  /// @brief Runs the position tracker
  void run() override;

  /// @brief Sets the global position of the position tracker
  /// @param position __Position__ The new position
  void setPosition(Position position) override;

  /// @brief Gets the current position of the position tracker
  /// @return __Position__ The current position
  Position getPosition() override;

  /// @brief Sets the x position of the position tracker
  /// @param x __double__ The new x position
  void setX(double x) override;

  /// @brief Sets the y position of the position tracker
  /// @param y __double__ The new y position
  void setY(double y) override;

  /// @brief Sets the heading of the position tracker
  /// @param theta __double__ The new heading
  void setTheta(double theta) override;

  /// @brief Sets the clock
  /// @param clock __std::unique_ptr<rtos::IClock>&__ The clock to
  /// set
  void setClock(std::unique_ptr<rtos::IClock>& clock);

  /// @brief Sets the delayer
  /// @param delayer __std::unique_ptr<rtos::IDelayer>&__ The delayer
  /// to set
  void setDelayer(std::unique_ptr<rtos::IDelayer>& delayer);

  /// @brief Sets the mutex
  /// @param mutex __std::unique_ptr<rtos::IMutex>&__ The mutex to
  /// set
  void setMutex(std::unique_ptr<rtos::IMutex>& mutex);

  /// @brief Sets the task
  /// @param task __std::unique_ptr<rtos::ITask>&__ The task to set
  void setTask(std::unique_ptr<rtos::ITask>& task);

  /// @brief Sets the position sensor
  /// @param position_sensor __std::unique_ptr<io::IPositionSensor>&__ The
  /// position sensor to set
  void setPositionSensor(std::unique_ptr<io::IPositionSensor>& position_sensor);

  /// @brief Sets the local X offset of the sensor
  /// @param local_x_offset __double__ The local x offset
  void setLocalXOffset(double local_x_offset);

  /// @brief Sets the local Y offset of the sensor
  /// @param local_y_offset __double__ The local y offset
  void setLocalYOffset(double local_y_offset);

  /// @brief Sets the local heading offset of the sensor
  /// @param local_theta_offset __double__ The local heading offset
  void setLocalThetaOffset(double local_theta_offset);
};
}  // namespace odometry
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless

#endif