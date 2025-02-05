#ifndef __ELEVATOR_AUTO_RING_REJECTOR_HPP__
#define __ELEVATOR_AUTO_RING_REJECTOR_HPP__

#include <stdint.h>

#include "driftless/processes/auto_ring_rejection/IAutoRingRejector.hpp"
#include "driftless/rtos/IDelayer.hpp"
#include "driftless/rtos/IMutex.hpp"
#include "driftless/rtos/ITask.hpp"

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for process management
/// @author Matthew Backman
namespace processes {

/// @brief Namespace for the auto ring rejection process
/// @author Matthew Backman
namespace auto_ring_rejection {

/// @brief Class representing a ring rejector using the elevator
/// @author Matthew Backman
class ElevatorAutoRingRejector : public IAutoRingRejector {
 private:
  /// @brief The task delay
  static constexpr uint8_t TASK_DELAY{10};

  /// @brief Constantly runs the task updates
  /// @param params __void*__ The instance being updated
  static void taskLoop(void* params);

  /// @brief The robot object
  std::shared_ptr<robot::Robot> m_robot{};

  /// @brief The alliance object
  std::shared_ptr<alliance::IAlliance> m_alliance{};

  /// @brief If the automatic ring rejector is paused
  bool paused{true};

  /// @brief The elevator position when the last opposing ring was detected
  double last_opposing_ring_pos{-__DBL_MAX__};

  /// @brief Whether the arm was moved out of the way of a ring or not
  bool was_arm_moved{false};

  /// @brief The Delayer used
  std::unique_ptr<rtos::IDelayer> m_delayer{};

  /// @brief The Mutex used
  std::unique_ptr<rtos::IMutex> m_mutex{};

  /// @brief The Task used
  std::unique_ptr<rtos::ITask> m_task{};

  /// @brief Updates all instance specific values
  void taskUpdate();

  /// @brief Gets the current position of the elevator
  /// @return __double__ The elevator's position
  double getElevatorPosition();

  /// @brief Gets the distance from the color sensor to the end of the elevator
  /// @return __double__ the distance from the sensor to the elevator's end
  double getElevatorDistanceToSensor();

  /// @brief Determines if the color sensor senses a ring of the opponent's
  /// color
  /// @return __bool__ True if there is an opposing ring, false otherwise
  bool hasOpposingRing();

  /// @brief Sets the position of the ring rejector
  /// @param active __bool__ The desired position, True for deployed, false for
  /// retracted
  void setRejectorPosition(bool active);

  /// @brief Moves the arm out of the way of the elevator if needed
  /// @param go_neutral __bool__ whether to go to the neutral position
  void setArmPosition(bool go_neutral);

 public:
  /// @brief Initializes the automatic ring rejector
  void init() override;

  /// @brief Runs the automatic ring rejector
  void run() override;

  /// @brief Pauses the automatic ring rejector
  void pause() override;

  /// @brief Resumes the automatic ring rejector
  void resume() override;

  /// @brief Starts the automatic ring rejector
  /// @param robot __std::shared_ptr<robot::Robot>&__ The robot to use
  /// @param alliance __std::shared_ptr<alliance::IAlliance>&__ The current
  /// alliance
  void rejectRings(std::shared_ptr<robot::Robot>& robot,
                   std::shared_ptr<alliance::IAlliance>& alliance) override;

  /// @brief Determines if the automatic ring rejector is paused
  bool isPaused() override;

  /// @brief Sets the delayer to be used
  /// @param delayer __std::unique_ptr<rtos::IDelayer>&__ The new delayer
  void setDelayer(std::unique_ptr<rtos::IDelayer>& delayer);

  /// @brief Sets the mutex to be used
  /// @param mutex __std::unique_ptr<rtos::IMutex>&__ The new mutex
  void setMutex(std::unique_ptr<rtos::IMutex>& mutex);

  /// @brief Sets the task to be used
  /// @param task __std::unique_ptr<rtos::ITask>&__ The new task
  void setTask(std::unique_ptr<rtos::ITask>& task);
};
}  // namespace auto_ring_rejection
}  // namespace processes
}  // namespace driftless
#endif