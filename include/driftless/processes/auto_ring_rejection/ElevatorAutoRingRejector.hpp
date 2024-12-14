#ifndef __ELEVATOR_AUTO_RING_REJECTOR_HPP__
#define __ELEVATOR_AUTO_RING_REJECTOR_HPP__

#include <stdint.h>

#include "driftless/processes/auto_ring_rejection/IAutoRingRejector.hpp"
#include "driftless/rtos/IDelayer.hpp"
#include "driftless/rtos/IMutex.hpp"
#include "driftless/rtos/ITask.hpp"

namespace driftless {
namespace processes {
namespace auto_ring_rejection {
class ElevatorAutoRingRejector : public IAutoRingRejector {
 private:
  static constexpr uint8_t TASK_DELAY{10};

  /// @brief Constantly runs the task updates
  /// @param params The instance being updated
  static void taskLoop(void* params);

  std::shared_ptr<robot::Robot> m_robot{};

  std::shared_ptr<alliance::IAlliance> m_alliance{};

  bool paused{true};

  double last_opposing_ring_pos{-__DBL_MAX__};

  std::unique_ptr<rtos::IDelayer> m_delayer{};

  std::unique_ptr<rtos::IMutex> m_mutex{};

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
  /// @param active The desired position, True for deployed, false for retracted
  void setRejectorPosition(bool active);

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
  /// @param robot The robot to use
  /// @param alliance The current alliance
  void rejectRings(std::shared_ptr<robot::Robot>& robot,
                   std::shared_ptr<alliance::IAlliance>& alliance) override;

  /// @brief Sets the delayer to be used
  /// @param delayer The new delayer
  void setDelayer(std::unique_ptr<rtos::IDelayer>& delayer);

  /// @brief Sets the mutex to be used
  /// @param mutex The new mutex
  void setMutex(std::unique_ptr<rtos::IMutex>& mutex);

  /// @brief Sets the task to be used
  /// @param task The new task
  void setTask(std::unique_ptr<rtos::ITask>& task);
};
}  // namespace auto_ring_rejection
}  // namespace processes
}  // namespace driftless
#endif