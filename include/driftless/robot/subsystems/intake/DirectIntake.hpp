#ifndef __DIRECT_INTAKE_HPP__
#define __DIRECT_INTAKE_HPP__

#include <memory>

#include "driftless/hal/MotorGroup.hpp"
#include "driftless/robot/subsystems/intake/IIntake.hpp"
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

/// @brief The namespace for the intake subsystem
/// @author Matthew Backman
namespace intake {

/// @brief Class for controlling the intake mechanism directly using motors
/// @author Matthew Backman
class DirectIntake : public IIntake {
 private:
  static constexpr uint8_t TASK_DELAY{10};

  /// @brief Periodically updates the designated intake
  /// @param params __void*__ The intake to update
  static void taskLoop(void* params);

  // group of motors used to run the intake
  driftless::hal::MotorGroup m_motors{};

  std::unique_ptr<rtos::IClock> m_clock{};

  std::unique_ptr<rtos::IDelayer> m_delayer{};

  std::unique_ptr<rtos::IMutex> m_mutex{};

  std::unique_ptr<rtos::ITask> m_task{};

  double target_voltage{};

  uint32_t latest_jam_time{};

  bool jammed{};

  /// @brief Updates the intake
  void taskUpdate();

  /// @brief Unjams the intake
  void unjam();

 public:
  /// @brief Initializes the intake
  void init() override;

  /// @brief Runs the intake
  void run() override;

  /// @brief Sets the voltage of the intake motors
  /// @param voltage __double__ The voltage to set
  void setVoltage(double voltage) override;

  /// @brief Sets the motors
  /// @param motors __driftless::hal::MotorGroup&__ The motors to set
  void setMotors(driftless::hal::MotorGroup& motors);

  /// @brief Sets the clock used by the intake
  /// @param clock __const std::unique_ptr<rtos::IClock>&__ The clock to use
  void setClock(const std::unique_ptr<rtos::IClock>& clock);

  /// @brief Sets the delayer used by the intake
  /// @param delayer __const std::unique_ptr<rtos::IDelayer>&__ The delayer to use
  void setDelayer(const std::unique_ptr<rtos::IDelayer>& delayer);

  /// @brief Sets the mutex used by the intake
  /// @param mutex __std::unique_ptr<rtos::IMutex>&__ The mutex to use
  void setMutex(std::unique_ptr<rtos::IMutex>& mutex);

  /// @brief Sets the task used by the intake
  /// @param task __std::unique_ptr<rtos::ITask>&__ The task to use
  void setTask(std::unique_ptr<rtos::ITask>& task);
};
}  // namespace intake
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif