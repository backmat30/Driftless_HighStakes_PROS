#ifndef __PID_ELEVATOR_HPP__
#define __PID_ELEVATOR_HPP__

#include <memory>

#include "driftless/control/PID.hpp"
#include "driftless/hal/MotorGroup.hpp"
#include "driftless/io/IRotationSensor.hpp"
#include "driftless/robot/subsystems/elevator/IElevator.hpp"
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

/// @brief The namespace for elevator subsystem code
/// @author Matthew Backman
namespace elevator {

/// @brief Class representing a PID-controlled elevator
/// @author Matthew Backman
class PIDElevator : public driftless::robot::subsystems::elevator::IElevator {
 private:
  // the delay between task updates
  static constexpr uint8_t TASK_DELAY{10};

  // constantly updates the task
  static void taskLoop(void* params);

  // the delayer used by the elevator
  std::unique_ptr<driftless::rtos::IDelayer> m_delayer{};

  // the mutex used by the elevator
  std::unique_ptr<driftless::rtos::IMutex> m_mutex{};

  // the task used by the elevator
  std::unique_ptr<driftless::rtos::ITask> m_task{};

  // motors used for the elevator
  driftless::hal::MotorGroup m_motors{};

  // rotation sensor used for the elevator
  std::unique_ptr<driftless::io::IRotationSensor> m_rotation_sensor{};

  // pid controller for elevator motion
  driftless::control::PID m_pid{};

  // ratio of radians to inches moved
  double m_radians_to_inches{};

  // position of the elevator
  double m_position{};

  // whether the elevator is manually controlled
  bool manual_control{};

  bool paused{};

  double latest_voltage{};

  // update all instance specific data
  void taskUpdate();

  // update the position of the elevator
  void updatePosition();

 public:
  // initialize the elevator
  void init() override;

  // run the elevator
  void run() override;

  // set the voltage of the elevator motors
  void setVoltage(double voltage) override;

  // set the position of the elevator
  void setPosition(double position) override;

  void pause() override;

  void resume() override;

  bool isPaused() override;

  // get the position of the elevator
  double getPosition() override;

  // set the delayer
  void setDelayer(const std::unique_ptr<driftless::rtos::IDelayer>& delayer);

  // set the mutex
  void setMutex(std::unique_ptr<driftless::rtos::IMutex>& mutex);

  // set the task
  void setTask(std::unique_ptr<driftless::rtos::ITask>& task);

  // sets the motors
  void setMotors(driftless::hal::MotorGroup& motors);

  // sets the rotation sensor
  void setRotationSensor(
      std::unique_ptr<driftless::io::IRotationSensor>& rotation_sensor);

  // sets the pid controller
  void setPID(driftless::control::PID pid);

  // sets the ratio between radians and inches travelled
  void setRadiansToInches(double radians_to_inches);
};
}  // namespace elevator
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif