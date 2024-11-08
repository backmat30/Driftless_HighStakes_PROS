#include "driftless/pros_adapters/ProsController.hpp"
namespace driftless {
namespace pros_adapters {

void ProsController::taskLoop(void *params) {
  // define the controller
  ProsController *controller{static_cast<ProsController *>(params)};

  // constantly update the controller
  while (true) {
    controller->taskUpdate();
    pros::delay(TASK_DELAY);
  }
}

void ProsController::updateRumble() {
  // get current time in ms
  uint32_t time{pros::millis()};
  // checks if a new rumble pattern is requested, then if the delay has been
  // long enough
  if (new_rumble_pattern && time - last_rumble_refresh >= RUMBLE_REFRESH_RATE) {
    // ensures controller is valid before call
    if (m_controller) {
      m_controller->rumble(rumble_pattern);
    }
    // resets rumble pattern
    new_rumble_pattern = false;
    // resets last rumble refresh to the current rumble
    last_rumble_refresh = time;
  }
}

void ProsController::taskUpdate() {
  // gives the controller priority as to not disturb other processes
  mutex.take();
  updateRumble();
  mutex.give();
}

ProsController::ProsController(std::unique_ptr<pros::Controller> &controller)
    : m_controller{std::move(controller)} {}

void ProsController::init() {}

void ProsController::run() {
  pros::Task controller_task{&ProsController::taskLoop, this};
}

double ProsController::getAnalog(op_control::EControllerAnalog channel) {
  double value{};

  if (ANALOGUE_MAP.contains(channel)) {
    if (m_controller) {
      value = m_controller->get_analog(ANALOGUE_MAP.at(channel));
    }
  }
  return value;
}

bool ProsController::getDigital(op_control::EControllerDigital channel) {
  bool value{};

  if (DIGITAL_MAP.contains(channel)) {
    if (m_controller) {
      value = m_controller->get_digital(DIGITAL_MAP.at(channel));
    }
  }
  return value;
}

bool ProsController::getNewDigital(op_control::EControllerDigital channel) {
  bool value{};

  if (DIGITAL_MAP.contains(channel)) {
    if (m_controller) {
      value = m_controller->get_digital_new_press(DIGITAL_MAP.at(channel));
    }
  }
  return value;
}

void ProsController::rumble(std::string pattern) {
  mutex.take();
  for (uint8_t i{0}; i < MAX_RUMBLE_LENGTH; ++i) {
    rumble_pattern[i] = pattern[i];
  }
  new_rumble_pattern = true;
  mutex.give();
}  // namespace pros_controller
}  // namespace pros_adapters
}  // namespace driftless