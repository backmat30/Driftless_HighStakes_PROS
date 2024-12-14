#include "driftless/processes/auto_ring_rejection/ElevatorAutoRingRejector.hpp"

namespace driftless {
namespace processes {
namespace auto_ring_rejection {
void ElevatorAutoRingRejector::taskLoop(void* params) {
  ElevatorAutoRingRejector* instance{
      static_cast<ElevatorAutoRingRejector*>(params)};

  while (true) {
    instance->taskUpdate();
  }
}

void ElevatorAutoRingRejector::taskUpdate() {
  if (m_mutex) {
    m_mutex->take();
  }
  if (!paused) {
    double elevator_pos{getElevatorPosition()};
    double elevator_distance_to_sensor{getElevatorDistanceToSensor()};
    bool has_opposing_ring{hasOpposingRing()};

    if (has_opposing_ring) {
      last_opposing_ring_pos = elevator_pos;
    }

    if (elevator_pos > last_opposing_ring_pos + 0.25 &&
        elevator_pos < last_opposing_ring_pos + elevator_distance_to_sensor) {
      setRejectorPosition(true);
    } else {
      setRejectorPosition(false);
      last_opposing_ring_pos = -__DBL_MAX__;
    }
  }
  if (m_mutex) {
    m_mutex->give();
  }
  m_delayer->delay(TASK_DELAY);
}

double ElevatorAutoRingRejector::getElevatorPosition() {
  double elevator_position{
      *static_cast<double*>(m_robot->getState("ELEVATOR", "GET POSITION"))};
  return elevator_position;
}

double ElevatorAutoRingRejector::getElevatorDistanceToSensor() {
  double distance_to_sensor{*static_cast<double*>(
      m_robot->getState("RING SORT", "GET DISTANCE TO END"))};

  return distance_to_sensor;
}

bool ElevatorAutoRingRejector::hasOpposingRing() {
  bool has_opposing_ring{};

  void* has_ring_state{m_robot->getState("RING_SORT", "HAS RING")};
  bool has_ring{has_ring_state != nullptr &&
                *static_cast<bool*>(has_ring_state)};

  void* ring_rgb_state{m_robot->getState("RING SORT", "GET RGB")};
  io::RGBValue ring_rgb{*static_cast<io::RGBValue*>(ring_rgb_state)};

  if (has_ring) {
    has_opposing_ring =
        ((m_alliance->getName() == "RED" && ring_rgb.red < ring_rgb.blue) ||
         (m_alliance->getName() == "BLUE" && ring_rgb.blue < ring_rgb.red));
  }

  return has_opposing_ring;
}

void ElevatorAutoRingRejector::setRejectorPosition(bool active) {
  if (active) {
    m_robot->sendCommand("ELEVATOR", "DEPLOY REJECTOR");
  } else {
    m_robot->sendCommand("ELEVATOR", "RETRACT REJECTOR");
  }
}

void ElevatorAutoRingRejector::init() {}

void ElevatorAutoRingRejector::run() { m_task->start(taskLoop, this); }

void ElevatorAutoRingRejector::pause() {
  if (m_mutex) {
    m_mutex->take();
  }
  paused = true;
  if (m_mutex) {
    m_mutex->give();
  }
}

void ElevatorAutoRingRejector::resume() {
  if (m_mutex) {
    m_mutex->take();
  }
  paused = false;
  if (m_mutex) {
    m_mutex->give();
  }
}

void ElevatorAutoRingRejector::rejectRings(
    std::shared_ptr<robot::Robot>& robot,
    std::shared_ptr<alliance::IAlliance>& alliance) {
  if (m_mutex) {
    m_mutex->take();
  }
  m_robot = robot;
  m_alliance = alliance;
  paused = false;

  if (m_mutex) {
    m_mutex->give();
  }
}

void ElevatorAutoRingRejector::setDelayer(
    std::unique_ptr<rtos::IDelayer>& delayer) {
  m_delayer = std::move(delayer);
}

void ElevatorAutoRingRejector::setMutex(
    std::unique_ptr<rtos::IMutex>& mutex) {
  m_mutex = std::move(mutex);
}

void ElevatorAutoRingRejector::setTask(
    std::unique_ptr<rtos::ITask>& task) {
  m_task = std::move(task);
}
}  // namespace auto_ring_rejection
}  // namespace processes
}  // namespace driftless