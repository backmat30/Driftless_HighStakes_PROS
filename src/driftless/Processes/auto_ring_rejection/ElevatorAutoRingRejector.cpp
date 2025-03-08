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
  if (m_robot && !paused) {
    double elevator_pos{getElevatorPosition()};
    double elevator_distance_to_sensor{getElevatorDistanceToSensor()};
    bool has_opposing_ring{hasOpposingRing()};

    if (has_opposing_ring && !seen_opposing_ring) {
      seen_opposing_ring = true;
      last_opposing_ring_pos = elevator_pos;
      setArmPosition(true);
    }
    if(!has_opposing_ring) {
      seen_opposing_ring = false;
    }

    if (elevator_pos >= last_opposing_ring_pos + 0.6 &&
        elevator_pos < last_opposing_ring_pos + elevator_distance_to_sensor && !rejecting_ring) {
      setRejectorPosition(true);
      rejecting_ring = true;
    } else if (elevator_pos < last_opposing_ring_pos - 0.25 ||
               elevator_pos >
                   last_opposing_ring_pos + elevator_distance_to_sensor && rejecting_ring) {
      setRejectorPosition(false);
      setArmPosition(false);
      rejecting_ring = false;
      last_opposing_ring_pos = -__DBL_MAX__;
    }
  }
  if (m_mutex) {
    m_mutex->give();
  }
  m_delayer->delay(TASK_DELAY);
}

double ElevatorAutoRingRejector::getElevatorPosition() {
  double elevator_position{*static_cast<double*>(m_robot->getState(
      robot::subsystems::ESubsystem::ELEVATOR,
      robot::subsystems::ESubsystemState::ELEVATOR_GET_POSITION))};
  return elevator_position;
}

double ElevatorAutoRingRejector::getElevatorDistanceToSensor() {
  double distance_to_sensor{*static_cast<double*>(m_robot->getState(
      robot::subsystems::ESubsystem::RING_SORT,
      robot::subsystems::ESubsystemState::RING_SORT_GET_DISTANCE_TO_END))};

  return distance_to_sensor;
}

bool ElevatorAutoRingRejector::hasOpposingRing() {
  bool has_opposing_ring{};

  void* has_ring_state{m_robot->getState(
      robot::subsystems::ESubsystem::RING_SORT,
      robot::subsystems::ESubsystemState::RING_SORT_HAS_RING)};
  bool has_ring{has_ring_state != nullptr &&
                *static_cast<bool*>(has_ring_state)};

  void* ring_rgb_state{
      m_robot->getState(robot::subsystems::ESubsystem::RING_SORT,
                        robot::subsystems::ESubsystemState::RING_SORT_GET_RGB)};
  io::RGBValue ring_rgb{*static_cast<io::RGBValue*>(ring_rgb_state)};

  if (has_ring) {
    has_opposing_ring =
        ((m_alliance->getAlliance() == alliance::EAlliance::RED &&
          ring_rgb.red < ring_rgb.blue) ||
         (m_alliance->getAlliance() == alliance::EAlliance::BLUE &&
          ring_rgb.blue < ring_rgb.red));
  }

  return has_opposing_ring;
}

void ElevatorAutoRingRejector::setRejectorPosition(bool active) {
  if (active) {
    m_robot->sendCommand(
        robot::subsystems::ESubsystem::ELEVATOR,
        robot::subsystems::ESubsystemCommand::ELEVATOR_DEPLOY_REJECTOR);
  } else {
    m_robot->sendCommand(
        robot::subsystems::ESubsystem::ELEVATOR,
        robot::subsystems::ESubsystemCommand::ELEVATOR_RETRACT_REJECTOR);
  }
}

void ElevatorAutoRingRejector::setArmPosition(bool go_neutral) {
  bool is_load{*static_cast<bool*>(
      m_robot->getState(robot::subsystems::ESubsystem::ARM,
                        robot::subsystems::ESubsystemState::ARM_IS_LOAD))};

  if (go_neutral && is_load) {
    m_robot->sendCommand(robot::subsystems::ESubsystem::ARM,
                         robot::subsystems::ESubsystemCommand::ARM_GO_NEUTRAL);
    was_arm_moved = true;
  } else if (was_arm_moved && !go_neutral) {
    m_robot->sendCommand(robot::subsystems::ESubsystem::ARM,
                         robot::subsystems::ESubsystemCommand::ARM_GO_LOAD);
    was_arm_moved = false;
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

bool ElevatorAutoRingRejector::isPaused() { return paused; }

void ElevatorAutoRingRejector::setDelayer(
    std::unique_ptr<rtos::IDelayer>& delayer) {
  m_delayer = std::move(delayer);
}

void ElevatorAutoRingRejector::setMutex(std::unique_ptr<rtos::IMutex>& mutex) {
  m_mutex = std::move(mutex);
}

void ElevatorAutoRingRejector::setTask(std::unique_ptr<rtos::ITask>& task) {
  m_task = std::move(task);
}
}  // namespace auto_ring_rejection
}  // namespace processes
}  // namespace driftless