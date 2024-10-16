#include "pvegas/robot/subsystems/odometry/InertialPositionTracker.hpp"

namespace pvegas {
namespace robot {
namespace subsystems {
namespace odometry {
void InertialPositionTracker::taskLoop(void* params) {
  InertialPositionTracker* position_tracker{
      static_cast<InertialPositionTracker*>(params)};

  while (true) {
    position_tracker->taskUpdate();
  }
}

void InertialPositionTracker::taskUpdate() {
  updatePosition();
  m_delayer->delay(TASK_DELAY);
}

void InertialPositionTracker::updatePosition() {
  if (m_mutex) {
    m_mutex->take();
  }

  double current_rotation{};
  double current_left_distance{};
  double current_right_distance{};
  uint32_t current_time{};

  // get raw data from sensors
  if (m_inertial_sensor) {
    current_rotation = m_inertial_sensor->getRotation() + m_heading_offset;
  }
  if (m_left_distance_tracker) {
    current_left_distance = m_left_distance_tracker->getDistance();
  }
  if (m_right_distance_tracker) {
    current_right_distance = m_right_distance_tracker->getDistance();
  }

  double rotation_change{current_rotation - last_heading};
  double left_linear_change{current_left_distance - last_left_distance};
  double right_linear_change{current_right_distance - last_right_distance};
  uint32_t time_change{current_time - last_time};

  double average_offset{
      (m_left_distance_tracker_offset + m_right_distance_tracker_offset) / 2.0};
  double average_linear_change{(left_linear_change + right_linear_change) /
                               2.0};
  double local_x{};
  double local_y{};
  double local_theta{};
  // find the x and y changes from the robot's POV
  if (rotation_change != 0.0) {
    // arc length = angle * radius => radius = arc length / angle
    double linear_radius{(average_linear_change / rotation_change) -
                         average_offset};
    local_x = 2 * std::cos(rotation_change / 2) * linear_radius;
    local_y = 2 * std::sin(rotation_change / 2) * linear_radius;
    local_theta = (last_heading + current_rotation) / 2;
  } else {
    // no strafe wheel, assume x is constant unless turning
    local_x = 0;
    local_y = average_linear_change;
    local_theta = current_rotation;
  }

  // translate the coordinates from robot-pov to field pov
  double global_x{(local_x * std::sin(local_theta)) +
                  (local_y * std::cos(local_theta))};
  double global_y{(local_y * std::sin(local_theta)) +
                  (local_x * std::cos(local_theta))};

  // add the global changes in position to the robot's position
  m_position.x += global_x;
  m_position.y += global_y;
  m_position.theta = current_rotation;

  // calculate velocities
  if (time_change) {
    m_position.xV = global_x / (time_change * MS_TO_SECONDS);
    m_position.yV = global_y / (time_change * MS_TO_SECONDS);
    m_position.thetaV = rotation_change / (time_change * MS_TO_SECONDS);
  }

  last_heading = current_rotation;
  last_left_distance = current_left_distance;
  last_right_distance = current_right_distance;
  last_time = current_time;

  if (m_mutex) {
    m_mutex->give();
  }
}

void InertialPositionTracker::init() {
  // initialize hardware and set latest data values
  if (m_inertial_sensor) {
    m_inertial_sensor->init();
    last_heading = m_inertial_sensor->getRotation();
  }
  if (m_left_distance_tracker) {
    m_left_distance_tracker->init();
    last_left_distance = m_left_distance_tracker->getDistance();
  }
  if (m_right_distance_tracker) {
    m_right_distance_tracker->init();
    last_right_distance = m_right_distance_tracker->getDistance();
  }
  if (m_clock) {
    last_time = m_clock->getTime();
  }
}

void InertialPositionTracker::run() {
  if (m_task) {
    m_task->start(&InertialPositionTracker::taskLoop, this);
  }
}

void InertialPositionTracker::setPosition(Position position) {
  if (m_mutex) {
    m_mutex->take();
  }
  m_position = position;
  if (m_inertial_sensor) {
    m_inertial_sensor->setRotation(position.theta);
  }
  last_heading = position.theta;
  if (m_mutex) {
    m_mutex->give();
  }
}

Position InertialPositionTracker::getPosition() {
  Position position{};
  if (m_mutex) {
    m_mutex->take();
  }
  position = m_position;
  if (m_mutex) {
    m_mutex->give();
  }
  return position;
}

void InertialPositionTracker::setX(double x) {
  if (m_mutex) {
    m_mutex->take();
  }
  m_position.x = x;
  if (m_mutex) {
    m_mutex->give();
  }
}

void InertialPositionTracker::setY(double y) {
  if (m_mutex) {
    m_mutex->take();
  }
  m_position.y = y;
  if (m_mutex) {
    m_mutex->give();
  }
}

void InertialPositionTracker::setTheta(double theta) {
  if (m_mutex) {
    m_mutex->take();
  }
  m_position.theta = theta;
  if (m_mutex) {
    m_mutex->give();
  }
}

void InertialPositionTracker::setClock(
    std::unique_ptr<pvegas::rtos::IClock>& clock) {
  m_clock = std::move(clock);
}

void InertialPositionTracker::setDelayer(
    std::unique_ptr<pvegas::rtos::IDelayer>& delayer) {
  m_delayer = std::move(delayer);
}

void InertialPositionTracker::setMutex(
    std::unique_ptr<pvegas::rtos::IMutex>& mutex) {
  m_mutex = std::move(mutex);
}

void InertialPositionTracker::setTask(
    std::unique_ptr<pvegas::rtos::ITask>& task) {
  m_task = std::move(task);
}

void InertialPositionTracker::setInertialSensor(
    std::unique_ptr<pvegas::io::IInertialSensor>& inertial_sensor) {
  m_inertial_sensor = std::move(inertial_sensor);
}

void InertialPositionTracker::setLeftDistanceTracker(
    std::unique_ptr<pvegas::io::IDistanceTracker>& left_distance_tracker) {
  m_left_distance_tracker = std::move(left_distance_tracker);
}

void InertialPositionTracker::setLeftDIstanceTrackerOffset(
    double left_distance_tracker_offset) {
  m_left_distance_tracker_offset = left_distance_tracker_offset;
}

void InertialPositionTracker::setRightDistanceTracker(
    std::unique_ptr<pvegas::io::IDistanceTracker>& right_distance_tracker) {
  m_right_distance_tracker = std::move(right_distance_tracker);
}

void InertialPositionTracker::setRightDistanceTrackerOffset(
    double right_distance_tracker_offset) {
  m_right_distance_tracker_offset = right_distance_tracker_offset;
}
}  // namespace odometry
}  // namespace subsystems
}  // namespace robot
}  // namespace pvegas