#ifndef __E_CONTROL_HPP__
#define __E_CONTROL_HPP__

namespace pvegas {
namespace op_control {
enum EControl {
  ARM_GRAB,
  ARM_RELEASE,
  ARM_OUT,
  ARM_IN,
  CLAMP_GRAB,
  CLAMP_RELEASE,
  CLIMB_UP,
  CLIMB_HOLD,
  CLIMB_PULL,
  CLIMB_RELEASE,
  ELEVATOR_SPIN,
  ELEVATOR_TOGGLE,
  INTAKE_SPIN,
  INTAKE_RAISE,
  INTAKE_LOWER,
  INTAKE_TOGGLE_HEIGHT,
  INTAKE_HOLD_UP
};
}  // namespace op_control
}  // namespace pvegas
#endif