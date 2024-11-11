#ifndef __ARM_OPERATOR_HPP__
#define __ARM_OPERATOR_HPP__

#include "driftless/alliance/Alliance.hpp"
#include "driftless/io/IController.hpp"
#include "driftless/op_control/EControllerDigital.hpp"
#include "driftless/op_control/arm/EArmControlMode.hpp"
#include "driftless/profiles/IProfile.hpp"
#include "driftless/robot/Robot.hpp"

namespace driftless {
namespace op_control {
namespace arm {
class ArmOperator {
 private:
  // name of the arm subsystem
  static constexpr char ARM_SUBSYSTEM_NAME[]{"ARM"};

  // command to go to the neutral position
  static constexpr char GO_NEUTRAL_COMMAND_NAME[]{"GO NEUTRAL"};

  // command to go to the load position
  static constexpr char GO_LOAD_COMMAND_NAME[]{"GO LOAD"};

  // command to go to the ready position
  static constexpr char GO_READY_COMMAND_NAME[]{"GO READY"};

  // command to go to the score position
  static constexpr char GO_SCORE_COMMAND_NAME[]{"GO SCORE"};

  // command to go to the rush position
  static constexpr char GO_RUSH_COMMAND_NAME[]{"GO RUSH"};

  // state determining if the arm is at neutral position
  static constexpr char IS_NEUTRAL_STATE_NAME[]{"IS NEUTRAL"};

  // state determining if the arm is moving to the neutral position
  static constexpr char IS_GOING_NEUTRAL_STATE_NAME[]{"IS GOING NEUTRAL"};

  // state determining if the arm is at load position
  static constexpr char IS_LOAD_STATE_NAME[]{"IS LOAD"};

  // state determining if the arm is going to the load position
  static constexpr char IS_GOING_LOAD_STATE_NAME[]{"IS GOING LOAD"};

  // state determining if the arm is at the ready position
  static constexpr char IS_READY_STATE_NAME[]{"IS READY"};

  // state determining if the arm is going to the ready position
  static constexpr char IS_GOING_READY_STATE_NAME[]{"IS GOING READY"};

  // state determining if the arm is at score position
  static constexpr char IS_SCORE_STATE_NAME[]{"IS SCORE"};

  // state determining if the arm is going to the score position
  static constexpr char IS_GOING_SCORE_STATE_NAME[]{"IS GOING SCORE"};

  // state determining if the arm is in the rush position
  static constexpr char IS_RUSH_STATE_NAME[]{"IS RUSH"};

  // state determining if the arm is going to the rush position
  static constexpr char IS_GOING_RUSH_STATE_NAME[]{"IS GOING RUSH"};

  // state determining if there is a ring in the loading zone
  static constexpr char HAS_RING_STATE_NAME[]{"HAS RING"};

  // state determining the color of the ring in the loading zone, if applicable
  static constexpr char GET_HUE_STATE_NAME[]{"GET HUE"};

  // the controller used
  std::shared_ptr<driftless::io::IController> m_controller{};

  // the robot being controlled
  std::shared_ptr<driftless::robot::Robot> m_robot{};

  // determines if the robot has an alliance ring loaded
  bool hasAllianceRing(const driftless::alliance::Alliance);

  // updates the arm using split toggle
  void updateSplitToggle(EControllerDigital neutral, EControllerDigital load,
                         EControllerDigital ready, EControllerDigital score,
                         const driftless::alliance::Alliance alliance);

  // update the arm using single toggle
  void updateSmartToggle(EControllerDigital toggle, EControllerDigital rush,
                         const driftless::alliance::Alliance alliance);

 public:
  // constructs a new arm operator
  ArmOperator(const std::shared_ptr<driftless::io::IController>& controller,
              const std::shared_ptr<driftless::robot::Robot>& robot);

  // update the arm
  void update(const std::unique_ptr<driftless::profiles::IProfile>& profile,
              const driftless::alliance::Alliance alliance);
};
}  // namespace arm
}  // namespace op_control
}  // namespace driftless
#endif