#ifndef __ARM_OPERATOR_HPP__
#define __ARM_OPERATOR_HPP__

#include "pvegas/alliance/Alliance.hpp"
#include "pvegas/io/IController.hpp"
#include "pvegas/op_control/EControllerDigital.hpp"
#include "pvegas/op_control/arm/EArmControlMode.hpp"
#include "pvegas/profiles/IProfile.hpp"
#include "pvegas/robot/Robot.hpp"

namespace pvegas {
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

  // command to go to the score position
  static constexpr char GO_SCORE_COMMAND_NAME[]{"GO SCORE"};

  // state determining if the arm is at neutral position
  static constexpr char IS_NEUTRAL_STATE_NAME[]{"IS NEUTRAL"};

  // state determining if the arm is at load position
  static constexpr char IS_LOAD_STATE_NAME[]{"IS LOAD"};

  // state determining if the arm is at score position
  static constexpr char IS_SCORE_STATE_NAME[]{"IS SCORE"};

  // state determining if there is a ring in the loading zone
  static constexpr char HAS_RING_STATE_NAME[]{"HAS RING"};

  // state determining the color of the ring in the loading zone, if applicable
  static constexpr char GET_HUE_STATE_NAME[]{"GET HUE"};

  // the controller used
  std::shared_ptr<pvegas::io::IController> m_controller{};

  // the robot being controlled
  std::shared_ptr<pvegas::robot::Robot> m_robot{};

  // determines if the robot has an alliance ring loaded
  bool hasAllianceRing(const pvegas::alliance::Alliance);

  // updates the arm using split toggle
  void updateSplitToggle(EControllerDigital neutral, EControllerDigital load,
                         EControllerDigital score,
                         const pvegas::alliance::Alliance alliance);

  // update the arm using single toggle
  void updateSingleToggle(EControllerDigital toggle,
                          const pvegas::alliance::Alliance alliance);

 public:
  // constructs a new arm operator
  ArmOperator(const std::shared_ptr<pvegas::io::IController>& controller,
              const std::shared_ptr<pvegas::robot::Robot>& robot);

  // update the arm
  void update(const std::unique_ptr<pvegas::profiles::IProfile>& profile,
              const pvegas::alliance::Alliance alliance);
};
}  // namespace arm
}  // namespace op_control
}  // namespace pvegas
#endif