#ifndef __ARM_OPERATOR_HPP__
#define __ARM_OPERATOR_HPP__

#include "driftless/alliance/IAlliance.hpp"
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
  // name of the blue alliance
  static constexpr char BLUE_ALLIANCE_NAME[]{"BLUE"};

  // name of the red alliance
  static constexpr char RED_ALLIANCE_NAME[]{"RED"};

  // name of the arm subsystem
  static constexpr char ARM_SUBSYSTEM_NAME[]{"ARM"};

  // name of the ring sort subsystem
  static constexpr char RING_SORT_SUBSYSTEM_NAME[]{"RING SORT"};

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

  // command to go to the alliance stake position
  static constexpr char GO_ALLIANCE_STAKE_COMMAND_NAME[]{"GO ALLIANCE STAKE"};

  // command to go to the previous position
  static constexpr char GO_PREVIOUS_COMMAND_NAME[]{"GO PREVIOUS"};

  // command to calibrate the arm
  static constexpr char CALIBRATE_COMMAND_NAME[]{"CALIBRATE"};

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

  // state determining if the arm is at the alliance stake position
  static constexpr char IS_ALLIANCE_STAKE_STATE_NAME[]{"IS ALLIANCE STAKE"};

  // state determining if the arm is going to the alliance stake position
  static constexpr char IS_GOING_ALLIANCE_STAKE_STATE_NAME[]{
      "IS GOING ALLIANCE STAKE"};

  // state determining if there is a ring in the loading zone
  static constexpr char HAS_RING_STATE_NAME[]{"HAS RING"};

  // state determining the color of the ring in the loading zone, if applicable
  static constexpr char GET_HUE_STATE_NAME[]{"GET HUE"};

  static constexpr char GET_RGB_STATE_NAME[]{"GET RGB"};

  // the controller used
  std::shared_ptr<driftless::io::IController> m_controller{};

  // the robot being controlled
  std::shared_ptr<driftless::robot::Robot> m_robot{};

  // determines if the robot has an alliance ring loaded
  bool hasAllianceRing(const std::shared_ptr<alliance::IAlliance>& alliance);

  bool hasOpposingRing(const std::shared_ptr<alliance::IAlliance>& alliance);

  // updates the arm using split toggle
  void updateSplitToggle(EControllerDigital neutral, EControllerDigital load,
                         EControllerDigital ready, EControllerDigital score,
                         const std::shared_ptr<alliance::IAlliance>& alliance);

  // update the arm using single toggle
  void updateSmartToggle(EControllerDigital toggle, EControllerDigital rush,
                         EControllerDigital calibrate,
                         EControllerDigital alliance_stake,
                         const std::shared_ptr<alliance::IAlliance>& alliance);

 public:
  // constructs a new arm operator
  ArmOperator(const std::shared_ptr<driftless::io::IController>& controller,
              const std::shared_ptr<driftless::robot::Robot>& robot);

  // update the arm
  void update(const std::unique_ptr<driftless::profiles::IProfile>& profile,
              const std::shared_ptr<alliance::IAlliance>& alliance);
};
}  // namespace arm
}  // namespace op_control
}  // namespace driftless
#endif