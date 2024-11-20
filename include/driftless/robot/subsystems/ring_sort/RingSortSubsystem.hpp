#ifndef __RING_SORT_SUBSYSTEM_HPP__
#define __RING_SORT_SUBSYSTEM_HPP__

#include <memory>

#include "driftless/robot/subsystems/ASubsystem.hpp"
#include "driftless/robot/subsystems/ring_sort/IRingSort.hpp"

namespace driftless {
namespace robot {
namespace subsystems {
namespace ring_sort {
/// @brief Subsystem for the ring sorter
class RingSortSubsystem : public ASubsystem {
 private:
  static constexpr char RING_SORT_SUBSYSTEM_NAME[]{"RING SORT"};

  static constexpr char HAS_RING_STATE_NAME[]{"HAS RING"};

  static constexpr char GET_HUE_STATE_NAME[]{"GET HUE"};

  static constexpr char GET_RGB_STATE_NAME[]{"GET RGB"};

  static constexpr char GET_DISTANCE_TO_END_STATE_NAME[]{"GET DISTANCE TO END"};

  std::unique_ptr<IRingSort> m_ring_sort{};

 public:
  /// @brief Creates a new RingSortSubsystem
  /// @param ring_sort The ring sorter to use
  RingSortSubsystem(std::unique_ptr<IRingSort>& ring_sort);

  /// @brief Initializes the subsystem
  void init() override;

  /// @brief Runs the subsystem
  void run() override;

  /// @brief Sends a command to the subsystem
  /// @param command_name The name of the command
  /// @param args Possible arguments for the desired command
  void command(std::string command_name, va_list& args) override;

  /// @brief Gets a specified state of the subsystem
  /// @param state_name The desired state
  /// @return __void*__ Pointer to the value of the state
  void* state(std::string state_name) override;
};
}  // namespace ring_sort
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif