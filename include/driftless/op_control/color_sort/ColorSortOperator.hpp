#ifndef __COLOR_SORT_OPERATOR_HPP__
#define __COLOR_SORT_OPERATOR_HPP__

#include "driftless/alliance/IAlliance.hpp"
#include "driftless/io/IController.hpp"
#include "driftless/op_control/EControllerAnalog.hpp"
#include "driftless/op_control/EControllerDigital.hpp"
#include "driftless/processes/EProcess.hpp"
#include "driftless/processes/EProcessCommand.hpp"
#include "driftless/processes/EProcessState.hpp"
#include "driftless/processes/ProcessSystem.hpp"
#include "driftless/profiles/IProfile.hpp"

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for operator control management
/// @author Matthew Backman
namespace op_control {

/// @brief Namespace for color sort control
/// @author Matthew Backman
namespace color_sort {

/// @brief Class to represent color sort control
/// @author Matthew Backman
class ColorSortOperator {
 private:
  std::shared_ptr<io::IController> m_controller{};

  std::shared_ptr<processes::ProcessSystem> m_process_system{};

 public:
  /// @brief Constructs a new ColorSortOperator object
  /// @param controller __std::shared_ptr<io::IController>&__ The controller
  /// used
  /// @param process_system __std::shared_ptr<processes::ProcessSystem>&__ The
  /// process system used
  ColorSortOperator(
      const std::shared_ptr<io::IController>& controller,
      const std::shared_ptr<processes::ProcessSystem>& process_system);

  /// @brief Updates the ring rejection process
  /// @param profile __const std::unique_ptr<profiles::IProfile>&__ The profile
  /// for control mapping
  /// @param alliance __const std::shared_ptr<alliance::IAlliance>&__ The
  /// current alliance
  void updateRingRejection(
      const std::unique_ptr<profiles::IProfile>& profile,
      const std::shared_ptr<alliance::IAlliance>& alliance);
};
}  // namespace color_sort
}  // namespace op_control
}  // namespace driftless
#endif