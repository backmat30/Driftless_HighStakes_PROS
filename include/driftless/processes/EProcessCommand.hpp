#ifndef __E_PROCESS_COMMAND_HPP__
#define __E_PROCESS_COMMAND_HPP__

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for process management
/// @author Matthew Backman
namespace processes {

/// @brief Enumeration representing the possible commands for a process
/// @author Matthew Backman
enum class EProcessCommand {
  AUTO_RING_REJECTION_REJECT_RINGS,
  CLIMB_START_CLIMB
};
}  // namespace processes
}  // namespace driftless
#endif