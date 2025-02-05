#ifndef __PROCESS_SYSTEM_HPP__
#define __PROCESS_SYSTEM_HPP__

#include <cstdarg>
#include <map>
#include <memory>
#include <vector>

#include "driftless/processes/AProcess.hpp"

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for process management
/// @author Matthew Backman
namespace processes {

/// @brief Manages and controls various processes
/// @author Matthew Backman
class ProcessSystem {
 private:
  std::map<EProcess, std::unique_ptr<AProcess>> m_processes{};

 public:
  /// @brief Add a process to the process system
  /// @param process __std::unique_ptr<AProcess>&__ The process to add
  void addProcess(std::unique_ptr<AProcess>& process);

  /// @brief Remove a process from the process system
  /// @param process_name __EProcess__ The name of the process to remove
  /// @return __bool__ True if the process was removed, false otherwise
  bool removeProcess(EProcess process_name);

  /// @brief Pause a process
  /// @param process_name __EProcess__ The name of the process to pause
  void pause(EProcess process_name);

  /// @brief Resume a process
  /// @param process_name __EProcess__ The name of the process to resume
  void resume(EProcess process_name);

  /// @brief Pause all processes
  void pauseAll();

  /// @brief Resume all processes
  void resumeAll();

  /// @brief Initialize all processes
  void init();

  /// @brief Run all processes
  void run();

  /// @brief Send a command to a process
  /// @param process_name __EProcess__ The name of the process to send the
  /// command to
  /// @param command_name __EProcessCommand__ The command to send
  /// @param ... The arguments to the command
  void sendCommand(EProcess process_name, EProcessCommand command_name, ...);

  /// @brief Get the state of a process
  /// @param process_name __EProcess__ The name of the process to get the state
  /// of
  /// @param state_name __EProcessState__ The state to get
  /// @return __void*__ A pointer to the state
  void* getState(EProcess process_name, EProcessState state_name);
};
}  // namespace processes
}  // namespace driftless
#endif