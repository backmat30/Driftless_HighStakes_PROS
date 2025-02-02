#ifndef __A_PROCESS_HPP__
#define __A_PROCESS_HPP__

#include <string>

#include "driftless/processes/EProcess.hpp"
#include "driftless/processes/EProcessCommand.hpp"
#include "driftless/processes/EProcessState.hpp"

/// @brief Namespace for all driftless library code
namespace driftless {
/// @brief Namespace for all background processes between subsystems
namespace processes {
/// @brief Abstract class representing a basic process
class AProcess {
 private:
  /// @brief The name of the process
  EProcess m_name{};

 public:
  /// @brief Constructs a new process object
  AProcess() = default;

  /// @brief Constructs a new process object by copying another process object
  /// @param other The process being copied
  AProcess(const AProcess& other) = default;

  /// @brief Constructs a new process object by moving another process object
  AProcess(AProcess&& other) = default;

  /// @brief Constructs a new process object with a given name
  /// @param name The name of the new process
  AProcess(EProcess name) : m_name{name} {}

  /// @brief Destroys the current process object
  virtual ~AProcess() = default;

  /// @brief Gets the name of the process
  /// @return The process name
  const EProcess &getName() const { return m_name; }

  /// @brief Initializes the process
  virtual void init() = 0;

  /// @brief Runs the process
  virtual void run() = 0;

  /// @brief Pauses the process
  virtual void pause() = 0;

  /// @brief Resumes the process
  virtual void resume() = 0;

  /// @brief Sends a command to the process
  /// @param command_name The name of the command
  /// @param args Arguments for the command, if any
  virtual void command(EProcessCommand command_name, va_list& args) = 0;

  /// @brief Gets a state of the process
  /// @param state_name The desired state to get
  /// @return __void*__ Information on the desired state, if any
  virtual void* state(EProcessState state_name) = 0;

  /// @brief Copies a given process object
  /// @param rhs The process to copy
  /// @return __AProcess&__ reference to the copy of the process object
  AProcess& operator=(const AProcess& rhs) = default;

  /// @brief Moves a given process object
  /// @param rhs The process to move
  /// @return __AProcess&__ reference to the moved process object
  AProcess& operator=(AProcess&& rhs) = default;
};
}  // namespace processes
}  // namespace driftless
#endif