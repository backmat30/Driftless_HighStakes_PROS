#ifndef __PROCESS_SYSTEM_HPP__
#define __PROCESS_SYSTEM_HPP__

#include <cstdarg>
#include <map>
#include <memory>
#include <vector>

#include "driftless/processes/AProcess.hpp"

namespace driftless {
namespace processes {
class ProcessSystem {
 private:
  std::map<EProcess, std::unique_ptr<AProcess>> m_processes{};

 public:
  void addProcess(std::unique_ptr<AProcess>& process);

  bool removeProcess(EProcess process_name);

  void pause(EProcess process_name);

  void resume(EProcess process_name);

  void pauseAll();

  void resumeAll();

  void init();

  void run();

  void sendCommand(EProcess process_name, EProcessCommand command_name, ...);

  void* getState(EProcess process_name, EProcessState state_name);
};
}  // namespace processes
}  // namespace driftless
#endif