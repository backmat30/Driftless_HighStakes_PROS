#ifndef __PROCESS_SYSTEM_HPP__
#define __PROCESS_SYSTEM_HPP__

#include <cstdarg>
#include <memory>
#include <vector>

#include "driftless/processes/AProcess.hpp"

namespace driftless {
namespace processes {
class ProcessSystem {
 private:
  std::vector<std::unique_ptr<AProcess>> m_processes{};

 public:
  void addProcess(std::unique_ptr<AProcess>& process);

  bool removeProcess(std::string process_name);

  void pause(std::string process_name);

  void resume(std::string process_name);

  void pauseAll();

  void resumeAll();

  void init();

  void run();

  void sendCommand(std::string process_name, std::string command_name, ...);

  void* getState(std::string process_name, std::string state_name);
};
}  // namespace processes
}  // namespace driftless
#endif