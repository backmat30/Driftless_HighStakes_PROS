#include "driftless/processes/ProcessSystem.hpp"

namespace driftless {
namespace processes {
void ProcessSystem::addProcess(std::unique_ptr<AProcess>& process) {
  EProcess process_name{process->getName()};
  m_processes.emplace(process_name, std::move(process));
}

bool ProcessSystem::removeProcess(EProcess process_name) {
  bool removed{};

  removed = m_processes.erase(process_name) > 0;

  return removed;
}

void ProcessSystem::pause(EProcess process_name) {
  m_processes.at(process_name)->pause();
}

void ProcessSystem::resume(EProcess process_name) {
  m_processes.at(process_name)->resume();
}

void ProcessSystem::pauseAll() {
  for (auto i{m_processes.begin()}; i != m_processes.end(); ++i) {
    i->second->pause();
  }
}

void ProcessSystem::resumeAll() {
  for (auto i{m_processes.begin()}; i != m_processes.end(); ++i) {
    i->second->resume();
  }
}

void ProcessSystem::init() {
  for (auto i{m_processes.begin()}; i != m_processes.end(); ++i) {
    i->second->init();
  }
}

void ProcessSystem::run() {
  for (auto i{m_processes.begin()}; i != m_processes.end(); ++i) {
    i->second->run();
  }
}

void ProcessSystem::sendCommand(EProcess process_name,
                                EProcessCommand command_name, ...) {
  va_list args;
  va_start(args, command_name);

  m_processes.at(process_name)->command(command_name, args);

  va_end(args);
}

void* ProcessSystem::getState(EProcess process_name, EProcessState state_name) {
  void* result{nullptr};

  result = m_processes.at(process_name)->state(state_name);

  return result;
}
}  // namespace processes
}  // namespace driftless