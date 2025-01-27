#include "driftless/processes/ProcessSystem.hpp"

namespace driftless {
namespace processes {
void ProcessSystem::addProcess(std::unique_ptr<AProcess>& process) {
  m_processes.push_back(std::move(process));
}

bool ProcessSystem::removeProcess(std::string process_name) {
  bool removed{};

  for (auto i{m_processes.begin()}; i != m_processes.end(); ++i) {
    if ((*i)->getName() == process_name) {
      m_processes.erase(i);
      removed = true;
      break;
    }
  }

  return removed;
}

void ProcessSystem::pause(std::string process_name) {
  for (auto& process : m_processes) {
    if (process->getName() == process_name) {
      process->pause();
    }
  }
}

void ProcessSystem::resume(std::string process_name) {
  for (auto& process : m_processes) {
    if (process->getName() == process_name) {
      process->resume();
    }
  }
}

void ProcessSystem::pauseAll() {
  for (auto& process : m_processes) {
    process->pause();
  }
}

void ProcessSystem::resumeAll() {
  for (auto& process : m_processes) {
    process->resume();
  }
}

void ProcessSystem::init() {
  for (auto& process : m_processes) {
    process->init();
  }
}

void ProcessSystem::run() {
  for (auto& process : m_processes) {
    process->run();
  }
}

void ProcessSystem::sendCommand(std::string process_name,
                                std::string command_name, ...) {
  va_list args;
  va_start(args, command_name);

  for (auto& process : m_processes) {
    if (process->getName() == process_name) {
      process->command(command_name, args);
      break;
    }
  }

  va_end(args);
}

void* ProcessSystem::getState(std::string process_name,
                              std::string state_name) {
  void* result{nullptr};

  for (auto& process : m_processes) {
    if (process->getName() == process_name) {
      result = process->state(state_name);
      break;
    }
  }

  return result;
}
}  // namespace processes
}  // namespace driftless