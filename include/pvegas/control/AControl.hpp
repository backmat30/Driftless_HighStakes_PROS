#ifndef __A_CONTROL_HPP__
#define __A_CONTROL_HPP__

#include <string>
namespace pvegas {
namespace control {
// Abstract class for control algorithms
class AControl {
private:
  std::string m_name{};

public:
  // basic constructor
  AControl() = default;

  // copy constructor
  AControl(const AControl &other) = default;

  // moves an AControl object to a new AControl object
  AControl(AControl &&other) = default;

  // constructor
  AControl(std::string name) : m_name{name} {}

  // basic deconstructor
  virtual ~AControl() = default;

  // gets the name of the control system
  const std::string &getName() const { return m_name; }

  // init the system
  virtual void init() = 0;

  // runs the system
  virtual void run() = 0;

  // pauses the system
  virtual void pause() = 0;

  // resumes the paused system
  virtual void resume() = 0;

  // runs a command for the control system
  virtual void command(std::string command_name, va_list &args) = 0;

  // gets the current state of the control
  virtual void *state(std::string state_name) = 0;

  // copy assignment operator ovveride
  AControl& operator=(const AControl &rhs) = default;

  // move assignment operator override
  // AControl& operator=(const AControl &&rhs) = default;
};
} // namespace control
} // namespace pvegas
#endif