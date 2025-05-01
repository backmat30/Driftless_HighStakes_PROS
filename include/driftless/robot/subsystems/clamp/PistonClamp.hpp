#ifndef __PISTON_CLAMP_HPP__
#define __PISTON_CLAMP_HPP__

#include <memory>

#include "driftless/io/IDistanceSensor.hpp"
#include "driftless/hal/PistonGroup.hpp"
#include "driftless/robot/subsystems/clamp/IClamp.hpp"

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief The namespace for subsystems code
/// @author Matthew Backman
namespace subsystems {

/// @brief The namespace for the clamp subsystem
/// @author Matthew Backman
namespace clamp {

/// @brief Class representing a piston-controlled clamp
/// @author Matthew Backman
class PistonClamp : public IClamp {
 private:
  // the pistons controlling the clamp
  driftless::hal::PistonGroup m_pistons{};

  std::unique_ptr<io::IDistanceSensor> m_distance_sensor{};

  double m_distance_to_goal{};

  // whether the clamp is active or idle
  bool state{};

 public:
  /// @brief Initializes the clamp
  void init() override;

  /// @brief Runs the clamp
  void run() override;

  /// @brief Sets the state of the clamp
  /// @param clamped __bool__ The state to set (true for clamped, false for unclamped)
  void setState(bool clamped) override;

  /// @brief Gets the state of the clamp
  /// @return __bool__ The current state of the clamp
  bool getState() override;

  /// @brief Determines if the clamp has a goal
  /// @return __bool__ True if the clamp has a goal, false otherwise
  bool hasGoal() override;

  /// @brief Sets the pistons used by the clamp
  /// @param pistons __driftless::hal::PistonGroup&__ The pistons to set
  void setPistons(driftless::hal::PistonGroup& pistons);

  /// @brief Sets the distance sensor used by the clamp
  /// @param distance_sensor __std::unique_ptr<io::IDistanceSensor>&__ The distance sensor to use
  void setDistanceSensor(std::unique_ptr<io::IDistanceSensor>& distance_sensor);

  /// @brief Sets the distance to the goal
  /// @param distance_to_goal __double__ The distance to the goal
  void setDistanceToGoal(double distance_to_goal);
};

}  // namespace clamp
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif