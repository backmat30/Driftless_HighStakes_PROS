#include "driftless/alliance/BlueAlliance.hpp"

namespace driftless {
namespace alliance {
std::string BlueAlliance::getName() { return BLUE_ALLIANCE_NAME; }

io::RGBValue BlueAlliance::getAllianceColor() {
  return ALLIANCE_COLOR;
}

io::RGBValue BlueAlliance::getOpposingColor() {
  return OPPOSING_COLOR;
}
}  // namespace alliance
}  // namespace driftless