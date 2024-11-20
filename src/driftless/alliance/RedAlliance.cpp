#include "driftless/alliance/RedAlliance.hpp"

namespace driftless {
namespace alliance {
std::string RedAlliance::getName() { return RED_ALLIANCE_NAME; }

io::RGBValue RedAlliance::getAllianceColor() {
  return ALLIANCE_COLOR;
}

io::RGBValue RedAlliance::getOpposingColor() {
  return OPPOSING_COLOR;
}
}
}