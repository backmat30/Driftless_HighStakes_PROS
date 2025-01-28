#include "driftless/alliance/BlueAlliance.hpp"

namespace driftless {
namespace alliance {
EAlliance BlueAlliance::getName() { return EAlliance::BLUE; }

io::RGBValue BlueAlliance::getAllianceColor() { return ALLIANCE_COLOR; }

io::RGBValue BlueAlliance::getOpposingColor() { return OPPOSING_COLOR; }
}  // namespace alliance
}  // namespace driftless