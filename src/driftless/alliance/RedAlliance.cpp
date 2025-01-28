#include "driftless/alliance/RedAlliance.hpp"

namespace driftless {
namespace alliance {
EAlliance RedAlliance::getName() { return EAlliance::RED; }

io::RGBValue RedAlliance::getAllianceColor() { return ALLIANCE_COLOR; }

io::RGBValue RedAlliance::getOpposingColor() { return OPPOSING_COLOR; }
}  // namespace alliance
}  // namespace driftless