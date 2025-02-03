#ifndef __RANGE_HPP__
#define __RANGE_HPP__

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for utility code
/// @author Matthew Backman
namespace utils {
  
/// @brief Template for a range of values given a max and minimum
/// @tparam t Type of data stored in the range
/// @author Matthew Backman
template <typename T>
struct Range {
  T min{};

  T max{};
};
}  // namespace utils
}  // namespace driftless
#endif