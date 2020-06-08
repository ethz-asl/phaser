#ifndef PHASER_BACKEND_FUSION_PYRAMID_STRUCT_H_
#define PHASER_BACKEND_FUSION_PYRAMID_STRUCT_H_

#include <cstdint>
#include <vector>

namespace phaser_core {

class PyramidStruct {
 public:
  explicit PyramidStruct(
      const uint32_t n_coeffs, const uint8_t n_levels, const float div);

  std::vector<uint32_t> getCoefficients() const noexcept;
  std::vector<uint32_t> getLowerBounds() const noexcept;
  std::vector<uint32_t> getUpperBounds() const noexcept;

  uint32_t getCoefficientsForLevel(const uint8_t level) const noexcept;
  uint32_t getLowerBoundForLevel(const uint8_t level) const noexcept;
  uint32_t getUpperBoundForLevel(const uint8_t level) const noexcept;
  uint32_t getInitialCoefficientSize() const noexcept;

 private:
  void computeCoefficientsPerLevel(
      const uint32_t init_coeffs, const uint8_t n_levels, const float div);

  std::vector<uint32_t> coefficients_per_level_;
  std::vector<uint32_t> lower_bound_per_level_;
  std::vector<uint32_t> upper_bound_per_level_;
  uint32_t initial_coefficient_size_;
};

}  // namespace phaser_core

#endif  // PHASER_BACKEND_FUSION_PYRAMID_STRUCT_H_
