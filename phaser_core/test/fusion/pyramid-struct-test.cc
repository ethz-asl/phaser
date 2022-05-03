#include "phaser/backend/fusion/pyramid-struct.h"

#include <chrono>
#include <cmath>
#include <gtest/gtest.h>
#include <memory>
#include <random>

#include "phaser/common/test/testing-entrypoint.h"
#include "phaser/common/test/testing-predicates.h"

namespace phaser_core {

class PyramidStructTest : public ::testing::Test {
 public:
  PyramidStructTest() {}

 private:
};

TEST_F(PyramidStructTest, ComputePyramidStructureTest) {
  const uint8_t n_levels = 2u;
  PyramidStruct py_struct(100u, n_levels, 4u);

  EXPECT_EQ(py_struct.getCoefficientsForLevel(0), 50u);
  EXPECT_EQ(py_struct.getCoefficientsForLevel(1), 25u);

  EXPECT_EQ(py_struct.getLowerBoundForLevel(0), 25u);
  EXPECT_EQ(py_struct.getLowerBoundForLevel(1), 13u);

  EXPECT_EQ(py_struct.getUpperBoundForLevel(0), 75u);
  EXPECT_EQ(py_struct.getUpperBoundForLevel(1), 38u);
}

}  // namespace phaser_core

MAPLAB_UNITTEST_ENTRYPOINT
