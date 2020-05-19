#include "phaser/common/test/testing-entrypoint.h"
#include "phaser/common/test/testing-predicates.h"

#include <gtest/gtest.h>

namespace fusion {

class LaplacePyramidTest : public ::testing::Test {
 protected:
  virtual void SetUp() {}
};

TEST_F(LaplacePyramidTest, SimpleReduceTest) {}

}  // namespace fusion

MAPLAB_UNITTEST_ENTRYPOINT
