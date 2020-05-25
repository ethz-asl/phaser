#include "phaser/common/test/testing-entrypoint.h"

#include <gtest/gtest.h>

namespace phaser_core {

class SanityCheckTest : public ::testing::Test {
 protected:
  virtual void SetUp() {}
};

TEST(SanityCheckTest, SanityCheck) {
  EXPECT_TRUE(true);
}

}  // namespace phaser_core

MAPLAB_UNITTEST_ENTRYPOINT
