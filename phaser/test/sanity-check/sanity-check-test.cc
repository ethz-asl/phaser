#include "maplab-common/test/testing-entrypoint.h"

#include <gtest/gtest.h>

namespace sanity {

class SanityCheckTest : public ::testing::Test {
  protected:
    virtual void SetUp() { }
};

TEST(SanityCheckTest, SanityCheck) {
  EXPECT_TRUE(true);
}

} // namespace sanity

MAPLAB_UNITTEST_ENTRYPOINT
