#include "maplab-common/test/testing-entrypoint.h"
#include "packlo/common/data/datasource-ply.h"

#include <gtest/gtest.h>
#include <memory>

namespace rotation {

class RotationAlignmentTest : public ::testing::Test {
  protected:
    virtual void SetUp() {
      ds_ = std::make_shared<data::DatasourcePly>();
    }

    data::DatasourcePtr ds_;
};

TEST_F(RotationAlignmentTest, RotationEasy) {
  CHECK(ds_);

}

} // namespace rotation

MAPLAB_UNITTEST_ENTRYPOINT
