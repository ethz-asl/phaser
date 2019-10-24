#include "maplab-common/test/testing-entrypoint.h"
#include "packlo/common/data/datasource-ply.h"

#include <gtest/gtest.h>
#include <memory>

namespace transformation {

class TransformationAlignmentTest : public ::testing::Test {
  protected:
    virtual void SetUp() {
      ds_ = std::make_shared<data::DatasourcePly>();
    }

    data::DatasourcePtr ds_;
};

TEST_F(TransformationAlignmentTest, TransformEasy) {
  CHECK(ds_);

}

} // namespace transformation

MAPLAB_UNITTEST_ENTRYPOINT
