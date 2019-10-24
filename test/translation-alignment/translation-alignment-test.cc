#include "maplab-common/test/testing-entrypoint.h"
#include "packlo/common/data/datasource-ply.h"

#include <gtest/gtest.h>
#include <memory>

namespace translation {

class TranslationAlignmentTest : public ::testing::Test {
  protected:
    virtual void SetUp() {
      ds_ = std::make_shared<data::DatasourcePly>();
    }

    data::DatasourcePtr ds_;
};

TEST_F(TranslationAlignmentTest, TranslationEasy) {
  CHECK(ds_);

}

} // namespace translation

MAPLAB_UNITTEST_ENTRYPOINT
