#ifndef PHASER_PRE_CLOUD_PRE_PROCESSOR_H_
#define PHASER_PRE_CLOUD_PRE_PROCESSOR_H_

#include <vector>

#include "phaser_pre/common/base-command.h"
#include "phaser_pre/common/base-pre-processor.h"

namespace preproc {

struct CloudPreProcessorSettings {
  static CloudPreProcessorSettings fromGflags();
  bool enable_voxel_grid_downsampling = false;
  bool enable_pass_through_gnd_filtering = false;
  bool enable_geometric_cloud_segmentation = false;
};

class CloudPreProcessor : public BasePreProcessor {
 public:
  CloudPreProcessor();
  explicit CloudPreProcessor(const CloudPreProcessorSettings& settings);
  void process(model::PointCloudPtr cloud) override;

 private:
  void initializeCommandFromSettings(const CloudPreProcessorSettings& settings);
  std::vector<BaseCommandPtr> processors_;
};

}  // namespace preproc

#endif  // PHASER_PRE_CLOUD_PRE_PROCESSOR_H_
