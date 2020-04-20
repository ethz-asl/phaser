#include "phaser_pre/cloud-pre-processor.h"
#include "phaser_pre/commands/voxel-grid-cmd.h"
#include "phaser_pre/common/pre-processor-gflags.h"

namespace preproc {

CloudPreProcessorSettings CloudPreProcessorSettings::fromGflags() {
  CloudPreProcessorSettings settings;
  settings.enable_voxel_grid_downsampling =
      FLAGS_phaser_pre_enable_voxel_grid_downsampling;
  return settings;
}

CloudPreProcessor::CloudPreProcessor(
    const CloudPreProcessorSettings& settings) {
  initializeCommandFromSettings(settings);
}

void CloudPreProcessor::initializeCommandFromSettings(
    const CloudPreProcessorSettings& settings) {
  if (settings.enable_voxel_grid_downsampling)
    processors_.emplace_back(std::make_unique<VoxelGridCmd>());
}

void CloudPreProcessor::process(model::PointCloudPtr cloud) {
  for (auto& processor : processors_) {
    processor->execute(cloud);
  }
}

}  // namespace preproc
