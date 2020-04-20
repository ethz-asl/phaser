#include "phaser_pre/cloud-pre-processor.h"
#include "phaser_pre/commands/pass-through-gnd-filter-cmd.h"
#include "phaser_pre/commands/voxel-grid-cmd.h"
#include "phaser_pre/common/pre-processor-gflags.h"

namespace preproc {

CloudPreProcessorSettings CloudPreProcessorSettings::fromGflags() {
  CloudPreProcessorSettings settings;
  settings.enable_voxel_grid_downsampling =
      FLAGS_phaser_pre_enable_voxel_grid_downsampling;
  settings.enable_pass_through_gnd_filtering =
      FLAGS_phaser_pre_enable_pass_through_gnd_filtering;
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
  if (settings.enable_pass_through_gnd_filtering)
    processors_.emplace_back(std::make_unique<PassThroughGndFilterCmd>());
}

void CloudPreProcessor::process(model::PointCloudPtr cloud) {
  for (auto& processor : processors_) {
    processor->execute(cloud);
  }
}

}  // namespace preproc
