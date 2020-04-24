#include "phaser_pre/cloud-pre-processor.h"
#include "phaser_pre/commands/pass-through-gnd-filter-cmd.h"
#include "phaser_pre/commands/segment-cloud-cmd.h"
#include "phaser_pre/commands/voxel-grid-cmd.h"
#include "phaser_pre/common/pre-processor-gflags.h"

namespace preproc {

CloudPreProcessorSettings CloudPreProcessorSettings::fromGflags() {
  CloudPreProcessorSettings settings;
  settings.enable_voxel_grid_downsampling =
      FLAGS_phaser_pre_enable_voxel_grid_downsampling;
  settings.enable_pass_through_gnd_filtering =
      FLAGS_phaser_pre_enable_pass_through_gnd_filtering;
  settings.enable_geometric_cloud_segmentation =
      FLAGS_phaser_pre_enable_geometric_cloud_segmentation;
  return settings;
}

CloudPreProcessor::CloudPreProcessor() {
  CloudPreProcessorSettings settings = CloudPreProcessorSettings::fromGflags();
  initializeCommandFromSettings(settings);
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
  if (settings.enable_geometric_cloud_segmentation)
    processors_.emplace_back(std::make_unique<SegmentCloudCmd>());
}

void CloudPreProcessor::process(model::PointCloudPtr cloud) {
  for (auto& processor : processors_) {
    processor->execute(cloud);
  }
}

}  // namespace preproc
