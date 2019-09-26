#include "packlo/packlo-node.h"
#include "packlo/controller/distributor.h"
#include "packlo/visualization/plotty-visualizer.h"

#include <glog/logging.h>

namespace packlo {

PackloNode::PackloNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private) 
		:spinner_(1), node_handle_(nh), node_handle_private_(nh_private), ds_(nh) {
	should_exit_.store(false);
	dist_ = std::make_unique<controller::Distributor>(ds_);
}

bool PackloNode::run() {
	LOG(INFO) << "Running PackLO";
	spinner_.start();
	return dist_.get() != nullptr;
}

const std::atomic<bool>& PackloNode::shouldExit() const noexcept {
	return should_exit_;
}

std::string PackloNode::updateAndPrintStatistics() {
	/*
	std::vector<common::StatisticsManager> managers = retrieveStatistics();
	for (common::StatisticsManager manager : managers) {
	}
	*/
	dist_->updateStatistics();
	common::StatisticsManager manager = dist_->getStatistics();
	visualization::PlottyVisualizer::getInstance()
		.createPlotFor(manager, "signal_values");

	return "";
}

void PackloNode::shutdown() {

}

std::vector<common::StatisticsManager> PackloNode::retrieveStatistics()
		const noexcept {
	std::vector<common::StatisticsManager> managers; 
	managers.emplace_back(dist_->getStatistics());

	return managers;
}

} // namespace packlo

