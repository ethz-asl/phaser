#pragma once

#include "packlo/common/data/base-datasource.h"
#include "packlo/model/point-cloud.h"

#include <vector>

namespace data {

class DatasourcePly : public BaseDatasource {
	public:
    virtual void subscribeToPointClouds(
        boost::function<void(const model::PointCloudPtr&)> func) override;
		virtual void startStreaming() override;
	private:
		std::vector<model::PointCloudPtr> readPly(const std::string& directory);

};

} // namespace data
