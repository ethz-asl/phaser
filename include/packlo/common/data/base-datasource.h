#pragma once

#include "packlo/model/point-cloud.h"
#include <boost/function.hpp>

#include <memory>

namespace data {

class BaseDatasource {
	public:
    virtual void subscribeToPointClouds(
        boost::function<void(const model::PointCloudPtr&)> func) = 0;
		virtual void startStreaming() = 0;

	protected:
		std::vector<boost::function<void(const model::PointCloudPtr&)>> callbacks_;
};

using DatasourcePtr = std::shared_ptr<BaseDatasource>;

} // namespace data
