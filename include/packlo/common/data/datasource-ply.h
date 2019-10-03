#pragma once

#include "packlo/common/data/base-datasource.h"

namespace data {

class DataSourcePly : public BaseDatasource {
	public:
    virtual void subscribeToPointClouds(
        boost::function<void(const model::PointCloudPtr&)> func) override;
};

} // namespace data
