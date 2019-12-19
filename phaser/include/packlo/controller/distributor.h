#pragma once

#include "packlo/common/data/datasource-ros.h"
#include "packlo/model/point-cloud.h"
#include "packlo/backend/registration/base-registration.h"
#include "packlo/common/statistics-manager.h"

#include <memory>

namespace controller {

class Distributor {

  public:
    explicit Distributor(data::DatasourcePtr& ds);

    void updateStatistics();
    void getStatistics(common::StatisticsManager*) const noexcept;

    void setRegistrationAlgorithm(std::string&& algorithm);
    void setRegistrationAlgorithm(const std::string& algorithm);
    void initializeRegistrationAlgorithm();

  private:
    void subscribeToTopics();
    void pointCloudCallback(const model::PointCloudPtr& cloud);
    void registerPointCloud(const model::PointCloudPtr& cloud);
    void preprocessPointCloud(const model::PointCloudPtr& cloud);
    
    data::DatasourcePtr ds_;
    registration::BaseRegistrationPtr registrator_;
    model::PointCloudPtr prev_point_cloud_;
    std::string registration_algorithm_;

    // Statistics
    const std::string kManagerReferenceName = "Distributor";
    common::StatisticsManager statistics_manager_;
};

}
