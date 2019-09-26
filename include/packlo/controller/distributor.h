#pragma once

#include "packlo/common/datasource.h"
#include "packlo/model/point-cloud.h"
#include "packlo/backend/registration/base-registration.h"
#include "packlo/common/statistics-manager.h"

#include <memory>

namespace controller {

class Distributor {

  public:
    explicit Distributor(common::Datasource& ds);

		void updateStatistics();
		void getStatistics(common::StatisticsManager*) const noexcept;

  private:
    void subscribeToTopics();
		void initializeRegistrationAlgorithm(const std::string& type);
    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud);
		model::PointCloud_tPtr preprocessPointCloud(
				const sensor_msgs::PointCloud2ConstPtr& cloud);
    
    common::Datasource& ds_;
		std::unique_ptr<registration::BaseRegistration> registrator_;
		model::PointCloudPtr prev_point_cloud_;

		// Statistics
		const std::string kManagerReferenceName = "Distributor";
		common::StatisticsManager statistics_manager_;
};

}
