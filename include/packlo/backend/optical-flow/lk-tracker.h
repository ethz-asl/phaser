#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/highgui/highgui.hpp>


#include <sensor_msgs/Image.h>

#include <vector>
#include <array>

namespace optical_flow {

class LKTracker {
  public:
    LKTracker();
    void trackNewImage(const sensor_msgs::ImageConstPtr& img);
    void trackNewImages(const sensor_msgs::ImageConstPtr& intensity,
       const sensor_msgs::ImageConstPtr& range,
       const sensor_msgs::ImageConstPtr& noise);
  private:
    void track(const cv::Mat &img,
       const std_msgs::Header &header);
    cv::Mat applyClahe(const std::vector<cv::Mat> &img);
    void publishTrackedImage(const cv::Mat &img, 
       const std_msgs::Header &header);

    bool is_initialized_ = false;
    cv::Mat prev_tracked_image_;
    std::array<std::vector<cv::Point2f>, 2> tracked_points_;
    const int max_tracked_count_ = 500u;
    cv::Size sub_pix_win_size_;
    cv::Size win_size_;
    cv::TermCriteria term_crit_;
    std::size_t frame_counter_ = 0u;
};

}
