#include <packlo/backend/optical-flow/lk-tracker.h>

#include <cv_bridge/cv_bridge.h>
#include <glog/logging.h>

namespace optical_flow {

LKTracker::LKTracker() 
  : sub_pix_win_size_(10, 10), win_size_(31, 31),
    term_crit_(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 20, 0.03) {
  cv::namedWindow( "LK Demo", 1 );
}

void LKTracker::trackNewImage(const sensor_msgs::ImageConstPtr& img)
{
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    track(cv_ptr->image);
  } catch (cv_bridge::Exception& e) {
    LOG(ERROR) << "Unable to convert to cv";
    VLOG(2) << "cv_bridge exception: " << e.what();
    return;
  }
}

void LKTracker::track(const cv::Mat &img) {
  // Convert input image to grayscale.
  cv::Mat conv_gray;
  cv::cvtColor(img, conv_gray, cv::COLOR_BGR2GRAY);

  if (!is_initialized_) {
    cv::goodFeaturesToTrack(conv_gray, tracked_points_[1], max_tracked_count_, 
        0.01, 10, cv::Mat(), 3, false, 0.04);
    cv::cornerSubPix(conv_gray, tracked_points_[1], 
        sub_pix_win_size_, cv::Size(-1,-1), term_crit_);
  } else if (!tracked_points_[0].empty()){
    std::vector<uchar> status;
    std::vector<float> err;
    if(prev_tracked_image_.empty())
      conv_gray.copyTo(prev_tracked_image_);
    calcOpticalFlowPyrLK(prev_tracked_image_, conv_gray,
        tracked_points_[0], tracked_points_[1],
        status, err, win_size_, 3, term_crit_, 0, 0.001);

    std::size_t i, k;
    const std::size_t n_points = tracked_points_[1].size();
    for( i = k = 0u; i < n_points; ++i ) {
      if(!status[i]) continue;
      tracked_points_[1][k++] = tracked_points_[1][i];
      cv::circle(img, tracked_points_[1][i], 3, cv::Scalar(0,255,0), -1, 8);
    }
    tracked_points_[1].resize(k);
  }

  is_initialized_ = true;
  cv::imshow("LK Demo", img);
  cv::waitKey(1);
  std::swap(tracked_points_[1], tracked_points_[0]);
  cv::swap(prev_tracked_image_, conv_gray);
}

} // namespace optical_flow
