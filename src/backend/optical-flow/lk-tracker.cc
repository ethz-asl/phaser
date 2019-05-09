#include <packlo/backend/optical-flow/lk-tracker.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/cuda.hpp>

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

void LKTracker::trackNewImages(const sensor_msgs::ImageConstPtr& intensity,
   const sensor_msgs::ImageConstPtr& range,
   const sensor_msgs::ImageConstPtr& noise) {
  cv_bridge::CvImagePtr cv_ptr_intensity, cv_ptr_range, cv_ptr_noise;
  try {
    cv_ptr_intensity 
      = cv_bridge::toCvCopy(intensity, sensor_msgs::image_encodings::MONO8);
    cv_ptr_range
      = cv_bridge::toCvCopy(range, sensor_msgs::image_encodings::MONO8);
    cv_ptr_noise
      = cv_bridge::toCvCopy(noise, sensor_msgs::image_encodings::MONO8);

    std::vector<cv::Mat> image_array {cv_ptr_intensity->image,
      cv_ptr_range->image, cv_ptr_noise->image};
    cv::Mat equalized = applyClahe(image_array);
    track(equalized);
  } catch (cv_bridge::Exception& e) {
    LOG(ERROR) << "Unable to convert to cv";
    VLOG(2) << "cv_bridge exception: " << e.what();
    return;
  }
  if (++frame_counter_ % 30 == 0) 
    is_initialized_ = false;
}

cv::Mat LKTracker::applyClahe(const std::vector<cv::Mat> &img_input) {
  cv::Ptr<cv::CLAHE> clahe_low = cv::createCLAHE(1.0, cv::Size(8, 8));
  cv::Ptr<cv::CLAHE> clahe_med = cv::createCLAHE(3.0, cv::Size(8, 8));
  cv::Ptr<cv::CLAHE> clahe_high = cv::createCLAHE(5.0, cv::Size(8, 8));

  //std::vector<cv::Mat> img_array {img_input[0], img_input[1], img_input[2]};
  std::vector<cv::Mat> img_array {img_input[0], img_input[1], img_input[0], img_input[1]};
  cv::Mat merged;
  cv::merge(img_array, merged);
  cv::Mat clahe_input;
  cv::cvtColor(merged, clahe_input, cv::COLOR_BGR2GRAY);

  cv::Mat img_low, img_med, img_high;
  /*
  clahe_low->apply(img_input[0], img_low);
  clahe_med->apply(img_input[1], img_med);
  clahe_high->apply(img_input[2], img_high);
  */
  clahe_low->apply(clahe_input, img_low);
  clahe_med->apply(clahe_input, img_med);
  clahe_high->apply(clahe_input, img_high);

  std::vector<cv::Mat> clahe_array {img_low, img_med, img_high};
  //cv::Mat merged;
  cv::merge(clahe_array, merged);

  return merged;
}

void LKTracker::track(const cv::Mat &img) {
  // Convert input image to grayscale.
  cv::Mat conv_gray;
  cv::cvtColor(img, conv_gray, cv::COLOR_BGR2GRAY);
  cv::Mat cropped = conv_gray(cv::Rect(128, 0, 512, 64));
  //cv::Mat image = conv_gray;
  cv::Mat image;
  cv::resize(cropped, image, cv::Size(512, 128), 0, 0, cv::INTER_NEAREST) ;


  if (!is_initialized_) {
    cv::goodFeaturesToTrack(image, tracked_points_[1], max_tracked_count_, 
        0.001, 0, cv::Mat(), 3, true, 0.02);
    cv::cornerSubPix(image, tracked_points_[1], 
        sub_pix_win_size_, cv::Size(-1,-1), term_crit_);
  } else if (!tracked_points_[0].empty()){
    std::vector<uchar> status;
    std::vector<float> err;
    if(prev_tracked_image_.empty())
      image.copyTo(prev_tracked_image_);
    calcOpticalFlowPyrLK(prev_tracked_image_, image,
        tracked_points_[0], tracked_points_[1],
        status, err, win_size_, 3, term_crit_, 0, 0.001);

    std::size_t i, k;
    const std::size_t n_points = tracked_points_[1].size();
    for( i = k = 0u; i < n_points; ++i ) {
      if(!status[i]) continue;
      tracked_points_[1][k++] = tracked_points_[1][i];
      cv::circle(image, tracked_points_[1][i], 3, cv::Scalar(0,255,0), -1, 8);
    }
    tracked_points_[1].resize(k);
  }

  is_initialized_ = true;
  cv::imshow("LK Demo", image);
  cv::waitKey(1);
  std::swap(tracked_points_[1], tracked_points_[0]);
  cv::swap(prev_tracked_image_, image);
}

} // namespace optical_flow
