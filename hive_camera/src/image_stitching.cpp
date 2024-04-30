#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/calib3d.hpp"

#define SHOW_MATCH 1
#define STITCHING_MODE 1 // 0: feature matching, 1: concatenate

class ImageStitcher : public rclcpp::Node
{
 public:
  ImageStitcher();
  ~ImageStitcher();

 private:
  bool is_first_ = true;
  cv::Mat H;
  bool can_stitch_ = false;
  int translation_x;
  int translation_y;
  message_filters::Subscriber<sensor_msgs::msg::Image> camera1_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> camera2_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr combined_image_;

  // 동기화 정책 타입 추가
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> MySyncPolicy;
  // Synchronizer 객체에 대한 포인터 선언
  std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;

  void syn_callback(const sensor_msgs::msg::Image::SharedPtr &camera1_msg, const sensor_msgs::msg::Image::SharedPtr &camera2_msg);
};

ImageStitcher::ImageStitcher() : Node("image_stitcher"),
                                 camera1_sub_(this, "camera1/image_raw"),
                                 camera2_sub_(this, "camera2/image_raw")
{
  sync_ = std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(MySyncPolicy(10), camera1_sub_, camera2_sub_);
  sync_->registerCallback(&ImageStitcher::syn_callback,this);
  combined_image_ = this->create_publisher<sensor_msgs::msg::Image>("combined_image", 10);
}

ImageStitcher::~ImageStitcher()
{
}

void ImageStitcher::syn_callback(const sensor_msgs::msg::Image::SharedPtr &camera1_msg, const sensor_msgs::msg::Image::SharedPtr &camera2_msg)
{
  cv_bridge::CvImagePtr cv_ptr1;
  cv_bridge::CvImagePtr cv_ptr2;
  try
  {
    cv_ptr1 = cv_bridge::toCvCopy(camera1_msg, sensor_msgs::image_encodings::BGR8);
    cv_ptr2 = cv_bridge::toCvCopy(camera2_msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception &e)
  {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }
  cv::Mat cvimg_from_camera1 = cv_ptr1->image;
  cv::Mat cvimg_from_camera2 = cv_ptr2->image;

  if (STITCHING_MODE)
  {
    // 이미지 이어붙이기
    cv::Mat combined_image(cvimg_from_camera1.rows + cvimg_from_camera2.rows, cvimg_from_camera2.cols, cvimg_from_camera1.type());
    cvimg_from_camera1.copyTo(combined_image(cv::Rect(0, 0, cvimg_from_camera1.cols, cvimg_from_camera1.rows)));
    cvimg_from_camera2.copyTo(combined_image(cv::Rect(0, cvimg_from_camera1.rows, cvimg_from_camera2.cols, cvimg_from_camera2.rows)));
    sensor_msgs::msg::Image::SharedPtr combined_image_msg = cv_bridge::CvImage(camera1_msg->header, sensor_msgs::image_encodings::BGR8, combined_image).toImageMsg();
    combined_image_->publish(*combined_image_msg);
  }
  else
  {
    // feature extraction
    cv::Ptr<cv::Feature2D> detector = cv::ORB::create();
    std::vector<cv::KeyPoint> keypoints1, keypoints2;
    cv::Mat descriptors1, descriptors2;
    detector->detectAndCompute(cvimg_from_camera1, cv::noArray(), keypoints1, descriptors1);
    detector->detectAndCompute(cvimg_from_camera2, cv::noArray(), keypoints2, descriptors2);

    // feature matching with good matching
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE_HAMMING);
    std::vector<cv::DMatch> matches;
    matcher->match(descriptors1, descriptors2, matches);
    std::sort(matches.begin(), matches.end());
    int num_good_matches = matches.size() * 0.05; // hand-crafted threshold
    std::vector<cv::DMatch> good_matches(matches.begin(), matches.begin() + num_good_matches);
    if (SHOW_MATCH)
    {
      cv::Mat img_matches;
      cv::drawMatches(cvimg_from_camera1, keypoints1, cvimg_from_camera2, keypoints2, good_matches, img_matches);
      cv::imshow("Matches", img_matches);
      //RCLCPP_INFO(this->get_logger(), "Matches: %ld", good_matches.size());
      cv::waitKey(1);
      if (cv::waitKey(1) == 27)
      {
        RCLCPP_INFO(this->get_logger(), "ESC key is pressed by user");
        return;
      }
    }

    // homography estimation
    std::vector<cv::Point2f> pts1;
    std::vector<cv::Point2f> pts2;
    for (size_t i = 0; i < good_matches.size(); i++)
    {
      pts1.push_back(keypoints1[good_matches[i].queryIdx].pt);
      pts2.push_back(keypoints2[good_matches[i].trainIdx].pt);
    }
    cv::Mat H_hat = cv::findHomography(pts1, pts2, cv::RANSAC);
    double homography_threshold = 0.01; // hand-crafted threshold
    // 회전 행렬이 거의 단위 행렬이고, 스케일이 거의 1인 경우라 가정
    if (is_first_ && abs(H_hat.at<double>(0, 0) - 1.0) <= homography_threshold
                  && abs(H_hat.at<double>(1, 1) - 1.0) <= homography_threshold
                  && abs(H_hat.at<double>(2, 2) - 1.0) <= homography_threshold)
    {
      RCLCPP_INFO(this->get_logger(), "Homography is identity matrix");
      is_first_ = false;
      H = H_hat.clone();
      can_stitch_ = true;
      translation_x = H.at<int>(0, 2);
      translation_y = H.at<int>(1, 2);
      std::cout << H << std::endl;
    }
    // image stitching
    //TODO: 이미지 스티칭 코드 작성
  }
  
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageStitcher>());
  rclcpp::shutdown();
  return 0;
}
