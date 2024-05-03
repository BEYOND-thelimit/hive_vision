#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#define SHOW_MATCH 1

cv::Mat stitch_image(cv::Mat &img1, cv::Mat &img2, cv::Mat &H);

class RSImageStitcher : public rclcpp::Node
{
 public:
  RSImageStitcher(/* args */);
  ~RSImageStitcher();

 private:
  /* data */
  bool is_first_ = true;
  cv::Mat H;
  bool can_stitch_ = false;
  int translation_x;
  int translation_y;
  message_filters::Subscriber<sensor_msgs::msg::CompressedImage> up_cam_c_sub_;
  message_filters::Subscriber<sensor_msgs::msg::CompressedImage> bottom_cam_c_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> up_cam_d_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> bottom_cam_d_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr combined_c_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr combined_d_pub_;
  // 동기화 정책 타입 추가
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::CompressedImage,
                                                          sensor_msgs::msg::CompressedImage,
                                                          sensor_msgs::msg::Image,
                                                          sensor_msgs::msg::Image> MySyncPolicy;
  // Synchronizer 객체에 대한 포인터 선언
  std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;

  void syn_callback(const sensor_msgs::msg::CompressedImage::SharedPtr &up_cam_c_msg,
                    const sensor_msgs::msg::CompressedImage::SharedPtr &bottom_cam_c_msg,
                    const sensor_msgs::msg::Image::SharedPtr &up_cam_d_msg,
                    const sensor_msgs::msg::Image::SharedPtr &bottom_cam_d_msg);
};

RSImageStitcher::RSImageStitcher(/* args */) : Node("images_stitcher"),
                                              up_cam_c_sub_(this, "camera1/image_raw/compressed"),
                                              bottom_cam_c_sub_(this, "camera2/image_raw/compressed"),
                                              up_cam_d_sub_(this, "camera1/depth/image_raw"),
                                              bottom_cam_d_sub_(this, "camera2/depth/image_raw")
{
  sync_ = std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(MySyncPolicy(10), up_cam_c_sub_, bottom_cam_c_sub_,up_cam_d_sub_,bottom_cam_d_sub_);
  sync_->registerCallback(&RSImageStitcher::syn_callback,this);
  combined_c_pub_ = this->create_publisher<sensor_msgs::msg::Image>("rs_camera/color/image_raw", 10);
  combined_d_pub_ = this->create_publisher<sensor_msgs::msg::Image>("rs_camera/depth/image_raw", 10);
}

RSImageStitcher::~RSImageStitcher()
{
}
void RSImageStitcher::syn_callback(const sensor_msgs::msg::CompressedImage::SharedPtr &up_cam_c_msg,
                    const sensor_msgs::msg::CompressedImage::SharedPtr &bottom_cam_c_msg,
                    const sensor_msgs::msg::Image::SharedPtr &up_cam_d_msg,
                    const sensor_msgs::msg::Image::SharedPtr &bottom_cam_d_msg)
{
  cv_bridge::CvImagePtr cv_ptr_c1;
  cv_bridge::CvImagePtr cv_ptr_c2;
  cv_bridge::CvImagePtr cv_ptr_d1;
  cv_bridge::CvImagePtr cv_ptr_d2;
  try
  {
    cv_ptr_c1 = cv_bridge::toCvCopy(up_cam_c_msg, sensor_msgs::image_encodings::BGR8);
    cv_ptr_c2 = cv_bridge::toCvCopy(bottom_cam_c_msg, sensor_msgs::image_encodings::BGR8);
    cv_ptr_d1 = cv_bridge::toCvCopy(up_cam_d_msg, sensor_msgs::image_encodings::TYPE_16UC1);
    cv_ptr_d2 = cv_bridge::toCvCopy(bottom_cam_d_msg, sensor_msgs::image_encodings::TYPE_16UC1);
  }
  catch(cv_bridge::Exception &e)
  {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }
  cv::Mat cvimg_from_up_cam_c = cv_ptr_c1->image;
  cv::Mat cvimg_from_bottom_cam_c = cv_ptr_c2->image;
  cv::Mat cvimg_from_up_cam_d = cv_ptr_d1->image;
  cv::Mat cvimg_from_bottom_cam_d = cv_ptr_d2->image;

  //TODO: 이미지 스티칭 코드 작성
  cv::Mat g_cvimg_from_up_cam_c, g_cvimg_from_bottom_cam_c;
  cv::cvtColor(cvimg_from_up_cam_c, g_cvimg_from_up_cam_c, cv::COLOR_BGR2GRAY);
  cv::cvtColor(cvimg_from_bottom_cam_c, g_cvimg_from_bottom_cam_c, cv::COLOR_BGR2GRAY);

// feature extraction
  cv::Ptr<cv::Feature2D> detector = cv::ORB::create();
  std::vector<cv::KeyPoint> keypoints1, keypoints2;
  cv::Mat descriptors1, descriptors2;
  detector->detectAndCompute(g_cvimg_from_up_cam_c, cv::noArray(), keypoints1, descriptors1);
  detector->detectAndCompute(g_cvimg_from_bottom_cam_c, cv::noArray(), keypoints2, descriptors2);

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
    cv::drawMatches(g_cvimg_from_up_cam_c, keypoints1, g_cvimg_from_bottom_cam_c, keypoints2, good_matches, img_matches);
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
  cv::Mat H_hat = cv::findHomography(pts2, pts1, cv::RANSAC);
  double homography_threshold = 0.05; // hand-crafted threshold
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
  else if (!can_stitch_)
  {
    std::cout << H_hat << std::endl;
  }
  else if (can_stitch_)
  {
    cv::Mat stitching_result_c = stitch_image(cvimg_from_up_cam_c, cvimg_from_bottom_cam_c, H);
    cv::imshow("Color", stitching_result_c);
    cv::waitKey(1);

    cv::Mat stitching_result_d = stitch_image(cvimg_from_up_cam_d, cvimg_from_bottom_cam_d, H);
    cv::imshow("Depth", stitching_result_d);
    cv::waitKey(1);

    sensor_msgs::msg::Image::SharedPtr combined_c_msg
                        = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", stitching_result_c).toImageMsg();
    combined_c_pub_->publish(*combined_c_msg);
    sensor_msgs::msg::Image::SharedPtr combined_d_msg
                        = cv_bridge::CvImage(std_msgs::msg::Header(), "16UC1", stitching_result_d).toImageMsg();
    combined_d_pub_->publish(*combined_d_msg);
  }
}

cv::Mat stitch_image(cv::Mat &img1, cv::Mat &img2, cv::Mat &H)
{
  cv::Mat mat_result;
    cv::warpPerspective(img2, mat_result, H, cv::Size(img1.cols*1.2, img1.rows*2), cv::INTER_CUBIC);

    cv::Mat mat_panorama;
	  mat_panorama = mat_result.clone();

    cv::Mat mat_roi(mat_panorama, cv::Rect(0, 0, img1.cols, img1.rows));
    img1.copyTo(mat_roi);

    int colorx = 0, colory = 0;
    for (int y = 0; y < mat_panorama.rows; y++) {
      for (int x = 0; x < mat_panorama.cols; x++) {
        if (mat_panorama.at<cv::Vec3b>(y, x)[0] == 0 && mat_panorama.at<cv::Vec3b>(y, x)[1] == 0 && mat_panorama.at<cv::Vec3b>(y, x)[2] == 0) {
          continue;
        }
        if (colorx < x) {
          colorx = x;
        }
        if (colory < y){
          colory = y;
        }
      }
    }
    cv::Mat black_cut_panorama;
    black_cut_panorama = mat_panorama(cv::Range(0, colory), cv::Range(0, colorx));

    return black_cut_panorama;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RSImageStitcher>());
  rclcpp::shutdown();
  return 0;
}