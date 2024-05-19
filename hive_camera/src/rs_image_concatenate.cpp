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
#define FIND_HOMOGRAPHY 1

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
  float trans_x = 0.0;
  float trans_y = 0.0;
  cv::Mat final_H;
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
                                              up_cam_c_sub_(this, "camera2/camera2/color/image_raw/compressed"),
                                              bottom_cam_c_sub_(this, "camera1/camera1/color/image_raw/compressed"),
                                              up_cam_d_sub_(this, "camera2/camera2/aligned_depth_to_color/image_raw"),
                                              bottom_cam_d_sub_(this, "camera1/camera1/aligned_depth_to_color/image_raw")
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
  cv::Mat up_cam_c = cv::imdecode(cv::Mat(up_cam_c_msg->data), cv::IMREAD_COLOR);
  cv::Mat bottom_cam_c = cv::imdecode(cv::Mat(bottom_cam_c_msg->data), cv::IMREAD_COLOR);
  cv::Mat up_cam_d = cv_bridge::toCvCopy(up_cam_d_msg, sensor_msgs::image_encodings::TYPE_16UC1)->image;
  cv::Mat bottom_cam_d = cv_bridge::toCvCopy(bottom_cam_d_msg, sensor_msgs::image_encodings::TYPE_16UC1)->image;

  cv::Mat combined_c;
  cv::Mat combined_d;

  //concate image
  cv::vconcat(up_cam_c, bottom_cam_c, combined_c);
  cv::vconcat(up_cam_d, bottom_cam_d, combined_d);

  sensor_msgs::msg::Image::SharedPtr combined_c_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", combined_c).toImageMsg();
  sensor_msgs::msg::Image::SharedPtr combined_d_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono16", combined_d).toImageMsg();
  combined_c_pub_->publish(*combined_c_msg);
  combined_d_pub_->publish(*combined_d_msg);

}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RSImageStitcher>());
  rclcpp::shutdown();
  return 0;
}