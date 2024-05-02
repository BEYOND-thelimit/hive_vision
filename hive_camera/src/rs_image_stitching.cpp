#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/calib3d.hpp"

class RSImageStitcher : public rclcpp::Node
{
 public:
  RSImageStitcher(/* args */);
  ~RSImageStitcher();

 private:
  /* data */
  message_filters::Subscriber<sensor_msgs::msg::CompressedImage> camera1_c_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> camera2_c_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> camera1_d_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> camera2_d_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr combined_c_image_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr combined_d_image_;
  // 동기화 정책 타입 추가
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::CompressedImage,
                                                          sensor_msgs::msg::Image,
                                                          sensor_msgs::msg::Image,
                                                          sensor_msgs::msg::Image> MySyncPolicy;
  // Synchronizer 객체에 대한 포인터 선언
  std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;

  void syn_callback(const sensor_msgs::msg::CompressedImage::SharedPtr &camera1_c_msg,
                    const sensor_msgs::msg::Image::SharedPtr &camera2_c_msg,
                    const sensor_msgs::msg::Image::SharedPtr &camera1_d_msg,
                    const sensor_msgs::msg::Image::SharedPtr &camera2_d_msg);
};

RSImageStitcher::RSImageStitcher(/* args */) : Node("images_stitcher"),
                                              camera1_c_sub_(this, "camera/camera/color/image_raw/compressed"),
                                              camera2_c_sub_(this, "camera1/image_raw"),
                                              camera1_d_sub_(this, "camera/camera/aligned_depth_to_color/image_raw"),
                                              camera2_d_sub_(this, "camera/camera/aligned_depth_to_color/image_raw")
{
  sync_ = std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(MySyncPolicy(10), camera1_c_sub_, camera2_c_sub_,camera1_d_sub_,camera2_d_sub_);
  sync_->registerCallback(&RSImageStitcher::syn_callback,this);
  combined_c_image_ = this->create_publisher<sensor_msgs::msg::Image>("rs_camera/color/image_raw", 10);
  combined_d_image_ = this->create_publisher<sensor_msgs::msg::Image>("rs_camera/depth/image_raw", 10);
}

RSImageStitcher::~RSImageStitcher()
{
}
void RSImageStitcher::syn_callback(const sensor_msgs::msg::CompressedImage::SharedPtr &camera1_c_msg,
                    const sensor_msgs::msg::Image::SharedPtr &camera2_c_msg,
                    const sensor_msgs::msg::Image::SharedPtr &camera1_d_msg,
                    const sensor_msgs::msg::Image::SharedPtr &camera2_d_msg)
{
  RCLCPP_INFO(this->get_logger(), "Received images");
  cv_bridge::CvImagePtr cv_ptr_c1;
  cv_bridge::CvImagePtr cv_ptr_c2;
  cv_bridge::CvImagePtr cv_ptr_d1;
  cv_bridge::CvImagePtr cv_ptr_d2;
  try
  {
    cv_ptr_c1 = cv_bridge::toCvCopy(camera1_c_msg, sensor_msgs::image_encodings::BGR8);
    cv_ptr_c2 = cv_bridge::toCvCopy(camera2_c_msg, sensor_msgs::image_encodings::BGR8);
    cv_ptr_d1 = cv_bridge::toCvCopy(camera1_d_msg, sensor_msgs::image_encodings::TYPE_16UC1);
    cv_ptr_d2 = cv_bridge::toCvCopy(camera2_d_msg, sensor_msgs::image_encodings::TYPE_16UC1);
  }
  catch(cv_bridge::Exception &e)
  {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }
  cv::Mat cvimg_from_camera1_c = cv_ptr_c1->image;
  cv::Mat cvimg_from_camera2_c = cv_ptr_c2->image;
  cv::Mat cvimg_from_camera1_d = cv_ptr_d1->image;
  cv::Mat cvimg_from_camera2_d = cv_ptr_d2->image;

  cv::Mat combined_c_image(cvimg_from_camera1_c.rows + cvimg_from_camera2_c.rows, cvimg_from_camera2_c.cols, cvimg_from_camera1_c.type());
  cv::Mat combined_d_image(cvimg_from_camera1_d.rows + cvimg_from_camera2_d.rows, cvimg_from_camera2_d.cols, cvimg_from_camera1_d.type());


  // Publish the combined image
  sensor_msgs::msg::Image::SharedPtr c_image_msg
              = cv_bridge::CvImage(camera1_c_msg->header, "bgr8", combined_c_image).toImageMsg();
  sensor_msgs::msg::Image::SharedPtr d_image_msg
              = cv_bridge::CvImage(camera1_d_msg->header, "16UC1", combined_d_image).toImageMsg();
  combined_c_image_->publish(*c_image_msg);
  combined_d_image_->publish(*d_image_msg);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RSImageStitcher>());
  rclcpp::shutdown();
  return 0;
}