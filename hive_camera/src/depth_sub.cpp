#include <cstring>
#include <iostream>
#include <string>
#include <cmath>
#include <sstream>

#include "std_msgs/msg/float32.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/opencv.hpp>

class DepthMapHandler : public rclcpp::Node
{
public:
  DepthMapHandler() : Node("depth_map")
  {
    depth_image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
    "/camera1/depth/image_raw", 10,
    std::bind(&DepthMapHandler::depthImageCallback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&DepthMapHandler::publishMap, this));
    
    image_width = 640;
    image_height = 480;
    map_matrix = cv::Mat::zeros(image_height, image_width, CV_8UC1);
  }

private:
  int image_width;
  int image_height;
  cv::Mat map_matrix;

  void depthImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv::Mat depth_image(image_height, image_width, CV_32FC1);
    memcpy(depth_image.data, msg->data.data(), msg->data.size());
    cv::Mat depth_matrix = cv::Mat::zeros(image_height, image_width, CV_8UC1);

    for (int y = 0; y < image_height; ++y) {
      for (int x = 0; x < image_width; ++x) {// depth 이미지에서 해당 픽셀의 z 값 가져오기 
        float z_value = depth_image.at<float>(y, x);  
        if (z_value < 2.3) {// z 값이 2.3보다 작으면 행렬의 해당 위치를 1로 설정
            depth_matrix.at<uchar>(y, x) = 255;
        }
      }
    }

    cv::Mat labels, stats, centroids;
    int num_labels = cv::connectedComponentsWithStats(depth_matrix, labels, stats, centroids,CV_32S);
    cv::Mat labels_8uc1;
    labels.convertTo(labels_8uc1, CV_8UC1);
    
    // 이미지에서 각 장애물들을 색상으로 표시하기 위한 색상 목록 
    std::vector<cv::Scalar> colors = {
        cv::Scalar(255, 0, 0),   // 파랑
        cv::Scalar(0, 255, 0),   // 초록
        cv::Scalar(0, 0, 255),   // 빨강
        cv::Scalar(255, 255, 0), // 시안
        cv::Scalar(255, 0, 255), // 자홍
        cv::Scalar(0, 255, 255)  // 노랑
    };

    // 레이블마다 색상을 적용(그룹별 색상 적용)
    cv::Mat colored_labels;
    cv::cvtColor(labels_8uc1, colored_labels, cv::COLOR_GRAY2BGR); // 8비트 단일 채널을 3채널로 변환

    for (int label = 1; label <= num_labels; ++label) {
        cv::Mat mask = (labels == label); // 현재 레이블에 해당하는 픽셀을 마스킹
        cv::Scalar color = colors[label % colors.size()]; // 레이블에 해당하는 색상을 선택
        colored_labels.setTo(color, mask); // 해당 레이블에 색상을 적용
    }

    // 결과 이미지를 표시 (로봇 & 장애물 모두 표시된 이미지)
    cv::imshow("Depth Image_1", colored_labels);

    cv::Mat labels_copy = labels.clone();    
    //로봇 별 좌표 저장
    int robot1_value_x=10;
    int robot1_value_y=460;
    int robot2_value_x=150;
    int robot2_value_y=460;
    int robot3_value_x=300;
    int robot3_value_y=460;

    int target_value_1 = labels_copy.at<int>(robot1_value_y, robot1_value_x);
    int target_value_2 = labels_copy.at<int>(robot2_value_y, robot2_value_x);
    int target_value_3 = labels_copy.at<int>(robot3_value_y, robot3_value_x);

    int x_1 = stats.at<int>(target_value_1, cv::CC_STAT_LEFT);
    int y_1 = stats.at<int>(target_value_1, cv::CC_STAT_TOP);
    int width_1 = stats.at<int>(target_value_1, cv::CC_STAT_WIDTH);
    int height_1= stats.at<int>(target_value_1, cv::CC_STAT_HEIGHT);

    int x_2 = stats.at<int>(target_value_2, cv::CC_STAT_LEFT);
    int y_2 = stats.at<int>(target_value_2, cv::CC_STAT_TOP);
    int width_2 = stats.at<int>(target_value_2, cv::CC_STAT_WIDTH);
    int height_2 = stats.at<int>(target_value_2, cv::CC_STAT_HEIGHT);

    int x_3 = stats.at<int>(target_value_3, cv::CC_STAT_LEFT);
    int y_3 = stats.at<int>(target_value_3, cv::CC_STAT_TOP);
    int width_3 = stats.at<int>(target_value_3, cv::CC_STAT_WIDTH);
    int height_3 = stats.at<int>(target_value_3, cv::CC_STAT_HEIGHT);
    
    
    // 로봇으로 추정된 바운딩 박스 영역을 픽셀 0으로 설정
    for (int row = y_1; row < y_1 + height_1; ++row){
      for (int col = x_1; col < x_1 + width_1; ++col){
        if (labels_copy.at<int>(row, col) == target_value_1) {
            labels.at<int>(row, col) = 0;
        }
      }
    }
    for (int row = y_2; row < y_2 + height_2; ++row){
      for (int col = x_2; col < x_2 + width_2; ++col){
        if (labels_copy.at<int>(row, col) == target_value_2) {
            labels.at<int>(row, col) = 0;
        }
      }
    }
    for (int row = y_3; row < y_3 + height_3; ++row){
      for (int col = x_3; col < x_3 + width_3; ++col){
        if (labels_copy.at<int>(row, col) == target_value_3) {
            labels.at<int>(row, col) = 0;
        }
      }
    }
    labels.convertTo(labels_8uc1, CV_8UC1);

    // 랜덤한 색상 맵 생성
    cv::cvtColor(labels_8uc1, colored_labels, cv::COLOR_GRAY2BGR); // 8비트 단일 채널을 3채널로 변환합니다.

    for (int label = 1; label <= num_labels; ++label) {
        cv::Mat mask = (labels == label); // 현재 레이블에 해당하는 픽셀을 마스킹합니다.
        cv::Scalar color = colors[label % colors.size()]; // 레이블에 해당하는 색상을 선택합니다.
        colored_labels.setTo(color, mask); // 해당 레이블에 색상을 적용합니다.
    }

    cv::imshow("Depth Image_2", colored_labels);

    int threshold_value = 0; // 임계값
    cv::threshold(labels_8uc1, map_matrix, threshold_value, 1, cv::THRESH_BINARY);
    cv::Mat k = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(50, 50));
    cv::dilate(map_matrix, map_matrix, k);//팽창 연산 적용
    
    map_matrix.at<unsigned char>(robot1_value_y, robot1_value_x) = 2;
    map_matrix.at<unsigned char>(robot2_value_y, robot2_value_x) = 3;
    map_matrix.at<unsigned char>(robot3_value_y, robot3_value_x) = 4;

    cv::waitKey(1);
  }
  
  void publishMap()
  {
    auto occupancy_grid_msg = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    occupancy_grid_msg->info.width = image_width;
    occupancy_grid_msg->info.height = image_height;
    occupancy_grid_msg->info.resolution = 1.0; // 임의의 해상도 설정 각 셀마다 1M*1M라는 의미
    occupancy_grid_msg->data.resize(image_width * image_height);
    
    for (int i = 0; i < map_matrix.rows * map_matrix.cols; ++i){
      occupancy_grid_msg->data[i] = map_matrix.data[i];
    } 
    publisher_->publish(*occupancy_grid_msg);

    cv::threshold(map_matrix, map_matrix, 0, 255, cv::THRESH_BINARY); // 최종 맵이 잘 나타나는 지 확인용 이미지
    cv::imshow("Depth Image_4", map_matrix);
  }
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_subscription_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DepthMapHandler>());
  rclcpp::shutdown();
  return 0;
}