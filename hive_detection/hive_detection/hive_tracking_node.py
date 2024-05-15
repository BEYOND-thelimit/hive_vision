import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge
from yolov8_msgs.msg import Detection, DetectionArray

class MyTrackingNode(Node):
    def __init__(self) -> None:
        super().__init__('my_tracking_node')
        self.publisher = self.create_publisher(Float64MultiArray, 'detection_info', 10)
        self.cv_bridge = CvBridge()
        self.depth_image = None  # Depth image 초기화

    def depth_callback(self, depth_msg: Image):
        self.depth_image = np.array(self.cv_bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough'))
        # => depth 값 저장


    def detection_callback(self, detections_msg: DetectionArray):
        if self.depth_image is None:
            self.get_logger().warn('No depth image available')
            return
            #=> depth 값 들어왔는지 확인

        self.publish_detection_info(detections_msg)

    def camera_info_callback(self, camera_info_msg: CameraInfo):
        # Camera intrinsic parameters of D435i which we use
        # self.fx = 607.0402221679688
        # self.fy = 606.7127685546875
        # self.cx = 311.1837158203125
        # self.cy = 251.5524139404297

        # Camera intrinsic parameters from camera_info_msg
        self.fx = camera_info_msg.k[0]
        self.fy = camera_info_msg.k[4]
        self.cx = camera_info_msg.k[2]
        self.cy = camera_info_msg.k[5]

    def publish_detection_info(
        self,
        tracked_detections_msg: DetectionArray
        ) -> None:

        for tracked_detection in tracked_detections_msg.detections:
            u = int(tracked_detection.bbox.center.position.x)
            v = int(tracked_detection.bbox.center.position.y)


            try:
                Z = self.depth_image[v][u]
                Z_=Z/1000 # mm -> m 변환!
            except IndexError:
                self.get_logger().error(f'Index out of bounds: ({u}, {v}) not in depth image dimensions')
                continue
                #=> 여기서 계속 오류났는데 u,v 가 depth 좌표값에 포함되지 않는다고 떠서, depth 해상도가 848,480 , image 해상도가 1280,960 이길래 그냥 image 해상도 848,480 으로 depth 해상도랑 맞추니까 됐어.
            if Z == 0:
                # 깊이 값이 0이면 유효한 데이터가 없음
                continue

            X = (u - self.cx) * Z_ / self.fx 
            Y = (v - self.cy) * Z_ / self.fy

            detection_info_array = Float64MultiArray()
            detection_info_array.data = [float(tracked_detection.id), X, Y, tracked_detection.score]
            self.publisher.publish(detection_info_array)

def main():
    rclpy.init()
    node = MyTrackingNode()

    node.create_subscription(
        Image,
        'camera/camera/aligned_depth_to_color/image_raw',
        node.depth_callback,
        10
    )

    node.create_subscription(
        DetectionArray,
        'tracking',
        node.detection_callback,
        10
    )

    node.create_subscription(
        CameraInfo,
        'camera/camera/aligned_depth_to_color/camera_info',
        node.camera_info_callback,
        10
    )

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()