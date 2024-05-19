from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory

# launch 파일을 생성하는 함수
def generate_launch_description():
    # ARGS 정의
    share_dir = get_package_share_directory('hive_detection')
    model_path = os.path.join(share_dir, 'models', 'best.pt') # 모델 경로
    model = LaunchConfiguration("model")
    model_cmd = DeclareLaunchArgument(
        "model",
        default_value=model_path,
        description="Model name or path"
    )

    tracker = LaunchConfiguration("tracker")
    tracker_cmd = DeclareLaunchArgument(
        "tracker",
        default_value="bytetrack.yaml",  # 트래커 설정 파일
        description="Tracker name or path"
    )

    device = LaunchConfiguration("device")
    device_cmd = DeclareLaunchArgument(
        "device",
        default_value="cuda:0",  # 실행 장치
        description="Device to use (GPU/CPU)"
    )

    enable = LaunchConfiguration("enable")
    enable_cmd = DeclareLaunchArgument(
        "enable",
        default_value="True",  # 모델 활성화 / 비활성화
        description="Whether to start YOLOv8 enabled"
    )

    threshold = LaunchConfiguration("threshold")
    threshold_cmd = DeclareLaunchArgument(
        "threshold",
        default_value="0.2",  # 최소 확률 임계값
        description="Minimum probability of a detection to be published"
    )

    input_image_topic = LaunchConfiguration("input_image_topic")
    input_image_topic_cmd = DeclareLaunchArgument(
        "input_image_topic",
        default_value="/camera/camera/color/image_raw",  # 입력 이미지 토픽
        description="Name of the input image topic"
    )

    input_depth_image_topic = LaunchConfiguration("input_depth_image_topic")
    input_depth_image_topic_cmd = DeclareLaunchArgument(
        "input_depth_image_topic", 
        default_value="/camera/camera/depth/image_rect_raw",  # 입력 이미지 토픽
        description="Name of the input_depth_image_topic"
    )

    image_reliability = LaunchConfiguration("image_reliability")
    image_reliability_cmd = DeclareLaunchArgument(
        "image_reliability",
        default_value="2",  # 입력 이미지 토픽의 신뢰성 수준
        choices=["0", "1", "2"],
        description="Specific reliability QoS of the input image topic (0=system default, 1=Reliable, 2=Best Effort)"
    )

    namespace = LaunchConfiguration("namespace")
    namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value="hive_yolo",  # 네임스페이스
        description="Namespace for the nodes"
    )
    
    # 실행할 노드 설정
    detector_node_cmd = Node(
        package="yolov8_ros",
        executable="yolov8_node",
        name="yolov8_node",
        parameters=[{
            "model": model,
            "device": device,
            "enable": enable,
            "threshold": threshold,
            "image_reliability": image_reliability,
        }],
        remappings=[
            ("camera/rgb/image_raw", input_image_topic),
            ("detections", "hive_yolo/detections")]
    )

    mytracking_node_cmd = Node(
        package="hive_detection",  # 패키지 이름 확인
        executable="hive_tracking_node",  # 실행 파일 이름
        name="hive_tracking_node",
        parameters=[{
            "model": model,
            "tracker": tracker,
            "device": device,
            "enable": enable,
            "threshold": threshold,
            "image_reliability": image_reliability
        }],
        remappings=[
            ("tracking", "hive_yolo/tracking"),
            ("detection_info", "hive_yolo/detection_info"),
            ("camera/camera/color/image_raw", input_image_topic),
            ("camera/camera/depth/image_rect_raw", input_depth_image_topic),
        ]
    )

    debug_node_cmd = Node(
        package="yolov8_ros",
        executable="debug_node",
        name="debug_node",
        namespace=namespace,
        parameters=[{"image_reliability": image_reliability}],
        remappings=[
            ("image_raw", input_image_topic),
            ("detections", "tracking")
        ]
    )

    tracking_node_cmd = Node(
        package="yolov8_ros",
        executable="tracking_node",
        name="tracking_node",
        namespace=namespace,
        parameters=[{
            "tracker": tracker,
            "image_reliability": image_reliability
        }],
        remappings=[("image_raw", input_image_topic),]
    )

    
    

    # LaunchDescription 객체 생성 및 실행할 명령 추가
    ld = LaunchDescription()

    ld.add_action(model_cmd)
    ld.add_action(tracker_cmd)
    ld.add_action(device_cmd)
    ld.add_action(enable_cmd)
    ld.add_action(threshold_cmd)
    ld.add_action(input_image_topic_cmd)
    ld.add_action(input_depth_image_topic_cmd)
    ld.add_action(image_reliability_cmd)
    ld.add_action(namespace_cmd)

    ld.add_action(mytracking_node_cmd)  # 노드 실행 명령 추가
    ld.add_action(detector_node_cmd)
    ld.add_action(debug_node_cmd)
    ld.add_action(tracking_node_cmd)

    return ld

