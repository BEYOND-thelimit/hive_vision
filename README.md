# hive_vision
This is our vision system implementation. A camera which we use is **on the ceiling**.
For more details, Refer to [our full documentation](https://github.com/BEYOND-thelimit/Capstone-Design-at-HYU).

---

### ⚠️ License & Patent Notice

> This repository is provided **for non-commercial academic and research use only**.
> **Commercial use is strictly prohibited** without a separate license agreement.

- This project includes technology that is **patent-pending in South Korea**
  (Korean Patent Application No. **10-2025-0008102**)
- If you use this code or its ideas in your research or projects,
  **you must provide proper attribution** to the original author.
- **Use of this software or its ideas for commercial purposes is not allowed.**

For commercial licensing inquiries, please contact:
**[taehun-ryu@gmail.com]**

---

## Prerequests
- [ultralytics](https://github.com/ultralytics/ultralytics)
- [our custom yolov8-ros driver](https://github.com/taehun-ryu/yolov8_ros)
- [realsense2-ros](https://github.com/IntelRealSense/realsense-ros.git)

Our detection code needs the above repository.

## How to run
Turn on realsense camera
```
ros2 launch realsense2_camera rs_launch.py
```
**Color format should be BGR8!!**

Then, Run YOLOv8
Our best.pt is in */hive_vision/hive_detection/models/best.pt*
```
ros2 launch hive_detection hive_yolov8.launch.py model:=/path/to/your_best.pt
```

For making depth map,
```
ros2 launch hive_camera depth_measurement.launch.py
```

## docker
We provide Dockerfile. There are some prerequisites to build successfully.

- nvidia-docker2
- X11 forwarding
- Add docker in sudo group

Just build image using below command. It takes about 15-20 minutes.
```
docker build -t [YourImageName:TAG] .
```
Then, you can use `run_docker.sh` to run Docker container.
```
sh run_docker.sh [YourImageName:TAG] [YourContainerName]
```
If you contribute our Dockerfile for better way(reducing time to build image or installing useful packages, etc.), just feel free to Pull Requests.
