# We provide docker image for users using other OS systems.
If you have any questions, please contact @taehun-ryu.

## How to use
1. Build image using Dockerfile in `docker` folder. Because It downloads both `ros-foxy-desktop` and `ultralytics`, **it takes over 20 minutes**.
  ```
  cd docker
  docker build --no-cache --force-rm -t [YourImageName] .
  ```
  
2. Run the container as you wish. Also, We provide our container running method at `run_docker.sh`.
   If you want to keep our settings intact, enter the command below.
 ```
bash run_docker.sh [YourImageName] [YourContainerName]
 ```
