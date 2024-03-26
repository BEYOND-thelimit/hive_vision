# hive_vision

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
