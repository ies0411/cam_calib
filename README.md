# Project Title

Camera Intrinsic Parameter


## Index
### CPP TYPE
  - [Dependancy](#Dependancy) 
  - [Install](#Install)
  - [Parameter](#Parameter)
  - [Demo](#Demo)
  - [Results](#Results)

---
## CPP TYPE

### Dependancy

- ROS melodic ver
- Opencv 3.4 이상
- c++17

## Install

1. 직접세팅
```
** TODO
```

2. Docker 사용
 Docker 설치 및 Nvidia docker Image도 설치필요
```
$ sudo apt-get install x11-xserver-utils
$ xhost +

$ docker pull authorsoo/px4:9.0
$ docker run --gpus all -it --ipc=host  --expose 22 --net=host --privileged -e DISPLAY=unix$DISPLAY 
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw -e NVIDIA_DRIVER_CAPABILITIES=all --name calib authorsoo/px4:9.0 bash

```

## Parameter
rosrun 이 아닌 launch 파일로 실행시길 경우 직접 파라미터 세팅
![launch file](https://user-images.githubusercontent.com/44966311/168318213-5733af5c-1723-4fea-98d8-90f480533510.png)
```
$ roslaunch mono_cam_calib mono_calib
```
## Demo
1. Dataset 이용
[Dataset](https://drive.google.com/a/tamu.edu/file/d/19Ke-oOhqkPKJBACmrfba4R5-w71_wrvT/view?usp=sharing)


도커 이용시 /home 디렉토리에 파일 존재함
```
$ rosrun mono_cam_calib mono_cam_calib
$ rosbag play [rosbag 파일명] // rosbag실행
```

[Demo Video](https://www.notion.so/superbai/Intrinsic-Calib-in-Gazebo-c4d0ac30e01f42908ce5e682d0faa9e9)

## Results
result폴더 Txt파일에 intrinsic, 왜곡, R, T 값들 

## Ref

---
## PYTHON Type
** TODO
