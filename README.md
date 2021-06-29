# Control_Planning
Control_Planning 파트는 차량의 주행 경로 알고리즘(path-planning algorithm)과 제어를 담당하는 파트입니다.

<img src="https://user-images.githubusercontent.com/59792475/87428588-840cbd00-c61d-11ea-8bbb-6b7a2d56f91a.png" align="center" width="40%" height="40%"/></img>

1. ERP42의 통신 방법과 제어 방법 [링크](./Control/ERP42)
2. IONIQ의 통신 방법과 제어 방법 [링크](./Control/Ioniq)
3. Path_planning과 Simulator [링크](./Planning)

## 개괄
![슬라이드1](https://user-images.githubusercontent.com/59792475/87427806-4c514580-c61c-11ea-8115-62d565470eb7.JPG)

## Path Planning
### Summery
 - State Lattice와 MPTG(Model Predictive Trajectory Generator)를 활용한 Local Path Planner를 제작했다.
 - Lidar(RS Lidar or VLP-16)을 활용해 Obstacles Detection 하였으며[Lidar](https://github.com/DGIST-ARTIV/Lidar), MPTG를 활용하여 Lane State Sampling을 했다.
 ### Algorithm

  - MPTG
  - State Lattice
  - Dynamic Obstacle Avoidance
  - move to pose

 ### Result

 #### Demo Video
 ![demo](/images/Ioniq_test.gif)
#### Youtube Videos
 [![ERP video](https://img.youtube.com/vi/E_8HAf1OwA8/0.jpg)](https://www.youtube.com/watch?v=E_8HAf1OwA8) | [![Ioniq video](https://img.youtube.com/vi/KWBubw8ciBU/0.jpg)](https://www.youtube.com/watch?v=KWBubw8ciBU) 
  :-------------------------:|:-------------------------:

#### Figure

 ![](/images/circle.png ) | ![](/images/straight-1cone.png )
 :-------------------------:|:-------------------------:
 ![](/images/straight-2cone.png) | ![](/images/straight-car.png)

This experiment conducted with **Hyundae Ioniq**



 ## Control
 ### Summery
  - PID제어를 활용한 Ioniq 종방향 속도제어
  - Lidar를 활용한 SCC(Smart Cruise Control)
  - AEB(Auto Emergency Braking)
  - Pure Pursuit, Stanly Method를 활용한 Lane Keeping Assistance System(LKAS), Global Path Following(GPS기반)
  - Move to Pose Control : Simple Geometric Control -> For Parking and MPTG

### Algorithm
 - PID
 - Stanly
 - Pure Pursuit
 - Move to Pose
 - Smart Cruise Control
 - Auto Emergency Braking
 - Lane Keeping System
### Result

#### PID Control
[링크](/Control/Ioniq/pid_ui)

#### SCC

#### Lane Keeping System
[Hybrid Tracker Based Optimal Path Tracking System of Autonomous Driving for Complex Road Environments](https://ieeexplore.ieee.org/document/9427137?source=authoralert)
#### AEB
![](/images/AEB_Good.png)

#### Lane Keeping System & Smart Cruise Control
[**Blog Link**]( https://dgist-artiv.github.io/blog/2021/02/25/LKS_cruise_test.html)
#### Youtube Video
[![video](https://img.youtube.com/vi/5POMPtsQw7Y/0.jpg)](https://www.youtube.com/watch?v=5POMPtsQw7Y)

