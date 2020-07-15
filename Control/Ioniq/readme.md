# Ioniq Control
Ioniq 차량의 제어 유틸리티를 개발하는 파트입니다.  
_Author: Seunggi Lee_  
__Date: 2020.07.14__  

## Ioniq과 컴퓨터 연결 방법(수동 version)  
Ioniq의 CAN 통신은 ROS1으로 짜여있고, 제어 코드는 ROS2로 짜여있기 때문에 제어를 하기 위해서는 ros bridge를 꼭 켜야한다.  
하지만 차량과의 연결만 필요하고, 제어가 필요없다면 ros1만 켜도 무방하다.
1. ros1을 실행한다.
2. ros2를 실행한다.(제어 필요없으면 넘어가도 됨.)
3. ros bridge를 실행한다.(제어 필요없으면 넘어가도 됨.)
```
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics 
```
4. cmd 창에 다음 명령어 4개를 입력한다.
```
sudo modprobe can
sudo modprobe kvaser_usb
sudo ip link set can0 type can bitrate 500000
sudo ifconfig can0 up
```
5. 다음 링크에서 dbw_ioniq_v2_release(Socketcan)와 ros_canopen.zip을 다운받는다. [링크](https://github.com/DGIST-ARTIV/ARTIV_Communication/tree/master/dbw_ioniq)
6. catkin_ws/src에 dwb_ioniq_v2_release와 ros_canopen을 넣는다.
7. catkin_ws에서 catkin_make를 한다.
8. catkin_make가 성공적으로 되면, 다음 명령어를 입력한다.
```
roslaunch dbw_ioniq_bridge dbw_ioniq_bridge.launch
```
9. 오류가 없으면 통신 끝!
10. 오류가 생기면 호영이에게 문의바람.


## Ioniq과 컴퓨터 연결 방법(자동 verison)
1. 다음 링크에서 dbw_ioniq_v2_release(Socketcan)와 ros_canopen.zip을 다운받는다. [링크](https://github.com/DGIST-ARTIV/ARTIV_Communication/tree/master/dbw_ioniq)
2. dbw_ioniq_v2_release와 ros_canopen을 catkin_ws에 넣고 catkin_make를 한다.
3. dbw_ioniq_v2_release에서 run_modeprobe.sh를 실행한다.
4. 오류가 없으면 통신 끝!
5. 오류 생기면 호영이에게 문의바람.


## Ioniq의 제어 유틸리티 사용 방법  
ERP42와 마찬가지로 IONIQ의 PID 제어 코드는 ROS2로 짜여있다.   
1. IONIQ과 컴퓨터를 연결하여 통신이 되는지 확인한다.
2. 다음 링크에서 pid_viewer_0707.py, pid_viewer_0707.ui를 다운받는다. [링크](./pid_ui/0707)
3. ros2를 실행시키고 pid_viewer_0707.py 파일을 실행시킨다.
4. k_p = 14.5, k_i = 0.25, k_d = 1, Anti_windup_guard = 70으로 맞출 경우 대략적인 pid 제어가 되는 것을 확인할 수 있다.
5. 만약 current_speed와 desired_speed를 그래프로 나타내어 비교하고 싶은 경우, 다음 링크에서 pid_graph_0707.py를 다운 받고 실행시키면 된다. [링크](./pid_graph_ioniq)

## Ioniq 제어 유틸리티 설명
**1. 사용하는 주요 Topic에 대한 설명**

![Screenshot from 2020-07-15 23-02-03](https://user-images.githubusercontent.com/59784459/87555065-e930e300-c6ef-11ea-911b-481603ca68a4.png)

**2. 사용하는 주요 Node에 대한 설명**

![Screenshot from 2020-07-15 23-05-55](https://user-images.githubusercontent.com/59784459/87555071-ec2bd380-c6ef-11ea-84a8-07cd2c93a08b.png)

![Screenshot from 2020-07-15 23-06-10](https://user-images.githubusercontent.com/59784459/87555074-ed5d0080-c6ef-11ea-8b85-9d7bd937ed92.png)

dbw_ioniq_node와 dbw_cmd_node에 더 자세히 알고 싶다면? 다음 링크를 참조해주세요.  
[링크](https://docs.google.com/document/d/1Mvyvs1Tt20U99uA4o_h4c2-KB7s64NOQz6vd_-SGwh4/edit?usp=sharing)  

**3. RQT**
![Screenshot from 2020-07-15 23-06-23](https://user-images.githubusercontent.com/59784459/87555082-efbf5a80-c6ef-11ea-821c-384742312e84.png)

## Ioniq 제어 결과 설명 
다음 링크를 참고해주세요~  
[링크](./pid_ui)
