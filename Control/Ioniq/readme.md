# Ioniq Control
Ioniq 차량의 제어 유틸리티를 개발하는 파트입니다.  
_Author: Seunggi Lee_  
__Date: 2020.07.14__  

## Ioniq과 컴퓨터 연결 방법(수동 version)
1. 

## Ioniq과 컴퓨터 연결 방법(자동 verison)




## ERP42의 제어 방법  
기본적으로 ERP42의 PID 제어 코드는 ROS2로 짜여있다.   
따라서 통신을 ROS1 버전으로 사용할 경우,  
```
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics 
```
다음 명령어를 실행시켜야 한다.  
통신을 ROS2 버전으로 사용할 경우, 그냥 바로 사용하면 된다.
1. ERP42와 컴퓨터를 연결하여 통신이 되는지 확인한다.
2. 다음 링크에서 pid_viewer_erp42_0711.py, pid_viewer.ui를 다운받는다. [링크](./pid_ui_erp42/0711)
3. ros2를 실행시키고 pid_viewer_erp42_0711.py 파일을 실행시킨다.
4. k_p = 2.5, k_i = 0.1, k_d = 1, Anti_windup_guard = 40으로 맞출 경우 대략적인 pid 제어가 되는 것을 확인할 수 있다.
5. 만약 current_speed와 desired_speed를 그래프로 나타내어 비교하고 싶은 경우, 다음 링크에서 graph_visulization_erp42.py를 다운 받고 실행시키면 된다. [링크](./pid_graph_erp42/graph_visulization_erp42.py)
