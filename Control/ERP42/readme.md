# ERP42 Control
ERP42 차량의 제어 유틸리티를 개발하는 파트입니다.  
_Author: Seunggi Lee_  
__Date: 2020.07.11__  

## ERP42와 컴퓨터 연결 방법(for ROS2)
1. ERP42 좌측 하단에 RS232 to USB 케이블을 찾는다.
2. RS232 포트는 ERP42에 연결, USB 포트는 컴퓨터에 연결한다. 
3. 터미널 창에 다음 명령어를 입력한다.
```
sudo chmod 666 /dev/ttyUSB0
```
4. 해당 링크에 있는 폴더 3개를 모두 다운 받는다.(dbw_erp42_node, dbw_erp42_bridge, dbw_cmd_erp42)  
[/dbw_erp42_ros2](https://github.com/DGIST-ARTIV/ARTIV_Communication/tree/master/dbw_erp42/dbw_erp42_ros2)
5. 폴더 3개를 다운 받은 후, dbw_erp42_node에 있는 dbw_erp42_node.py와 dbw_cmd_node.py을 실행한다. (각각 다른 터미널 창에서 실행시켜야 함.)
6. 위의 두 파일을 실행하고 ERP42 mode를 manual에서 auto로 변경했을 때 "MODE : AUTO"라고 뜨면 ERP42와 통신이 완료된다.

## ERP42와 컴퓨터 연결 방법(for ROS1)
1. ERP42 좌측 하단에 RS232 to USB 케이블을 찾는다.
2. RS232 포트는 ERP42에 연결, USB 포트는 컴퓨터에 연결한다. 
3. 터미널 창에 다음 명령어를 입력한다.(이 명령어는 cmd 창을 새로 킬 때마다 실행시켜야 ERP42와 연결이 됨.)
```
sudo chmod 666 /dev/ttyUSB0
```
4. 해당 링크에 있는 것들을 모두 다운 받는다.
[/dbw_erp42_v2_release(2020)](https://github.com/DGIST-ARTIV/ARTIV_Communication/tree/master/dbw_erp42/dbw_erp42_v2_release(2020))
5. 다운 받은 후, scripts에 있는 dbw_erp42_node.py와 dbw_cmd_erp42.py을 실행한다. (각각 다른 터미널 창에서 실행시켜야 함.)
6. 위의 두 파일을 실행하고 ERP42 mode를 manual에서 auto로 변경했을 때 "MODE : AUTO"라고 뜨면 ERP42와 통신이 완료된다.

### 만약 sudo chmod 666/dev/ttyUSB0 명령어를 사용하여도 연결이 되지 않는다면?
1. cmd 창에 ls /dev를 입력한다.
2. USB 포트 이름을 확인하고 만약 이름이 다르다면 그에 맞게 ttyUSB 뒤의 숫자를 변경해준다.
3. 이 과정이 번거롭다면, 다음 링크로 들어가서 UBUNTU에 연결되어 있는 device에 이름을 부여해보자. [링크](https://github.com/shinkansan/ARTIV/blob/master/Sensors/setting/readme.md)
4. 이래도 안된다? 알아서 잘 해보셈. ㅡㅡ

## ERP42의 제어 유틸리티 사용 방법  
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
