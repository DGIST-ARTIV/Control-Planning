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
4. 해당 링크에 있는 것들을 모두 다운 받는다.(dbw_erp42_node, dbw_erp42_bridge, dbw_cmd_erp42)  
[/dbw_erp42_v2_release(2020)](https://github.com/DGIST-ARTIV/ARTIV_Communication/tree/master/dbw_erp42/dbw_erp42_v2_release(2020))
5. 다운 받은 후, scripts에 있는 dbw_erp42_node.py와 dbw_cmd_erp42.py을 실행한다. (각각 다른 터미널 창에서 실행시켜야 함.)
6. 위의 두 파일을 실행하고 ERP42 mode를 manual에서 auto로 변경했을 때 "MODE : AUTO"라고 뜨면 ERP42와 통신이 완료된다.

## ERP42의 제어 방법  
기본적으로 ERP42의 PID 제어 코드는 ROS2로 짜여있다. 따라서 통신을 ROS1버전으로 사용할 경우, ros2 run ros1_bridge dynamic_bridge --bridge-all-topics 명령어를 실행시켜야 한다.
1. 
