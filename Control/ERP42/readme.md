# ERP42 Control
ERP42 차량의 제어 유틸리티를 개발하는 파트입니다.  
_Author: Seunggi Lee_  
__Date: 2020.07.11__  

## ERP42와 컴퓨터 연결 방법(ROS2)
1. ERP42 좌측 하단에 RS232 to USB 케이블을 찾는다.
2. RS232 포트는 ERP42에 연결, USB 포트는 컴퓨터에 연결한다. 
3. 터미널 창에 다음 명령어를 입력한다.
```
sudo chmod 666 /dev/ttyUSB0
```
4. ROS 파일을 실행한다. (dbw_erp42_node.py, dbw_cmd_node.py 두 개의 python 파일 실행) [링크](./ARTIV_communication/dbw_erp42/dbw_erp42_ros2)
5. 위의 두 파일을 실행하고 mode를 manual에서 auto로 변경했을 때 "MODE : AUTO"라고 뜨면 ERP42와 통신이 완료된다.

## ERP42의 제어 방법
1. 
