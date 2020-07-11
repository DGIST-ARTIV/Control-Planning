# ERP42 Control
ERP42 차량의 제어 유틸리티를 개발하는 파트입니다.  
_Author: Seunggi Lee  
__Date: 2020.07.11  

## ERP42와 컴퓨터 연결 방법
1. ERP42 좌측 하단에 RS232 to USB 케이블을 찾는다.
2. RS232 포트는 ERP42에 연결, USB 포트는 컴퓨터에 연결한다. 
3. 터미널 창에 다음 명령어를 입력한다.
```
sudo chmod 666 /dev/ttyUSB0
```
4. ROS 파일을 실행한다. (dbw_erp42_node.py, dbw_cmd_node.py 두 개의 python 파일 실행)

