
# How to install CARLA simulator and Run Carla, Carla-ROS-bridge
Carla Git Book, please click : https://app.gitbook.com/@artiv/s/carla-manual/
## Building CARLA(install)
확인할것!
1. Ubuntu 버전확인: Ubuntu 18.04인지 확인하자. Ubuntu 16.04에서는 기본 컴파일러로인해 동작 불가능
2. 50GB정도의 여유공간이 있는지 확인하자.
3. 4GB 이상의 GPU(server)
4. 2개의 TCP ports와 좋은 인터넷상태(client).
### Carla Server download
Carla는 서버에서 World를 만들고 Client가 서버에 접속하는 방식이다. 
서버를 열기위해 Carla를 설치해보자.
!!!만약 이미 열린 서버를 통해 접속할 경우 이 과정은 생략하고 서버에 접속하기위해 아래 carla-ros-bridge를 설치하도록 한다.

```
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 92635A407F7A020C
sudo add-apt-repository "deb [arch=amd64 trusted=yes] http://dist.carla.org/carla-0.9.9/ all main"
```

```
sudo apt-get update
sudo apt-get install carla-simulator
cd /opt/carla-simulator
```
Calra가 /opt폴더에 설치되었는지 확인!!
Carla설치가 완료되었다!
```
cd bin
./CarlaUE4.sh
```
Carla가 실행되는지 확인하고 wasd키를 이용해 세계를 돌아다녀 보자
https://carla.readthedocs.io/en/latest/start_quickstart/#running-carla
위 문서에 자세한 Command-line option이 있으니 참고.
### Carla-ROS bridge
자세한 문서는 https://carla.readthedocs.io/en/latest/ros_installation/ 참고.
#### Installation carla-ros-bridge(/opt)
ROS Kinetic or Melodic 버전과 Carla 0.9.7이후 버전에서는 ROS bridge를 활용할 수 있다.
artiv에서는 Melodic 버전을 사용중이니 다음 명령을 통해 ROS bridge를 설치하자
```
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 81061A1A042F527D
sudo add-apt-repository "deb [arch=amd64 trusted=yes] http://dist.carla.org/carla-ros-bridge-melodic/ bionic main"
sudo apt update
sudo apt install carla-ros-bridge-melodic
```
#### carla-ros-bridge(/home)
Artiv개발자 : https://github.com/DGIST-ARTIV/Control-Planning/releases/tag/1.0 에서 carla-ros-bridge를 다운받은 뒤 /home에 압축해제
```
정석 설치 방법 위에서 Release된 파일을 다운받은경우 안하시면됩니다.
mkdir -p ~/carla-ros-bridge/catkin_ws/src
cd ~/carla-ros-bridge
git clone https://github.com/carla-simulator/ros-bridge.git
cd ros-bridge
git submodule update --init
cd ../catkin_ws/src
ln -s ../../ros-bridge
source /opt/ros/melodic/setup.bash  
cd ..

#install required ros-dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r

#build
catkin_make
```
#### Run

1. Run carla
```
cd /opt/carla-simulator/bin
./CarlaUE4.sh
```
2. Source
```
source /opt/carla-ros-bridge/melodic/setup.bash
source ~/carla-ros-bridge/catkin_ws/devel/setup.bash
```
위 명령어는 alias 하는것을 권장한다.
3. start ROS bridge
```
# Option 1: start the ros bridge
roslaunch carla_ros_bridge carla_ros_bridge.launch

# Option 2: start the ros bridge together with RVIZ
roslaunch carla_ros_bridge carla_ros_bridge_with_rviz.launch

# Option 3: start the ros bridge together with an example ego vehicle
roslaunch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch

only for Release version
# Option 4: start the ros bridge and Artiv Master(Enbin Server)
roslaunch carla_ros_bridge carla_ros_bridge_online_server.launch

# Option 5: start the ros brige and Artiv Master(Local Server)
roslaunch carla_ros_bridge carla_ros_brige_local_server.launch

```
3번 명령을 실행하면 자동으로 pygame이 실행되면서 자동차를 운전할 수 있다!

online server를 활용하여 carla를 실행할 수 있다. 
이미 carla를 설치한 경우, local server를 통해 접속할 수 있다. 

### For Artiv Carla user
carla를 활용하여 개발을 하게될 Artiv Developer들을 위해서...
#### Run carla-ros-bridge
```
source /opt/carla-ros-bridge/melodic/setup.bash
source ~/carla-ros-bridge/catkin_ws/devel/setup.bash
source /opt/ros/melodic/setup.bash

roslaunch carla_ros_bridge carla_ros_bridge_online_server.launch
```

#### Carla ros topics and msg for Developer

Artiv에서 사용하는 Ros code 개발을 위한 Topic and msgs : https://docs.google.com/document/d/1Q3B6fCE7Mvl5EyOgdGUf9F7UBtlyKkRuPpDVOXJluRs/edit?usp=sharing
Carla에 대해 더 자세한 Topic Info를 알고싶은신 분들은 : https://carla.readthedocs.io/en/latest/ros_msgs/ 참고^^
