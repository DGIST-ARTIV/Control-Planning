# How to install CARLA simulator and Run Carla, Carla-ROS-bridge
## Building CARLA
확인할것!
1. Ubuntu 버전확인: Ubuntu 18.04인지 확인하자. Ubuntu 16.04에서는 기본 컴파일러로인해 동작 불가능
2. 50GB정도의 여유공간이 있는지 확인하자.
3. 4GB 이상의 GPU(server)
4. 2개의 TCP ports와 좋은 인터넷상태(client).
### Carla Server download
Carla는 서버에서 World를 만들고 Client가 서버에 접속하는 방식이다. 
서버를 열기위해 Carla를 설치해보자.
```
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 92635A407F7A020C
sudo add-apt-repository "deb [arch=amd64 trusted=yes] http://dist.carla.org/carla-0.9.9/ all main"
```
Calra가 /opt폴더에 설치되었는지 확인!!
```
sudo apt-get update
sudo apt-get install carla-simulator
cd /opt/carla-simulator
```
Carla설치가 완료되었다!
```
./CarlaUE4.sh
```
Carla가 실행되는지 확인하고 wasd키를 이용해 세계를 돌아다녀 보자
https://carla.readthedocs.io/en/latest/start_quickstart/#running-carla
위 문서에 자세한 Command-line option이 있으니 참고.
### Carla-ROS bridge
자세한 문서는 https://carla.readthedocs.io/en/latest/ros_installation/ 참고.
#### Installation
ROS Kinetic or Melodic 버전과 Carla 0.9.7이후 버전에서는 ROS bridge를 활용할 수 있다.
artiv에서는 Melodic 버전을 사용중이니 다음 명령을 통해 ROS bridge를 설치하자
```
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 81061A1A042F527D
sudo add-apt-repository "deb [arch=amd64 trusted=yes] http://dist.carla.org/carla-ros-bridge-melodic/ bionic main"
sudo apt update
sudo apt install carla-ros-bridge-melodic

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
```
3번 명령을 실행하면 자동으로 pygame이 실행되면서 자동차를 운전할 수 있다!
