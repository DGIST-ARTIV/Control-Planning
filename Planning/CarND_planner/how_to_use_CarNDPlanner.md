# Develog: How to use CarND Planner with term 3 simulator
Author: 서은빈 <br/>
Date: 2020.04.07

## Introduction
covid-19로 인해 모두가 재택근무를 하게 돼서 우리가 선정한 시뮬레이터 lgvsl를 사용하지 못하므로 udacity에서 제공하는 term 3 simulator를 이용하기로 했다. CarND planner를 simulator에 돌려보기로 했다.

## Setting
이 [link](https://github.com/udacity/CarND-Path-Planning-Project)를 참고하였다. <br/>

### 1. Simulator
1. 이 링크[release tab(https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2). 에 접속해 term3_sim_linux.zip을 다운 받는다.
2. term3_sim_linux.zip을 압축을 풀어준다.(extract)
3. 명령창을 키고 다음 명령어를 입력한다.
~~~(bash)
cd term3_sim_linux/
sudo chmod u+x term3_sim.x86_64
~~~
4. term3_sim.x86_64를 더블클릭하여 잘 들어가지나 확인한다.

### 2. CarND Planner
이 명령어를 통해 github에 있는 파일을 clone
~~~ (bash)
git clone https://github.com/udacity/CarND-Path-Planning-Project.git
~~~

### Dependencies
![dependencies](./media/CarND_planner_dependencies.PNG)
우리는 linux 기반이므로 uWebSockets만 잘 설정해주면 된다.
1) 방법 1: 권한을 주고 install-ubuntu.sh를 실행시킨다.
CarNd-Path-Planner-Project 파일에서 명령창을 키고 다음 명령어 입력
~~~ (bash)
sudo chmod +x install-ubuntu.sh
./install-ubuntu.sh
~~~

2) 방법 2: 다음 명령어들을 명령창에 입력한다.
~~~ (bash)
#! /bin/bash
sudo apt-get install libuv1-dev libssl-dev libz-dev
git clone https://github.com/uWebSockets/uWebSockets 
cd uWebSockets
git checkout e94b6e1
mkdir build
cd build
cmake ..
make 
sudo make install
cd ..
cd ..
sudo ln -s /usr/lib64/libuWS.so /usr/lib/libuWS.so
sudo rm -r uWebSockets
~~~

잘 설치가 되었는지 확인하기!
~~~(bash)
git clone https://github.com/uWebSockets/uWebSockets 
cd uWebSockets
git checkout e94b6e1
~~~

### Run CarND Planner
1. main.cpp 수정 및 spline.h 추가하기
- main.cpp 수정
main.cpp 내용을 모두 지우고 https://github.com/darienmt/CarND-Path-Planning-Project-P1/blob/master/src/main.cpp 이 링크에 있는 main.cpp를 붙여넣는다.
- spline.h 파일 만들기
위의 main.cpp에 spline.h를 사용하므로 파일을 만들어주어야한다. 위치는 src에 만들면 된다.
코드는 https://github.com/darienmt/CarND-Path-Planning-Project-P1/blob/master/src/spline.h 이 링크에 있다.

2. CarND-Path-Planning-Project 파일에 들어가서 명령창을 띄운 후 다음 작업실시.
~~~ (bash)
mkdir build&& cd build
cmake ..
make
./path_planning
~~~

2. 모두 실행한 후 simulator를 켜 자율주행을 하는지 확인

### 내가 겪은 Compile error
1. compile할 때 library 파트에서 오류
- uWebSockets와 관련된 오류 --> 다시 uWebSockets를 설정해준다.
- uv.h 와 관련된 library 오류가 난다. --> library version 확인이 필요 libuv 1.12.0으로 깔려 있는지 확인하고 아니라면 이 version으로 깔아주어야한다.
2. compile할 때 빨간 글씨 없이 다음과 같은 오류가 뜬다.
/path_planning: error while loading shared libraries: libuWS.so: cannot open shared object file: no such file or director <br/>
--> shared object file이 존재하는 directory가 library path에 설정되지 않았을 때 발생하는 것이다. 즉, 여기서는 libuWS가 library path에 설정되지 않음. 다음 command를 치면 해결
~~~(bash)
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib64
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib
~~~
앞으로도 다음과 같은 오류가 떴을 때는 library path에 directory가 추가되었는지 확인!!
