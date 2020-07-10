# Install_CARLA
## Building CARLA
확인할것!
1. Ubuntu 버전확인: Ubuntu 18.04인지 확인하자. Ubuntu 16.04에서는 기본 컴파일러로인해 동작 불가능
2. 50GB정도의 여유공간이 있는지 확인하자.
3. 4GB 이상의 GPU
4. 2개의 TCP ports와 좋은 인터넷상태.
### Dependencies
CARLA를 실행시키기 위한 Dependencies download

```
sudo apt-get update
sudo apt-get install wget software-properties-common
sudo add-apt-repository ppa:ubuntu-toolchain-r/test
wget -O - https://apt.llvm.org/llvm-snapshot.gpg.key|sudo apt-key add -
sudo apt-add-repository "deb http://apt.llvm.org/xenial/ llvm-toolchain-xenial-8 main"
sudo apt-get update

sudo apt-get install build-essential clang-8 lld-8 g++-7 cmake ninja-build libvulkan1 python python-pip python-dev python3-dev python3-pip libpng-dev libtiff5-dev libjpeg-dev tzdata sed curl unzip autoconf libtool rsync libxml2-dev
pip2 install --user setuptools && pip3 install --user setuptools 

```
CARLA dependencies와 Unreal Engine사이의 호환성 문제를 피하기 위해서 동일한 컴파일러 버전과 C++runtime library를 사용하는것을 권장한다. CARLA에서는 lang-8 and LLVM's libc++를 사용한다. Unreal Engine과 CARLA dependencies를 컴파일하기 위해 default clang version을 바꾸자
```
sudo update-alternatives --install /usr/bin/clang++ clang++ /usr/lib/llvm-8/bin/clang++ 180 && sudo update-alternatives --install /usr/bin/clang clang /usr/lib/llvm-8/bin/clang 180
```
### Unreal Engine
Unreal Engine을 다운로드하기 위해 먼저 https://www.unrealengine.com/ko/ 에 접속하여 회원가입한 후 -> connection -> acount로 들어가서 github계정을 연결해준다.

Unreal Engine 설치
```
git clone --depth=1 -b 4.24 https://github.com/EpicGames/UnrealEngine.git ~/UnrealEngine_4.24
```



