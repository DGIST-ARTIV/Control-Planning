## ARTIV HLSwitch
호영과 승기의 Vision, GPS 추종 알고리즘을 카메라의 전방 데이터를 이용하여 적절하게 사용하는 딥러닝 네트워크
왜 HL Switch 인지는 묻지 말자, 호(H) 이(L) 이긴 한데 성과 이름의 혼용인지는 방금 마크다운 작성하면서 깨달았다.

그래도 HSS 보다는 HLS 라고 부르는게 입에 착착 붙는다.

 ## dependencies
  1. tensorflow==1.14.0
  2. keras==2.3.0

  (텐서플로우 2만 아니면 된다.)

## How to run
`python3 HLSwitch_eval.py`


## How to Train
`python3 train.py`

## How to make dataset
dataset_raw 폴더에서 video 폴더에 영상을 넣고 `annotTool.py` 실행

GPS, VISION 폴더에 영상 이름과 순서를 가지고 생성된다. 그리고 이 폴더를 상위 디렉토리의 TrainData에 넣어서

위 파이썬 파일을 실행하면 알아서 된다. 자세한 파라미터는 코드 내부 참고


#### Version Control

최초 릴리즈 시점은 train.py의 v04 버전이 최초 릴리즈임.

|버전명|작성자|업데이트 내용|
|---|---|---|
|04|신칸센|최초 릴리즈, 기본적인 기능 충실|
| | |
