# 이미지 정규화 하는 코드
# 그레이스케일, 리사이징, 정규화 (/255.0) 하고 npy에 stack
# 사진들이 분류 되어있는 폴더의 상위 디렉토리에 넣고 돌리기...

# script for preparing datasets, loading fer data and generating scaled images
import numpy as np
import cv2
from PIL import Image
import sys
import os

emotions = {0:'GPS', 1:'VISION'}
global outfile, data, data0, label, label0, desired_size

def _cls():
    os.system("cls" if os.name == "nt" else "clear")

def run(data, label, desired_size):
    data0 = data[:]
    label0 = label[:]
    for root, dirs, files in os.walk(os.getcwd(), topdown=False):

        for name in files:
            try:

                filename = os.path.join(root, name)
                print(filename)
                cvim = cv2.imread(filename, cv2.IMREAD_GRAYSCALE)
                grayCv = cv2.resize(cvim, desired_size)
                print(grayCv.shape)
                pdata = np.array(grayCv.reshape(-1)).astype('float32')
                pdata = [np.divide(d, 255.0) for d in pdata]
                try:
                    data = np.concatenate((data, [pdata]))
                except Exception as ex:
                    print('first start', ex)
                    data = np.array([pdata])

                emotios_k = list(emotions.keys())
                for key in emotios_k:
                    if emotions[key] in filename:
                        label = np.concatenate((label, [key]))
                        _cls()
                        print(f'add img2pix {name}:{emotions[key]}')

            except Exception as rex:
                print('Image OpenError!', rex)
    print(data.shape)
    print(f"All done! \n{len(data)} files are coverted and added ")
    name = "Scaled_" + str(len(data)) + ".npy"
    name_lab = "labeld_" + str(len(data)) + ".npy"
    np.save(name, data)
    np.save(name_lab, label)



# load previous npy file if exist
try:
    global outfile, data, data0, label, label0, desired_size
    var1, var2, desired_size = '', '', (320, 180)
    var1 = sys.argv[2] #pixel
    var2 = sys.argv[3] #label
    data = [[]]
    dataO = data[:]
    label = []
    labelO = label[:]
    desired_size = eval(sys.argv[1])

    run(data, label, desired_size)
except Exception as ex:
    tuto = '''
    image2pixel by shinkansan
    =Manual=
    e.g) python img2pixel.py '(48, 48)' data.npy label.npy
    argv[1] = img resize..tuple
    argv[2] = merging existing data npy file
    argv[3] = merging existing label npy file
    '''
    print(tuto)
    print(ex)
