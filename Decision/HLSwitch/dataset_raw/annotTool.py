# Dataset Gen

import cv2
import os


videoName = str(input("Video Name : (with Extension *.mp4)  "))
try:
    print('./video/' + videoName)
    cap = cv2.VideoCapture('./video/' + videoName)
except:
    print("Error while reading video file")# Dataset Gen

idx = 0
while(1):
    ret, frame = cap.read()
    print("Arrow Key LEFT : GPS\nArrow Key Right VISION\nArrow Key Up Save all and Exit")
    if ret:

        cv2.imshow('SCREEN', frame)
        ket = cv2.waitKey(0)
        print(ket)
        if ket == 81 :
            # Left
            dir = './GPS/'
            print("GPS")

            cv2.imwrite(dir + videoName + '_' + str(idx) + '.jpg', cv2.resize(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), (640, 360)))
            pass
        elif ket == 83:
            # Right
            dir = './VISION/'
            cv2.imwrite(dir + videoName + '_' + str(idx) + '.jpg', cv2.resize(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), (640, 360)))
            print("VISION")
            pass
        elif ket == 82:
            print("SAVE")
            break

    else:
        break
    idx += 1

cap.release()
cv2.destroyAllWindows()


while(1):
    ret, frame = cap.read()
    print("Arrow Key LEFT : GPS\nArrow Key Right VISION\nArrow Key Up Save all and Exit")
    if ret:
        cv2.imshow('SCREEN', frame)
        ket = cv2.waiyKey(0)

        if ket == 2424832 :
            # Left
            dir = './GPS/'
            print("GPS")
            pass
        elif ket == 2555904:
            # Right
            dir = './VISION'
            print("VISION")
            pass
        elif ket == 2490368:
            print("SAVE")
            break

    else:
        break

cap.release()
cv2.destroyAllWindows()
