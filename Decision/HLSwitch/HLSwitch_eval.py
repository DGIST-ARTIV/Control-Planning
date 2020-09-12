# HLSwitch evaluation py
import tensorflow as tf
import numpy as np
import time
import cv2
from keras.models import load_model
import time
from keras.preprocessing.image import load_img, save_img, img_to_array
import multiprocessing


config = tf.ConfigProto()
config.gpu_options.allow_growth = True
session = tf.Session(config=config)

model_dir = './models/final_model_new_4.hdf5'

class imageProcess():
    def preprocess(self, img):
        #img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img = img.astype(np.float32)
        img /= 255.0
        return img

    def postprocess(self, img):
        pass


class inference():
    def __init__(self):
        self.model = load_model(model_dir)
        self.switchValue = 0.5
        self.imageSize = (200,200)
        self.improc = imageProcess()
        self.showInferenceTime = True
        self.ans_dict = {0:'GPS', 1:'VISION'}


    def run(self, img):
        start = time.time()
        tempStr = ""
        tempImage = self.improc.preprocess(img)
        result = self.model.predict(tempImage.reshape(1, self.imageSize[0], self.imageSize[1], 1))
        prediction_result_prob = result * 100
        end = time.time()
        if self.showInferenceTime:
            print("Inference Time(s) :", end-start)
        if result[0] > self.switchValue:
            tempStr = "VISION"
        else:
            tempStr = "GPS"

        return [tempStr, prediction_result_prob]



class subNode():
    def __init__(self):
        #rospy.Subscriber('/camera/image_color', Image, self.imageCb)
        runTh = multiprocessing.Process(target=self.run)
        runTh.start()


    def run(self):
        pass

    def imageCb(self, msg):
        self.trafficIdx = msg.data
        print(self.trafficIdx)
        pass

class pubNode():
    def __init__(self):
        #self.switchPub = rospy.Publisher(rootname+pubAccel, Int16, queue_size = 1)
        pass
    def run(self):
        pass

class main():
    def run(self):
        hlSwitch = inference()
        video_dir = 'test.webm'
        cap = cv2.VideoCapture(video_dir)
        while(1):
            ret, frame = cap.read()
            if not ret:
                continue
            tempImage = cv2.resize(frame, (200, 200))
            tempImage = cv2.cvtColor(tempImage, cv2.COLOR_BGR2GRAY)
            location = (0, 40)
            font = cv2.FONT_HERSHEY_SIMPLEX  # hand-writing style font
            fontScale = 0.9
            strn = str(hlSwitch.run(tempImage))
            cv2.putText(frame, strn, location, font, fontScale, (0,100,255), 2)
            cv2.imshow('asd', frame)


            cv2.waitKey(1)


if __name__ == "__main__":
    run = main()
    run.run()
