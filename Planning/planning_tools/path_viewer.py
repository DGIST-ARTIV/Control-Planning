import csv
import utm
import argparse
import math
import matplotlib.pyplot as plt

from pyproj import Proj, transform
import numpy as np
import pandas  as pd
# Projection 정의
# UTM-K
proj_UTMK = Proj(init='epsg:5178') # UTM-K(Bassel) 도로명주소 지도 사용 중

# WGS1984
proj_WGS84 = Proj(init='epsg:4326') # Wgs84 경도/위도, GPS사용 전지구 좌표
class modified_path:
    def __init__(self, input_path= '', output_path= ''):
        self.x_path = []
        self.y_path = []
        self.latlon=[]
        self.utm_files = ["/home/vision/Desktop/path/parking1.csv"]#,"/home/vision/Desktop/path/parking2-1.csv","/home/vision/Desktop/path/parking3.csv","/home/vision/Desktop/path/parking4-2.csv","/home/vision/Desktop/path/parking5.csv","/home/vision/Desktop/path/parking6-1.csv"]
        self.gps_files= ["/home/vision/k_parking_start1_p.csv","/home/vision/k_parking_start2_p.csv","/home/vision/k_parking_start3_p.csv"]
        #,"/home/vision/k_parking_start2_p.csv","/home/vision/k_parking_start3_p.csv"
        self.off = 3
        self.x_point = []
        self.y_point = []
        self.buff = []
        for file in self.utm_files:
            self.get_utm_path(file,self.x_path,self.y_path)

        for file in self.gps_files:
            self.get_gps_path(file, self.x_point, self.y_point)
        print("path size :", len(self.x_point))
        '''
        for file in self.gps_files:
            self.get_gps_path1(file, self.x_point, self.y_point)
        print("path size :", len(self.x_point))
        '''
        self.fig, self.ax = plt.subplots(figsize=(15, 8))
        plt.grid(True)
        plt.xlabel('x')
        plt.ylabel('y')
        plt.title('Interactive Plot')

        self.ax.set(xlim=[min(self.x_path+self.x_point)-self.off, max(self.x_path+self.x_point)+self.off], ylim=[min(self.y_path+self.y_point)-self.off, max(self.y_path+self.y_point)+self.off])
        self.ax.set_aspect(1, adjustable='box')

        self.line, = self.ax.plot(self.x_path, self.y_path, "bo")
        self.cid = self.fig.canvas.mpl_connect('button_press_event', self.add_point)
        plt.show()
    def add_point(self, event):
        #print('you pressed', event.key, event.xdata, event.ydata)
        if event.inaxes != self.ax:
            return

        x = event.xdata
        y = event.ydata

        if event.button == 1 :
            if event.key == 'shift':
                id1, id2 = self.get_point_id(x,y)
                if id1>id2:
                    self.x_path.insert(id1,x)
                    self.y_path.insert(id1,y)
                else:
                    self.x_path.insert(id2,x)
                    self.y_path.insert(id2,y)

            if event.key =='control':
                plt.plot(self.x_point, self.y_point, "go")

            self.line.set_data(self.x_path, self.y_path)
            plt.draw()


        if event.button == 3:
            if event.key == 'shift':
                id1, id2= self.get_point_id(x,y)
                self.buff.append([id1, self.x_path[id1], self.y_path[id1]])
                del self.x_path[id1]
                del self.y_path[id1]
            if event.key =='control':
                try:
                    self.x_path.insert(self.buff[-1][0], self.buff[-1][1])
                    self.y_path.insert(self.buff[-1][0], self.buff[-1][2])
                    self.buff.pop()
                except:
                    print("Don't Ctrl z")
            self.line.set_data(self.x_path, self.y_path)
            plt.draw()


        if event.button == 2:
            self.save_point()
            plt.disconnect(self.cid)
            plt.close()

    def get_utm_path(self,file_name,x_path,y_path):

        print("start read path :",file_name)
        p=[0,0]
        with open(file_name, "r") as f:
            reader = csv.reader(f)
            for raw in reader:
                x_path.append(float(raw[0]))
                y_path.append(float(raw[1]))

                print(raw[0], raw[1],395201.3103811303, 5673135.241182375 )
                #p=utm.to_latlon(raw[0], raw[1])
                self.latlon.append([p[0],p[1]])
                #print(p)
        print ("path size :", len(self.x_path))
    def get_gps_path1(self,file_name, x_path,y_path):
        cnt = 0
        print("start read path :",file_name)
        proj_UTMK = Proj(init='epsg:5178') # UTM-K(Bassel) 도로명주소 지도 사용 중

        # WGS1984
        proj_WGS84 = Proj(init='epsg:4326') # Wgs84 경도/위도, GPS사용 전지구 좌표
        with open(file_name, "r") as f:
            reader = csv.reader(f)
            x=0
            y=0
            for raw in reader:

                if cnt==0:
                    cnt = 1
                    continue
                x,y=transform(proj_WGS84, proj_UTMK, float(raw[7]), float(raw[6]))
                x_path.append(x)
                y_path.append(y)
    def get_gps_path(self,file_name, x_path,y_path):
        cnt = 0
        print("start read path :",file_name)
        with open(file_name, "r") as f:
            reader = csv.reader(f)
            p = [0,0]
            for raw in reader:
                if cnt==0:
                    cnt = 1
                    continue
                p = utm.from_latlon(float(raw[6]), float(raw[7]))
                x_path.append(p[0])
                y_path.append(p[1])

    def get_point_id(self, x, y):
        first_id = -1
        second_id = -1
        mind1 = 100000000000000
        mind2 = 100000000000000
        for i in range(len(self.x_path)):
            d = math.sqrt((self.x_path[i]-x)*(self.x_path[i]-x) + (self.y_path[i]-y)*(self.y_path[i]-y))
            if(d<mind1):
                second_id = first_id
                mind2 = mind1

                first_id = i
                mind1 = d
                continue
            if(d<mind2):
                second_id = i
                mind2 = d
        return first_id, second_id
    def save_point(self):
        print("saving points...")
        for file in self.utm_files:
            with open(r_strip(file,'.csv')+"modified_path.csv",'w', encoding='utf-8') as f:
                wr = csv.writer(f)
                for i in range(len(self.x_path)):
                    wr.writerow([self.x_path[0], self.y_path[1]])
                print("Done with {}points".format(len(self.x_path)))



if __name__ == "__main__":
    a = modified_path()
