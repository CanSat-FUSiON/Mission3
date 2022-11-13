#以下自作ライブラリ
import MPU2
#別からもらったもの
import math
import geopy
from geopy.distance import geodesic
import RPi.GPIO as GPIO
from time import sleep
from gps3 import gps3
import csv
import datetime 

#gpsの取得とCSV保存
gps_socket = gps3.GPSDSocket()
data_stream = gps3.DataStream()
gps_socket.connect()
gps_socket.watch()
 
#9軸センサの加速度と地磁気取得
mpu9250 = MPU2.MPU9250()
Accel = mpu9250.readAccel()
magnet = mpu9250.readMagnet()
  
#ゴールの緯度経度
#goal_lat = ??
#goal_lon = ??

count = 0
#モーターの設定とタイヤの回し方
right_front=8
left_front=24
right_back=25
left_back=23

#タイヤを何秒間回すか
t1 = 1    #直進、後進に関する秒数
t2 = 0.1  #右or左向き、右左回転に関する秒数

GPIO.setmode(GPIO.BCM)  #GPIOの番号で指定する
GPIO.setup(right_front,GPIO.OUT) #right_frontのピンを出力端子として初期値0で使用する
GPIO.setup(left_front,GPIO.OUT)
GPIO.setup(right_back,GPIO.OUT)
GPIO.setup(left_back,GPIO.OUT)

GPIO.setwarnings(False) #GPIOピンの値がデフォルトでない時に出てくる文

def stop(x):
    GPIO.output(right_front,GPIO.LOW)
    GPIO.output(left_front,GPIO.LOW)
    GPIO.output(right_back,GPIO.LOW)
    GPIO.output(left_back,GPIO.LOW)
    sleep(1)
    return()

def forward(t1):
    GPIO.output(right_front,GPIO.HIGH)
    GPIO.output(left_front,GPIO.HIGH)
    GPIO.output(right_back,GPIO.LOW)
    GPIO.output(left_back,GPIO.LOW)
    sleep(1)
    return()

def back(t1):
    GPIO.output(right_back,GPIO.HIGH)
    GPIO.output(left_back,GPIO.HIGH)
    GPIO.output(right_front,GPIO.LOW)
    GPIO.output(left_front,GPIO.LOW)
    sleep(1)
    return()

def leftturn(t2):
    GPIO.output(right_front,GPIO.HIGH)
    GPIO.output(left_front,GPIO.LOW)
    GPIO.output(right_back,GPIO.LOW)
    GPIO.output(left_back,GPIO.HIGH)
    sleep(1)
    return()

def rightturn(t2):
    GPIO.output(right_front,GPIO.LOW)
    GPIO.output(left_front,GPIO.HIGH)
    GPIO.output(right_back,GPIO.HIGH)
    GPIO.output(left_back,GPIO.LOW)
    sleep(1)
    return()

def leftfacing(t2):#右タイヤ前進,左タイヤストップ
    GPIO.output(right_front,GPIO.HIGH)
    GPIO.output(left_front,GPIO.LOW)
    GPIO.output(right_back,GPIO.LOW)
    GPIO.output(left_back,GPIO.LOW)
    sleep(1)
    return()    

def rightfacing(t2): #右タイヤストップ,左タイヤ前進
    GPIO.output(right_front,GPIO.LOW)
    GPIO.output(left_front,GPIO.HIGH)
    GPIO.output(right_back,GPIO.LOW)
    GPIO.output(left_back,GPIO.LOW)
    sleep(1)
    return()



#スタック関数 4で割った余りでスタック処理を行う
def stack(x,y,z): #x=モーター動かす秒数, y=動きごとに何秒止まるか
    global count
    leftturn(1)
    stop(1)
    forward(t1)
    #本当にスタックしているのか
    if  -0.1<=ax<=0.1 and -0.1<=ay<=0.1:
        #スタック処理
        while -0.1<=ax<=0.1 and -0.1<=ay<=0.1:
            stop(1)
            if count%4 ==0:
               leftturn(y)
               print('leftturn')
               stop(1)
            elif count%4 == 1:
               rightturn(y)
               print('rightturn')
               stop(1)
            elif count%4 == 2:
               rightturn(y)
               print('rightturn*2')
               stop(1)
               rightturn(y)
               stop(1)
            elif count%4 == 3:
               leftturn(y)
               print('leftturn*2')
               stop(1)
               leftturn(y)
               stop(1)
            #向きを変えたら前進
            forward(1)  
    else:
         stop(1)
         print('escape stack')
         #forward(t1)
     
    count += 1
     
      
def rad_magnet(mx2, my2):
    rad = 0
    rad = math.atan2(mx2, my2)
    return rad

def Deg_cansat(mx2,my2):
    deg_cansat = rad_magnet(mx2,my2)*180/math.pi    
    if deg_cansat<0:
       return(deg_cansat+ 360)
    else:
       return(deg_cansat)
       
def Deg_goal(deg_goal):
   if deg_goal<0:
     return(deg_goal+ 360)
   else:
     return(deg_goal)

#以下値取得後の動き
#以下機体とゴールの向きを合わせる
while True:

    for new_data in gps_socket:
        if new_data:
            data_stream.unpack(new_data)    
            gps_dict = {"time" : data_stream.TPV['time'],
            "lat" : data_stream.TPV['lat'],
            "lon" : data_stream.TPV['lon'],
            "alt" : data_stream.TPV['alt'],
            "speed" : data_stream.TPV['speed']} 
            if gps_dict["time"] == "n/a":
               continue
            if gps_dict["time"] != "n/a":
   
               lat_i = gps_dict["lat"]
               lon_i = gps_dict["lon"]
               position_i = (lat_i, lon_i)
               position_cone = (goal_lat, goal_lon) 
               now_dis = geodesic(position_i, position_cone).m
               #print('ゴールまであと',now_dis)

               #ゴールと機体の方位角deg_goal
               #positioniからgoalまでの角度 -180<θ<180
               rad_goal = math.atan2((lon_i - goal_lon), (lat_i - goal_lat))
               #ラジアンからdegに変換
               deg_goal = rad_goal * 180 / math.pi
               break
               
   ax = Accel['x']
   ay = Accel['y']
   
   mx = magnet[0]
   my = magnet[1]
  
   mx2 = -mx
   my2 = my
   #print('x地磁気:',mx2,'y地磁気:',my2)

   #制御履歴
   with open('gps.csv', 'a') as f:
         writer = csv.writer(f, lineterminator='\n') # 改行コード（\n）を指定しておく
         list=[data_stream.TPV['time'],
         data_stream.TPV['lat'],
         data_stream.TPV['lon'],
         str(mx2),
         str(my2),
         str(now_dis),
              ]
         writer.writerow(list)     # list（1次元配列）の場合
       
   
   #1m以内になったらカメラ判定へ行く
   if 0.0 <= now_dis <= 1.0:
         stop(1)
         print('camera')
         break
   else:
         if -0.1<=ax<=0.1 and -0.1<=ay<=0.1:
             stop(1)
             #stack(moter,moter,sleep)
             stack(t1,t2,1)
             print('stack')
         else:
             if 0<= Deg_goal(deg_goal) <=180.0  and 165.0<= Deg_cansat(mx2,my2) - Deg_goal(deg_goal) <= 195.0:
                 forward(t1)
                 print('forward')
                 #stop(1)
                     
             elif 180< Deg_goal(deg_goal) <= 360.0 and 165.0 <= Deg_goal(deg_goal) - Deg_cansat(mx2,my2) <= 195.0:
                 forward(t1)
                 print('forward')
                 #stop(1)
       
             else:
                 #向きが違うなら一度止めて回転,ここは機体の回転具合次第でもっと考える必要あり
                 stop(1)
                 leftturn(t2) 
                 print('leftturn')
                 stop(1)         

GPIO.cleanup()
