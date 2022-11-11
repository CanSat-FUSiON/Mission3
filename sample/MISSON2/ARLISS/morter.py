from operator import le
import RPi.GPIO as GPIO
import time

left_front=23
right_front=25
left_back=24
right_back=8

GPIO.setwarnings(False)

GPIO.setmode(GPIO.BCM)
GPIO.setup(right_front,GPIO.OUT)
GPIO.setup(left_front,GPIO.OUT)
GPIO.setup(right_back,GPIO.OUT)
GPIO.setup(left_back,GPIO.OUT)

def stop(x=0.1):
    GPIO.output(right_front,GPIO.LOW)
    GPIO.output(left_front,GPIO.LOW)
    GPIO.output(right_back,GPIO.LOW)
    GPIO.output(right_back,GPIO.LOW)
    time.sleep(x)
    return()

def forward():
    GPIO.output(right_front,GPIO.HIGH)
    GPIO.output(left_front,GPIO.HIGH)
    GPIO.output(right_back,GPIO.LOW)
    GPIO.output(left_back,GPIO.LOW)
    return()

def back():
    GPIO.output(right_back,GPIO.HIGH)
    GPIO.output(left_back,GPIO.HIGH)
    GPIO.output(right_front,GPIO.LOW)
    GPIO.output(left_front,GPIO.LOW)
    return()

def leftturn(x=0.3):
    GPIO.output(right_front,GPIO.HIGH)
    GPIO.output(left_front,GPIO.LOW)
    GPIO.output(right_back,GPIO.LOW)
    GPIO.output(left_back,GPIO.HIGH)
    time.sleep(x) #路面状況などによって時間調整
    stop()
    return()

def rightturn(x=0.3):
    GPIO.output(right_front,GPIO.LOW)
    GPIO.output(left_front,GPIO.HIGH)
    GPIO.output(right_back,GPIO.HIGH)
    GPIO.output(left_back,GPIO.LOW)
    time.sleep(x) #路面状況などによって時間調整
    stop()
    return()

def reforward(x):#back and forward
    GPIO.output(right_back,GPIO.HIGH)
    GPIO.output(left_back,GPIO.HIGH)
    GPIO.output(right_front,GPIO.LOW)
    GPIO.output(left_front,GPIO.LOW)
    time.sleep(x)
    GPIO.output(right_front,GPIO.HIGH)
    GPIO.output(left_front,GPIO.HIGH)
    GPIO.output(right_back,GPIO.LOW)
    GPIO.output(left_back,GPIO.LOW)
    time.sleep(5)

def wave(x):#continuous back and forward
    for i in range(x):
        GPIO.output(right_back,GPIO.HIGH)
        GPIO.output(left_back,GPIO.HIGH)
        GPIO.output(right_front,GPIO.LOW)
        GPIO.output(left_front,GPIO.LOW)
        time.sleep(1)
        GPIO.output(right_front,GPIO.HIGH)
        GPIO.output(left_front,GPIO.HIGH)
        GPIO.output(right_back,GPIO.LOW)
        GPIO.output(left_back,GPIO.LOW)
        time.sleep(1)

def alltry(x):
    GPIO.output(right_front, GPIO.HIGH)
    GPIO.output(left_front, GPIO.LOW)
    GPIO.output(right_back, GPIO.LOW)
    GPIO.output(left_back, GPIO.LOW)
    time.sleep(1)
    GPIO.output(right_back, GPIO.HIGH)
    GPIO.output(left_back, GPIO.LOW)
    GPIO.output(right_front, GPIO.LOW)
    GPIO.output(left_front, GPIO.LOW)
    time.sleep(1)
    GPIO.output(right_front, GPIO.LOW)
    GPIO.output(left_front, GPIO.HIGH)
    GPIO.output(right_back, GPIO.LOW)
    GPIO.output(left_back, GPIO.LOW)
    time.sleep(1)
    GPIO.output(right_back, GPIO.LOW)
    GPIO.output(left_back, GPIO.HIGH)
    GPIO.output(right_front, GPIO.LOW)
    GPIO.output(left_front, GPIO.LOW)
    time.sleep(1)
    GPIO.output(right_back, GPIO.HIGH)
    GPIO.output(left_back, GPIO.HIGH)
    GPIO.output(right_front, GPIO.LOW)
    GPIO.output(left_front, GPIO.LOW)
    time.sleep(1)
    GPIO.output(right_front, GPIO.HIGH)
    GPIO.output(left_front, GPIO.HIGH)
    GPIO.output(right_back, GPIO.LOW)
    GPIO.output(left_back, GPIO.LOW)
    time.sleep(1)

def rightfwave(x): #right_front and wave
    GPIO.output(right_front, GPIO.HIGH)
    GPIO.output(left_front, GPIO.LOW)
    GPIO.output(right_back, GPIO.LOW)
    GPIO.output(left_back, GPIO.LOW)
    time.sleep(1)
    GPIO.output(right_back, GPIO.HIGH)
    GPIO.output(left_back, GPIO.HIGH)
    GPIO.output(right_front, GPIO.LOW)
    GPIO.output(left_front, GPIO.LOW)
    time.sleep(1)
    GPIO.output(right_front, GPIO.HIGH)
    GPIO.output(left_front, GPIO.HIGH)
    GPIO.output(right_back, GPIO.LOW)
    GPIO.output(left_back, GPIO.LOW)
    time.sleep(1)

def leftfwave(x): #left_front and wave
    GPIO.output(right_front, GPIO.LOW)
    GPIO.output(left_front, GPIO.HIGH)
    GPIO.output(right_back, GPIO.LOW)
    GPIO.output(left_back, GPIO.LOW)
    time.sleep(1)
    GPIO.output(right_back, GPIO.HIGH)
    GPIO.output(left_back, GPIO.HIGH)
    GPIO.output(right_front, GPIO.LOW)
    GPIO.output(left_front, GPIO.LOW)
    time.sleep(1)
    GPIO.output(right_front, GPIO.HIGH)
    GPIO.output(left_front, GPIO.HIGH)
    GPIO.output(right_back, GPIO.LOW)
    GPIO.output(left_back, GPIO.LOW)
    time.sleep(1)

def rightbwave(x): #right_back and wave
    GPIO.output(right_back, GPIO.HIGH)
    GPIO.output(left_back, GPIO.LOW)
    GPIO.output(right_front, GPIO.LOW)
    GPIO.output(left_front, GPIO.LOW)
    time.sleep(1)
    GPIO.output(right_back, GPIO.HIGH)
    GPIO.output(left_back, GPIO.HIGH)
    GPIO.output(right_front, GPIO.LOW)
    GPIO.output(left_front, GPIO.LOW)
    time.sleep(1)
    GPIO.output(right_front, GPIO.HIGH)
    GPIO.output(left_front, GPIO.HIGH)
    GPIO.output(right_back, GPIO.LOW)
    GPIO.output(left_back, GPIO.LOW)
    time.sleep(1)

def leftbwave(x): #left_back and wave
    GPIO.output(right_back, GPIO.LOW)
    GPIO.output(left_back, GPIO.HIGH)
    GPIO.output(right_front, GPIO.LOW)
    GPIO.output(left_front, GPIO.LOW)
    time.sleep(1)
    GPIO.output(right_back, GPIO.HIGH)
    GPIO.output(left_back, GPIO.HIGH)
    GPIO.output(right_front, GPIO.LOW)
    GPIO.output(left_front, GPIO.LOW)
    time.sleep(1)
    GPIO.output(right_front, GPIO.HIGH)
    GPIO.output(left_front, GPIO.HIGH)
    GPIO.output(right_back, GPIO.LOW)
    GPIO.output(left_back, GPIO.LOW)
    time.sleep(1)
