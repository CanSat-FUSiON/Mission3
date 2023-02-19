# Edge Impulse - OpenMV Object Detection Example

import sensor, image, time, os, tf, math, uos, gc, pyb
from pyb import UART

uart = UART(3, 115200, timeout_char=10)                         # init with given baudrate

blue_led = pyb.LED(3)
green_led = pyb.LED(2)

sensor.reset()                         # Reset and initialize the sensor.
sensor.set_pixformat(sensor.RGB565)    # Set pixel format to RGB565 (or GRAYSCALE)
sensor.set_framesize(sensor.QVGA)      # Set frame size to QVGA (320x240)
sensor.set_windowing((240, 240))       # Set 240x240 window.
sensor.skip_frames(time=2000)          # Let the camera adjust.

net = None
labels = None
min_confidence = 0.5

try:
    # load the model, alloc the model file on the heap if we have at least 64K free after loading
    net = tf.load("trained.tflite", load_to_fb=uos.stat('trained.tflite')[6] > (gc.mem_free() - (64*1024)))
except Exception as e:
    raise Exception('Failed to load "trained.tflite", did you copy the .tflite and labels.txt file onto the mass-storage device? (' + str(e) + ')')

try:
    labels = [line.rstrip('\n') for line in open("labels.txt")]
except Exception as e:
    raise Exception('Failed to load "labels.txt", did you copy the .tflite and labels.txt file onto the mass-storage device? (' + str(e) + ')')

colors = [ # Add more colors if you are detecting more than 7 types of classes at once.
    (255,   0,   0),
    (  0, 255,   0),
    (255, 255,   0),
    (  0,   0, 255),
    (255,   0, 255),
    (  0, 255, 255),
    (255, 255, 255),
]

clock = time.clock()


def send(data):
    """ Send 'data' over UART. The first byte is 128 (10000000) which is a start of packet indicator.
    All other values are split into first and second 14 bit integer values.
    Max of 2^14 or 16,384.
    """

    sendData = [0x80]

    for num in data:
        num = round(num)
        sendData.append((num >> 7) & 0x7F)
        sendData.append(num & 0x7F)

    for num in sendData:
        try:
            uart.writechar(num)
        except:
            pass
    #print(sendData)

while(True):
    clock.tick()
    ave_center_x = 0
    ave_center_y = 0
    count = 0
    img = sensor.snapshot()

    # detect() returns all objects found in the image (splitted out per class already)
    # we skip class index 0, as that is the background, and then draw circles of the center
    # of our objects

    for i, detection_list in enumerate(net.detect(img, thresholds=[(math.ceil(min_confidence * 255), 255)])):
        if (i == 0): continue # background class
        if (len(detection_list) == 0): continue # no detections for this class?

        #print("********** %s **********" % labels[i])
        for d in detection_list:
            count += 1
            [x, y, w, h] = d.rect()
            center_x = math.floor(x + (w / 2))
            center_y = math.floor(y + (h / 2))
            #print('x %d\ty %d' % (center_x, center_y))
            #img.draw_circle((center_x, center_y, 12), color=colors[i], thickness=1)
            ave_center_x += center_x
            ave_center_y += center_y
    if count != 0:
        ave_center_x = ave_center_x/count
        ave_center_y = ave_center_y/count
        #print('ave_x %d\tave_y %d' % (ave_center_x, ave_center_y))
        #img.draw_circle((int(ave_center_x), int(ave_center_y), 12), color=colors[i], thickness=3)
        send([ave_center_x, ave_center_y]) # topleft = (0, 0); bottomright = (240, 240)
        #blue_led.off()
        #green_led.on()
    #else:
        #blue_led.on()
        #green_led.off()
    #print(clock.fps(), "fps", end="\n\n")
