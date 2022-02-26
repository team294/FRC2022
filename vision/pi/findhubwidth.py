from cscore import CameraServer, MjpegServer
from networktables import NetworkTablesInstance
import cv2
import numpy as np

width = 680 
height = 480

name = "shooter-cam"
yTolerance = 10 # in pixels
contourType = [('x', int), ('y', int), ('left', int), ('right', int), ('top', int), ('bottom', int)]

NetworkTablesInstance.getDefault().initialize(server='10.2.94.2')
sd = NetworkTablesInstance.getDefault().getTable(name)
sd.putNumber("LowerThresholdH", 70)
sd.putNumber("LowerThresholdS", 60)
sd.putNumber("LowerThresholdV", 200)
sd.putNumber("UpperThresholdH", 82)
sd.putNumber("UpperThresholdS", 255)
sd.putNumber("UpperThresholdV", 255)
sd.putNumber("rv", 1000)
sd.putNumber("rx", 1000)
sd.putNumber("ytol", yTolerance)

cs = CameraServer.getInstance()
cs.enableLogging()

camera = cs.startAutomaticCapture()
camera.setResolution(width, height)

sink = cs.getVideo()
output = cs.putVideo(name, width, height)
mjpeg = MjpegServer("cvhttpserver", "", 8083)
mjpeg.setSource(output)

input_img = np.array([[]])

def sortY(c):
    return c[1]

while True:
    time, input_img = sink.grabFrame(input_img)

    if time == 0: # There is an error
        output.notifyError(sink.getError())
        continue

    hsv = cv2.cvtColor(input_img, cv2.COLOR_BGR2HSV)

    
    lower_threshold = np.array([sd.getNumber("LowerThresholdH", 0), sd.getNumber("LowerThresholdS", 0), sd.getNumber("LowerThresholdV", 0)])
    upper_threshold = np.array([sd.getNumber("UpperThresholdH", 255), sd.getNumber("UpperThresholdS", 255), sd.getNumber("UpperThresholdV", 255)])
    threshold = cv2.inRange(hsv, lower_threshold, upper_threshold)
    kernel = np.ones((3, 3), np.uint8)
    threshold = cv2.morphologyEx(threshold, cv2.MORPH_OPEN, kernel)    

    _, contours, _ = cv2.findContours(threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    filtered = []
    
    for c in contours:
        rect = cv2.boundingRect(c)
        x,y,w,h = rect
        M = cv2.moments(c)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            # print([cX, cY, x, x+w, y, y+h])
            filtered.append([cX, cY, x, x+w, y, y+h])

    rv = 0
    rw = 0
    if len(filtered) > 0:
        # filtered = np.array(filtered, dtype=contourType)
        # filtered = np.sort(filtered, order='y')
        
        filtered.sort(key=sortY)

        rv = 1
        # median = filtered[int(filtered.size/2)]['y'] if filtered.size % 2 == 1 else filtered[filtered.size/2]['y']+filtered[filtered.size/2-1]['y']/2
        median = filtered[int(len(filtered)/2)][1]
        # filtered = np.array(list(filter(lambda c: abs(median-c['y']) < sd.getNumber("ytol", yTolerance), filtered)), dtype=contourType)
        # filtered[abs(median-filtered['y']) < sd.getNumber("ytol", yTolerance)]
        filtered = list(filter(lambda f: abs(median-f[1]) < sd.getNumber("ytol", yTolerance), filtered))

        if len(filtered) > 0:
            fx, fy, bx, by = filtered[0][2], filtered[0][4], filtered[0][3], filtered[0][5]
            for f in filtered:
                if f[2] < fx: fx = f[2]
                if f[4] < fy: fy = f[4]
                if f[3] > bx: bx = f[3]
                if f[5] > by: by = f[5]
            rw = bx - fx
            # print(f"({fx}, {fy}),({bx}, {by}")
            cv2.rectangle(input_img,(fx, fy),(bx, by),(0,255,0),2)



    # TODO add x and y position?
    sd.putNumber("rv", rv)
    sd.putNumber("rw", rw)
    

    output.putFrame(input_img)