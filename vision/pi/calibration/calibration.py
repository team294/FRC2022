from cscore import CameraServer, MjpegServer
from networktables import NetworkTablesInstance
import cv2
import numpy as np

# Allow smart dashboard input to change threshold

# basic setting variables
width = 680 
height = 480

name = "shooter-cam"
yTolerance = 60 # in pixels
lt = (56, 129, 65)
ut = (74, 255, 255)
contourType = [('x', int), ('y', int), ('left', int), ('right', int), ('top', int), ('bottom', int)]

settings = [ { "name": "connect_verbose", "value": 1 }, { "name": "contrast", "value": 50 }, { "name": "saturation", "value": 60 }, { "name": "power_line_frequency", "value": 2 }, { "name": "sharpness", "value": 50 }, { "name": "backlight_compensation", "value": 0 }, { "name": "pan_absolute", "value": 0 }, { "name": "tilt_absolute", "value": 0 }, { "name": "zoom_absolute", "value": 0 } ]

# initialize network tables
NetworkTablesInstance.getDefault().initialize(server='10.2.94.2')
sd = NetworkTablesInstance.getDefault().getTable(name)
sd.putNumber("LowerThresholdH", 56)
sd.putNumber("LowerThresholdS", 129)
sd.putNumber("LowerThresholdV", 65)
sd.putNumber("UpperThresholdH", 74)
sd.putNumber("UpperThresholdS", 255)
sd.putNumber("UpperThresholdV", 255)
sd.putNumber("rv", 1000)
sd.putNumber("rx", 1000)
sd.putNumber("ytol", yTolerance)

# initialize camera server
cs = CameraServer.getInstance()
cs.enableLogging()

camera = cs.startAutomaticCapture()
camera.setResolution(width, height)

# initialize input and output instances
sink = cs.getVideo()
output = cs.putVideo(name, width, height)
mjpeg = MjpegServer("cvhttpserver", "", 8083)
mjpeg.setSource(output)

input_img = np.array([[]])

# vision loop
while True:
    # get frame
    time, input_img = sink.grabFrame(input_img)

    if time == 0: # There is an error
        output.notifyError(sink.getError())
        continue

    # convert image to hsv
    hsv = cv2.cvtColor(input_img, cv2.COLOR_BGR2HSV)
    
    # get threshold
    # # TODO may return erroneous values (may be why sometimes not detect)
    lower_threshold = np.array([sd.getNumber("LowerThresholdH", lt[0]), sd.getNumber("LowerThresholdS", lt[1]), sd.getNumber("LowerThresholdV", lt[2])])
    upper_threshold = np.array([sd.getNumber("UpperThresholdH", ut[0]), sd.getNumber("UpperThresholdS", ut[1]), sd.getNumber("UpperThresholdV", ut[2])])
    threshold = cv2.inRange(hsv, lower_threshold, upper_threshold)

    # erode and then dilate by 3 x 3 kernel of 1s
    kernel = np.ones((3, 3), np.uint8)
    threshold = cv2.morphologyEx(threshold, cv2.MORPH_OPEN, kernel)    
    calibration = cv2.cvtColor(threshold, cv2.COLOR_GRAY2RGB)

    # get contours
    _, contours, _ = cv2.findContours(threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # init filtered array
    filtered = []
    
    # fill filtered array with values from all contours with an area greater than 15
    # respective values are the contour's center x-value, center y-value, 
    # bounding-rectangle lower-left x-val bounding-rect lower-left y-val, 
    # bounding rect upper-right x, bounding-rect upper-right y-val, and area
    for c in contours:
        area = cv2.contourArea(c)
        if (area < 15):
            continue
        rect = cv2.boundingRect(c)
        x,y,w,h = rect
        M = cv2.moments(c)
        # gets center x and y
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            filtered.append((cX, cY, x, x+w, y, y+h, area))

    rv = 0
    rw = 0
    rx = 0
    # if there are any values in filtered
    if len(filtered) > 0:
        # finds median value and filters for all values within the y tolerance on smart dashboard
        filtered.sort(key=lambda c: c[1]) # sorts array by y-value
        median = filtered[int(len(filtered)/2)][1] # gets median y-value
        # filtered = list(filter(lambda f: abs(median-f[1]) < yT, filtered)) # filters
        filtered = list(filter(lambda f: abs(median-f[1]) < yT, yTolerance)) # filters

        # if there are any values left in filtered
        if len(filtered) > 0:
            # sorts filtered array by contour area and caps it to at-most 4 elements
            filtered = sorted(filtered, key=lambda f: f[6])[-4:]
            rv = len(filtered) # gets the amount of contours found

            # gets lower-left-most x- and y-value and upper-right-most x- and y-value for final bounding box
            fx, fy, bx, by = filtered[0][2], filtered[0][4], filtered[0][3], filtered[0][5]
            for f in filtered:
                if f[2] < fx: fx = f[2]
                if f[4] < fy: fy = f[4]
                if f[3] > bx: bx = f[3]
                if f[5] > by: by = f[5]
            rw = bx - fx
            rx = -0.0937486*(0.5*(bx+fx)-0.5*width) - 4.99446 # x in pixels converted to angle in degrees!
            # draws bounding rectangle
            cv2.rectangle(calibration,(fx, fy),(bx, by),(0,255,0),2)


    # posts number of elements found, width of bounding box, and x-value in angle degrees
    sd.putNumber("rv", rv)
    sd.putNumber("rw", rw)
    sd.putNumber("rx", rx)

    # posts final image to stream
    # output.putFrame(cv2.resize(input_img, ))
    output.putFrame(calibration)