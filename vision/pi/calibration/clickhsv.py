import cv2
import numpy as np

samples = []
mouseX = 0
mouseY = 0
# frame = cv2.imread('C:\\Users\\griff\\Documents\\Programming\\Python\\OpenCV\\framechange\\c.png')
frame = cv2.imread('C:\\Users\\griff\\Documents\\Programming\\Python\\OpenCV\\assets\\WIN_20220223_21_18_31_Pro.jpg')

def click_event(event,x,y,flags,param):
    global mouseX,mouseY
    if event == cv2.EVENT_LBUTTONDOWN:
        mouseX,mouseY = x,y
        samples.append((x,y))
        for c in samples:
            cv2.circle(frame,(c[0],c[1]),5,(255,0,0),-1)
        print(f"Min: {minVal([hsv[c[1], c[0]] for c in samples])}")
        print(f"Max: {maxVal([hsv[c[1], c[0]] for c in samples])}")

def minVal(arr):
    if len(arr) <= 0:
        return np.array([0,0,0])
    color=[arr[0][0], arr[0][1], arr[0][2]]
    for val in arr:
        if val[0] < color[0]:
            color[0] = val[0]
        if val[1] < color[1]:
            color[1] = val[1]
        if val[2] < color[2]:
            color[2] = val[2]
    return np.array(color)

def maxVal(arr):
    if len(arr) <= 0:
        return np.array([255, 255, 255])
    color=[arr[0][0], arr[0][1], arr[0][2]]
    for val in arr:
        if val[0] > color[0]:
            color[0] = val[0]
        if val[1] > color[1]:
            color[1] = val[1]
        if val[2] > color[2]:
            color[2] = val[2]
    return np.array(color)

hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

lower_threshold = np.array([ 42, 170,  59])
upper_threshold = np.array([ 80, 255, 255])

mask = cv2.inRange(hsv, lower_threshold, upper_threshold)

# Bitwise-AND mask and original image
res = cv2.bitwise_and(frame,frame, mask= mask)

cv2.imshow('frame',frame)
cv2.imshow('res',mask)
cv2.setMouseCallback('frame',click_event)

key = cv2.waitKey(0)
if key == 27:
    cv2.destroyAllWindows()
elif key == ord('q') and len(samples) > 0:
    del samples[-1]
elif key == ord('s'):
    with open("vision.cfg", "w") as file:
        file.write(str(lower_threshold))
        file.write("\n")
        file.write(str(upper_threshold))
elif key == ord('p'):
    print(samples)

