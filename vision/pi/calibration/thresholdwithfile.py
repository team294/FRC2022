import cv2
import numpy as np
import math

# b = cv2.imread('C:\\Users\\griff\\Documents\\Programming\\Python\\OpenCV\\framechange\\b.png')
# c = cv2.imread('C:\\Users\\griff\\Documents\\Programming\\Python\\OpenCV\\framechange\\c.png')
b = cv2.imread('C:\\Users\\griff\\Documents\\Programming\\Python\\OpenCV\\assets\\WIN_20220223_21_18_29_Pro.jpg')
c = cv2.imread('C:\\Users\\griff\\Documents\\Programming\\Python\\OpenCV\\assets\\WIN_20220223_21_18_31_Pro.jpg')
# b = cv2.imread('C:\\Users\\griff\\Documents\\Programming\\Python\\OpenCV\\framechange\\WIN_20220223_21_13_55_Pro.jpg')
# c = cv2.imread('C:\\Users\\griff\\Documents\\Programming\\Python\\OpenCV\\framechange\\WIN_20220223_21_13_54_Pro.jpg')

def removeOutliers(arr, axis):
    mean = np.mean(arr)
    standard_deviation = np.std(arr)
    distance_from_mean = abs(arr - mean)
    max_deviations = 4
    not_outlier = distance_from_mean < max_deviations * standard_deviation

diff = np.array([[]])
diff = cv2.absdiff(b, c)

k = (5,5)
kernel = np.ones((7,7), np.uint8)
# diff = cv2.erode(diff, kernel, cv2.BORDER_CONSTANT)
# diff = cv2.GaussianBlur(diff, k, 0)
# diff = cv2.medianBlur(diff, k[0])
diff = cv2.bilateralFilter(diff, 11, 41, 21)
# diff = cv2.blur(diff, k)

diff = cv2.erode(diff, kernel, cv2.BORDER_CONSTANT)

mask = cv2.inRange(diff, np.array([8, 8, 8]), np.array([255, 255, 255]))

img_mask = cv2.cvtColor(c, cv2.COLOR_BGR2HSV)[np.where(mask == 255)]

img_avg = np.mean(img_mask, axis=0)
img_min = np.min(img_mask, axis=0)
img_max = np.max(img_mask, axis=0)
print(img_avg)
print(img_min)
print(img_max)

mean = cv2.mean(cv2.cvtColor(b, cv2.COLOR_BGR2HSV), mask=mask)
print(mean)

cv2.imshow('Difference', cv2.bitwise_and(c,c, mask= mask))

k = cv2.waitKey(0)
if k == 27:         # wait for ESC key to exit
    cv2.destroyAllWindows()
elif k == ord('s'): # wait for 's' key to save and exit
    with open("vision.cfg", "w+") as file:
        file.write(",".join(img_min))
        file.write("\n")
        file.write(",".join(img_max))
    np.savetxt('test.out', img_mask, delimiter=',', fmt='%1.3f')