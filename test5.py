import cv2
import numpy as np
from matplotlib import image, pyplot as plt
src = cv2.imread('./Pictures/S1P2.jpeg',0)
img = cv2.medianBlur(src,5)
# ret,th1 = cv2.threshold(img,127,255,cv2.THRESH_BINARY)
# th2 = cv2.adaptiveThreshold(img,255,cv2.ADAPTIVE_THRESH_MEAN_C,\
#             cv2.THRESH_BINARY,11,2)
gray = cv2.adaptiveThreshold(img,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY_INV,5,2)

cv2.imshow("threshold", gray)
# Apply Hough transform on the blurred image.
rows = gray.shape[0]
circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 2,
                            param1=5, param2=5,
                            minRadius=2, maxRadius=8)


if circles is not None:
    circles = np.uint16(np.around(circles))
    for i in circles[0, :]:
        center = (i[0], i[1])
        # circle center
        cv2.circle(img, center, 1, (0, 100, 100), 1)
        # circle outline
        radius = i[2]
        cv2.circle(img, center, radius, (255, 0, 255), 1)


cv2.imshow("detected circles", img)
cv2.waitKey(0)