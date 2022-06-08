import cv2
import numpy as np
from matplotlib import image, pyplot as plt
org = cv2.imread('./Pictures/S1P1.jpeg')
img = cv2.cvtColor(org, cv2.COLOR_BGR2GRAY)
img = cv2.medianBlur(img,5)
# ret,th1 = cv2.threshold(img,127,255,cv2.THRESH_BINARY)
# th2 = cv2.adaptiveThreshold(img,255,cv2.ADAPTIVE_THRESH_MEAN_C,\
#             cv2.THRESH_BINARY,11,2)
gray = cv2.adaptiveThreshold(img,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY_INV,11,2)
org_gray = gray

# cv2.imshow("threshold", org_gray)
# cv2.waitKey(0)
# quit()

# cv2.imshow("threshold", gray)
kernel = np.ones((3,3), np.uint8)
kernel = cv2.circle(kernel,(1,1),2,0,1,-1)
img = cv2.erode(gray, kernel, iterations=1)
img = cv2.dilate(img, kernel, iterations=1)

proc = cv2.bitwise_not(img, img)
# cv2.imshow("proc", proc)
# cv2.imshow('Erosion', img)
# cv2.waitKey(0)

params = cv2.SimpleBlobDetector_Params()
# Set Area filtering parameters
params.filterByArea = True
params.minArea = 35
 
# Set Circularity filtering parameters
params.filterByCircularity = True
params.minCircularity = 0.1
 
# Set Convexity filtering parameters
params.filterByConvexity = False
params.minConvexity = 0.2
     
# Set inertia filtering parameters
params.filterByInertia = True
params.minInertiaRatio = 0.4
 
# Create a detector with the parameters
detector = cv2.SimpleBlobDetector_create(params)
     
# Detect blobs
keypoints = detector.detect(proc)

print(type(keypoints))

px_list = []
# coord = {}
for _item in keypoints:
    coord = {}
    coord["x"] =  _item.pt[0]
    coord["y"] =  _item.pt[1]
    coord["r"] =  _item.size
    px_list.append(coord)
    
print(px_list)

avr_radius = 0.0
for _item in px_list:
    avr_radius += _item["r"]

avr_radius/= len(px_list)
print("average radius", avr_radius)

    
print(len(px_list))
cv2.imshow("org_gray", org_gray)
cv2.imshow("org_gray", org_gray)
for _item in px_list:
    
    # if abs(_item["r"]-avr_radius)/avr_radius >0.3:
    cv2.circle(org,[int(_item["x"]), int(_item["y"])], int(_item["r"]/2),(0,0,255))
    cv2.circle(org_gray,[int(_item["x"]), int(_item["y"])], int(_item["r"]/2)+10,0,-1)
    
    # else:
    #     cv2.circle(org,[int(_item["x"]), int(_item["y"])], int(_item["r"]/2),(0,255,255))
    #     # px_list.remove(_item)
        # print(_item.size)
print(len(px_list))

# proc = cv2.bitwise_not(org_gray, org_gray)

# kernel = np.zeros((3,3), np.uint8)
# # kernel = cv2.circle(kernel,(1,1),2,0,1,-1)
# # # Show blobs
# # cv2.imshow("Filtering Circular Blobs Only", org_gray)
# img = cv2.erode(org_gray, kernel, iterations=1)
cv2.imshow("sdfsdfFiltering Circular Blobs Only", org)
cv2.imwrite("stone.png", org_gray)
cv2.waitKey(0)
# cv2.destroyAllWindows()

