import os
import cv2
import numpy as np
from matplotlib import image, pyplot as plt

def find_ratio(img_path):
    org = cv2.imread(img_path)
    img = cv2.cvtColor(org, cv2.COLOR_BGR2GRAY)
    img = cv2.medianBlur(img,5)

    gray = cv2.adaptiveThreshold(img,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY_INV,11,2)
    org_gray = gray

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
        


    avr_radius = 0.0
    for _item in px_list:
        avr_radius += _item["r"]

    # avr_radius/= len(px_list)
    # print("average radius", avr_radius)
    cnt1 = len(px_list) 

    for _item in px_list:
        cv2.circle(org,[int(_item["x"]), int(_item["y"])], int(_item["r"]/2),(0,0,255))
        cv2.circle(org_gray,[int(_item["x"]), int(_item["y"])], int(_item["r"]/2)+10,0,-1)

    # cv2.imshow("sdfsdfFiltering Circular Blobs Only", org)

    ##################################################

    img = org_gray
    img = cv2.medianBlur(img,9)

    gray = cv2.adaptiveThreshold(img,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,11,2)
    org_gray = gray

    img = cv2.erode(gray, kernel, iterations=1)
    img = cv2.dilate(img, kernel, iterations=1)

    proc = cv2.bitwise_not(img, img)
    # Create a detector with the parameters
    detector = cv2.SimpleBlobDetector_create(params)
        
    # Detect blobs
    keypoints = detector.detect(proc)

    px_list = []
    # coord = {}
    for _item in keypoints:
        coord = {}
        coord["x"] =  _item.pt[0]
        coord["y"] =  _item.pt[1]
        coord["r"] =  _item.size
        px_list.append(coord)

    # print()
    cnt2 = len(px_list)
    for _item in px_list:
        cv2.circle(org,[int(_item["x"]), int(_item["y"])], int(_item["r"]/2),(255,0,0),2)

    
    _ratio = round(cnt2/(cnt1+cnt2)*100,2)
    rtext = f"Obstruction percentage : {_ratio}%"
    org = cv2.putText(org, rtext, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
    
    _tcnt = (cnt1+cnt2)
    rtext = f"Total count : {_tcnt}"
    org = cv2.putText(org, rtext, (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
    cv2.imshow(img_path + ": Result", org)
    return _tcnt, _ratio





for file in os.listdir("./Pictures"):
    if file.endswith(".jpeg"):
        _path = './Pictures/' + file
        # print(_path)
        find_ratio(_path)
        # print(os.path.join("./Pictures", file))
cv2.waitKey(0)
cv2.destroyAllWindows()