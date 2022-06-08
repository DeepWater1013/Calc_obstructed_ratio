import os
import cv2
import numpy as np

def myFunc(e):
  return e['x']

def calc_length2(pt1, pt2): # calculate the square of distance between 2 points.
    return (pt1['x']-pt2['x']) *(pt1['x']-pt2['x'])  +  (pt1['y']-pt2['y'])*(pt1['y']-pt2['y'])

def triangeArea(pt1, pt2, pt3): # calculate the area of triangle obstruction percentage
  return abs((0.5)*(pt1['x']*(pt2['y']-pt3['y'])+pt2['x']*(pt3['y']-pt1['y'])+pt3['x']*(pt1['y']-pt2['y'])))
  
def find_ratio(img_path): # calcluate the 
    org = cv2.imread(img_path)
    
    wid = org.shape[0]
    hei = org.shape[1]
    print("width height of image :  ", wid," x ",hei)
    
    img = cv2.cvtColor(org, cv2.COLOR_BGR2GRAY)
    img = cv2.medianBlur(img,5)
    
    gray = cv2.adaptiveThreshold(img,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY_INV,11,2)

    kernel = np.ones((3,3), np.uint8)
    kernel = cv2.circle(kernel,(1,1),2,0,1,-1)
    img = cv2.erode(gray, kernel, iterations=3)
    img = cv2.dilate(img, kernel, iterations=3)

    proc = cv2.bitwise_not(img, img)

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

    pt_list = [] # point list for open holes.
    for _item in keypoints:
        coord = {}
        coord["x"] =  int(_item.pt[0])
        coord["y"] =  int(_item.pt[1])
        coord["r"] =  int(_item.size/2)
        pt_list.append(coord)

    avr_radius = 0.0  # average radius of open holes
    for _item in pt_list:
        avr_radius += _item["r"]

    open_hole_cnt = len(pt_list) # count of open holes.
    avr_radius/= len(pt_list)
    print("average radius  "+str(avr_radius), " px")
    
    for _item in pt_list:
        #draw the red circles around open holes.
        cv2.circle(org,[_item["x"], _item["y"]], _item["r"],(0,0,255))

    # sort by x values.
    pt_list.sort(key=myFunc)
    
    # _max_len means the maximum distance between 2 open holes.
    _max_len = avr_radius*6
    
    # square of _max_len for calc_length2(). can avoid square root calculation.
    _max_len2 = _max_len * _max_len 
    
    
    triangular_cnt = 0
    triangular_avr_area = 0.0
    
    for i in range(open_hole_cnt-2):
        for j in range (i+1, open_hole_cnt-1):
            if pt_list[j]['x'] - pt_list[i]['x'] > _max_len:
                break
            
            if calc_length2(pt_list[i], pt_list[j])>_max_len2:
                continue
            
            for k in range (j+1, open_hole_cnt):
                if pt_list[k]['x'] - pt_list[j]['x'] > _max_len:
                    break
                if calc_length2(pt_list[i], pt_list[k])>_max_len2 or calc_length2(pt_list[j], pt_list[k])>_max_len2:
                    continue
                
                triangular_cnt +=1
                _area = triangeArea(pt_list[i],pt_list[j],pt_list[k])
                
                triangular_avr_area += _area
                # taglr_list.append([i,j,k, _area])
                cv2.line(org, [pt_list[i]['x'],pt_list[i]['y']], [pt_list[j]['x'],pt_list[j]['y']], (0,255,0), 1)
                cv2.line(org, [pt_list[i]['x'],pt_list[i]['y']], [pt_list[k]['x'],pt_list[k]['y']], (0,255,0), 1)
                cv2.line(org, [pt_list[k]['x'],pt_list[k]['y']], [pt_list[j]['x'],pt_list[j]['y']], (0,255,0), 1)
                # print(i,j,k)
                
    cv2.imshow(img_path + ": Result", org)
    
    print("red cnt ", open_hole_cnt)
    print("triangular cnt ", triangular_cnt)
    
    print(triangular_avr_area/(wid*hei))
    try:
        triangular_avr_area /= triangular_cnt
        print("average area of a triangular : ",triangular_avr_area)
        
        hexagon_area = triangular_avr_area*2
        ratio = round(100 - hexagon_area*open_hole_cnt/(wid*hei)*100,2)
        print("average are of a hexagone",hexagon_area)
        print(ratio)
    except:
        ratio = 100
    
    rtext = f"Obstruction percentage : {ratio}%"
    org = cv2.putText(org, rtext, (100, 200), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 0, 0), 4, cv2.LINE_AA)
    cv2.imshow(img_path + ": Result", org)
    
    return ratio





for file in os.listdir("./Pictures"):
    if file.endswith(".jpeg"):
        
        _path = './Pictures/' + file
        print(_path)
        find_ratio(_path)
        # break
        
cv2.waitKey(0)
cv2.destroyAllWindows()