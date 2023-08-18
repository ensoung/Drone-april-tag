#!/usr/bin/env python
# coding: utf-8




import cv2
from pupil_apriltags import Detector
from djitellopy import Tello
from time import sleep
import numpy as np
import sys




#isTello=True
isTello=False

# 0x01: u,d,l r     0x02: fwd, bwd
# 0x04: cw, ccw     0x07: all
moving=0x7

if isTello==False: 
    moving=0
    
qr_edge_size = 14000
qr_tolerance = 1400+1400 # qr_size/10

rc_params = [0,0,0,0] # l-r, b-f, d-u, yaw   -100~100
rc_default = 15

#isTakeoff = False





def TurningSign(frame, direction):  # direction : True-Left, False-right function draws the direction the drone will turn
    YEL = (0, 255, 255)
    radius = 36 #frame.shape[1]>>5
    axes = (radius, radius)  # The x,y radius must be the same if is a circle otherwise it is eclipse
    angle = 0  #results in a lot of spinning
    
    #center = (frame.shape[0]>>8 , frame.shape[1]-radius*2)
    arc_end_x, arc_end_y = frame.shape[1]>>1, frame.shape[0]-radius*4 
    #print(f'{arc_end_x}, {arc_end_y}, {arc_end_x}, {arc_end_y}')
    if direction: # left
        center = ( arc_end_x-radius , arc_end_y)
        startAngle = 0
        endAngle = 270
    else: # right
        center = ( arc_end_x+radius , arc_end_y)
        startAngle = -90
        endAngle = 180
    thickness = 4
    cv2.ellipse(frame, center, axes, angle, startAngle, endAngle, YEL, thickness)

    pts = np.array([[arc_end_x-(radius>>1),arc_end_y],[arc_end_x+(radius>>1),arc_end_y],[arc_end_x, arc_end_y-(radius>>1)]],
                   np.int32)
    cv2.fillPoly(frame, [pts], YEL)
    





def MovementSign(frame, moving, start_rect, end_rect, move_dir): #draws out the direction the drone will travel
    global rc_params, rc_default  # l-r, b-f, d-u, yaw   -100~100
    rc_params=[0,0,0,0]
    #rc_default=20
    YEL = (0, 255, 255)
    dist = 25
    mid_x=frame.shape[1]>>1
    mid_y=frame.shape[0]>>1
    if move_dir["up"]:
        pt1=[mid_x, end_rect[1]]
        pts=np.array([pt1, [pt1[0]-dist,pt1[1]+dist], [pt1[0]+dist,pt1[1]+dist]])
        cv2.fillPoly(frame, [pts], YEL)
        if moving&0x01:
            rc_params[2]=rc_default
        
    if move_dir["down"]:
        pt1=[mid_x, start_rect[1]]
        cv2.fillPoly(frame, [np.array([pt1, [pt1[0]-dist,pt1[1]-dist], [pt1[0]+dist,pt1[1]-dist]])], YEL)
        if moving&0x01:
            rc_params[2]=rc_default*(-1)
        
    if move_dir["left"]:
        pt1=[end_rect[0], mid_y]
        cv2.fillPoly(frame, [np.array([pt1, [pt1[0]+dist,pt1[1]-dist], [pt1[0]+dist, pt1[1]+dist]])], YEL)
        if moving&0x01:
            rc_params[0]=rc_default*(-1)
        
    if move_dir["right"]:
        pt1=[start_rect[0], mid_y]
        cv2.fillPoly(frame,  [np.array([pt1, [pt1[0]-dist,pt1[1]-dist], [pt1[0]-dist, pt1[1]+dist]])], YEL)
        if moving&0x01:
            rc_params[0]=rc_default
       
    if move_dir["forward"]:
        if moving&0x02:
            rc_params[1]=rc_default
        tri_len=60
        start_pt = [mid_x, tri_len]
        cv2.fillPoly(frame, [np.array([start_pt, [start_pt[0]-tri_len, start_pt[1]+tri_len], 
                                      [start_pt[0]+tri_len, start_pt[1]+tri_len]])], YEL)
                
    if move_dir["backward"]:
        if moving&0x02:
            rc_params[1]=rc_default*(-1)
        tri_len=60
        start_pt = [mid_x, tri_len+tri_len]
        cv2.fillPoly(frame, 
                     [np.array([start_pt, [start_pt[0]-tri_len, start_pt[1]-tri_len], 
                                      [start_pt[0]+tri_len, start_pt[1]-tri_len]])], YEL)
        
     
    if move_dir["cw"]:
        if moving&0x04:
            rc_params[3]=rc_default  # 회전 cw (rc_parmas[3]) 
        #    rc_params[0]=rc_default*(-1) # Move left
        TurningSign(frame, False)  # direction : Ture-Left, False-right
        
    if move_dir["ccw"]:  
        if moving&0x04:
            rc_params[3]=rc_default*(-1) # 회전 ccw
           # rc_params[0]=rc_default # Move Right
        TurningSign(frame, True)
        
    if isTello:
        str=f"rc {rc_params[0]} {rc_params[1]} {rc_params[2]} {rc_params[3]}"
        tello.send_command_without_return(str)





# will change apriltag corner position to top_left, top_right, bottom_left, bottom_right
def SwapPosition(list, pos1, pos2):
    list[pos1][0],list[pos1][1],list[pos2][0], list[pos2][1] = list[pos2][0], list[pos2][1],list[pos1][0], list[pos1][1]
    return list





def SortCorners(coners): #sorts the corners to the respective corner it is supposed to be at
    for i in range(4):  # put top_left to index0 and bottom_left to index3
        for j in range(i+1, 4):
            if (coners[i][0]+coners[i][1]) > (coners[j][0]+coners[j][1]):
                coners=SwapPosition(coners, i, j)
                #print(f"swap{i}, {j}")
                
    if coners[1][1] < coners[2][1]: #put top_right to index1 and bottom_left to index2,
        coners=SwapPosition(coners, 1, 2)





if isTello: #checks if the drone is active and creates the object for the rest of the code to reference
    tello = Tello() # if this is in the try it bugs out
    try:
        # Create Tello Object
        tello.connect()
        print(f"Battery Life Percentage: {tello.get_battery()}")
        #tello.reboot()
        sleep(1)

        #if tello.send_command_with_return("takeoff", 7) != 'ok':
        if moving!=0:
            tello.takeoff()
            print('takeoff')
            
        # Start the video Stream
        tello.streamon()
        print('tello init')
        
    except cv2.error as e:
        print('Tello Init Error!! : [{e}]')
        tello.streamoff()
        tello.end()
        exit(1)







if isTello==False: #if drone is not found use back camera
    try:
        #cap = cv2.VideoCapture(0 , cv2.CAP_DSHOW) # Use This for Windows + Webcam
        cap = cv2.VideoCapture(1) # Use This for ubuntu + Webcam
        #cap = cv2.VideoCapture(0) # Use This for Raspberry Pi+WebCam 
        #cap = cv2.VideoCapture(f'rtsp://admin:admin@192.168.0.62:1935')  # IP Camera

        #cv2.namedWindow('Capturing', cv2.WINDOW_NORMAL)
        cap.set(cv2.CAP_PROP_FRAME_COUNT,5) # set 5 frame per Second

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT,720)
    except cv2.error as e:
        print('CV Start Error!!')
        exit(1)





at_detector = Detector(
   families="tag36h11",
   nthreads=1,
   quad_decimate=1.0,
   quad_sigma=0.0,
   refine_edges=1,
   decode_sharpening=0.25,
   debug=0
)

#tello.takeoff()

center=[-1,-1] # [x,y] if it is -1 it is not on the screen
while True: # main control loop for tracking and following april tag 
    move_dir = {"up":False,      "down":False, 
            "left":False,    "right":False, 
            "forward":False, "backward":False,
            "cw":False,      "ccw":False
           } 
    # If ESC is pressed then stop
    key = cv2.waitKey(1) & 0xff

    if key == 27: # ESC
        if isTello and moving!=0:
            tello.land()
        break
    elif key == ord(' '):
        print('space')
        if isTello and moving!=0:
            tello.takeoff()
        break
    elif key == ord('a'):
        print('a')
    elif key == ord('A'):
        print('A')
    
    try:
        ret=False
        retry=0
        while retry<20:
            if isTello:  # Get the frame reader from Tello Cam
                frame = tello.get_frame_read().frame
                #frame = cv2.flip(tello.get_frame_read().frame, 1) # TTello is atomaticall flipped so april tag is detected
                if frame.any():
                    ret=True
            else: # get a frame from webCam
                ret, frame = cap.read() # Just getting actual Webcam resolution.
            if ret==False:
                retry+=1
                sleep(0.5)
            else:
                break
                
                    
        if ret==False and retry>20:
            print('openCV : Failed to capture!!!')
            exit(1)
    
    except cv2.error as e:
        print(f'openCV reading Error {e}')
        exit(1)

    if isTello==False:
        frame = cv2.resize(frame, dsize=(1280,720), interpolation=cv2.INTER_CUBIC)
    
    grey_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    d=at_detector.detect(grey_frame)


    if isTello:
        f=cv2.flip(frame, 1)

    f=frame
    cen_x, cen_y = f.shape[1]>>1, f.shape[0]>>1
    adj_x, adj_y=0,0  # Tello camera look down. need rect move down.
    if isTello:
        adj_x=(cen_x>>2)-48
        adj_y=cen_y>>3
    rect_pt1 = (cen_x-(cen_x>>2)-adj_x, cen_y-(cen_y>>2)-adj_y) 
    rect_pt2 = (cen_x+(cen_x>>2)+adj_x, cen_y+(cen_y>>2)+adj_y)
    # f = cv2.rectangle(f, rect_pt1, rect_pt2,
    #         (0, 255, 255), 5)
    


    if len(d)==0:
        f = cv2.rectangle(f, rect_pt1, rect_pt2,
        (0, 0, 255), 5)    
    
    if len(d)==0: # if april tag isnt found sleep for 0.5 seconds and look again
        center=[-1, -1]
        if isTello:
            grey_frame=cv2.flip(grey_frame, 1)
            
        #cv2.imshow("Tello View", cv2.flip(grey_frame, 1))
        frame=cv2.flip(frame, 1)
        cv2.imshow("Tello View",frame)
        #sleep(0.5)
        continue

    for r in d: #loops through all the april tags and finds id 125
        if r.tag_id!=125: #if the tag_id is 125 continue
            print(f'tag_id: {r.tag_id}')
            center=[-1, -1]
            continue


        
        SortCorners(r.corners)
        (ptA, ptB, ptC, ptD) = ( (int(n[0]), int(n[1]) ) for n in r.corners )
        center=[int(frame.shape[1]-r.center[0]), int(r.center[1])] # since the camera will be flipped switch the corners
        
        # draw the bounding box of the AprilTag detection
        # (255, 0, 0) : BlUE
        cv2.line(frame, ptA, ptB, (255, 0, 0), 5) # BLUE   top-right to btm-right (will be flip L-R)
        cv2.line(frame, ptB, ptD, (0, 255, 0), 5) # GRN    btm-right to btm-left (will be flip)
        cv2.line(frame, ptA, ptC, (0, 0, 255), 5) # RED    top-right to top-left (will be flip)
        cv2.line(frame, ptC, ptD, (255, 0, 255), 5) #B+R   top-left  to btm-left (will be flip)
        '''
        font = cv2.FONT_HERSHEY_SIMPLEX
        fontScale = 1
        color = (64, 64, 64)
        thickness = 2
        
        frame=cv2.putText(frame, 'A', ptA, font,fontScale, color, thickness, cv2.LINE_AA)
        frame=cv2.putText(frame, 'B', ptB, font,fontScale, color, thickness, cv2.LINE_AA)
        frame=cv2.putText(frame, 'C', ptC, font,fontScale, color, thickness, cv2.LINE_AA)
        frame=cv2.putText(frame, 'D', ptD, font,fontScale, color, thickness, cv2.LINE_AA)
        '''
       
        up_edge= (ptC[0]-ptA[0])*(ptC[0]-ptA[0]) + (ptC[1]-ptA[1])*(ptC[1]-ptA[1]) #상  1m : 9000~10000
        dn_edge= (ptD[0]-ptB[0])*(ptD[0]-ptB[0]) + (ptD[1]-ptB[1])*(ptD[1]-ptB[1]) #하  1.5m : 4000
        
        left_edge = (ptB[0]-ptA[0])*(ptB[0]-ptA[0]) + (ptB[1]-ptA[1])*(ptB[1]-ptA[1]) #좌  
        right_edge= (ptD[0]-ptC[0])*(ptD[0]-ptC[0]) + (ptD[1]-ptC[1])*(ptD[1]-ptC[1]) #우

        if r.tag_id == 125:
            f = cv2.rectangle(f, rect_pt1, rect_pt2,
            (0, 255, 255), 5)

        
        
        #frame=cv2.putText(frame, f"T{up_edge}, B{dn_edge}, L{left_edge}, R{right_edge}", (100,200), cv2.FONT_HERSHEY_SIMPLEX,
        #                  1, (255,0,0), 2, cv2.LINE_AA)
    

    

    
    r.center=(frame.shape[1]-r.center[0], r,center[1])
    dir_text=''
    if center[0]<rect_pt1[0]:
#        if isTello:
        move_dir["right"]=True
        dir_text+='right '
#        else:
#            move_dir["left"]=True
        
    if center[0]>rect_pt2[0]:
#        if isTello:
        move_dir["left"]=True
        dir_text+='left '
#        else:
#            move_dir["right"]=True
    
    if center[1]<rect_pt1[1]:
        move_dir["up"]=True
        dir_text+='up '

    if center[1]>rect_pt2[1]:
        move_dir["down"]=True
        dir_text+='down '
    

    
    #up+dn edge가 left+right보다 짧아지면 회전을 함.
    #if the top and the bottom edge is shorter the the left and right edge then spin the drone respectivly
    if (left_edge+right_edge)-(up_edge+dn_edge) > qr_tolerance>>1:
        if right_edge>left_edge:
            move_dir["cw"]=True
            dir_text+='cw '
        elif left_edge>right_edge:
            move_dir["ccw"]=True
            dir_text+='ccw '
    
    # qr_edge_size = 14000
    # qr_tolerance = 1400*2 # qr_size/10
    if left_edge+right_edge > qr_edge_size + qr_tolerance :
        move_dir["backward"]=True
        dir_text+='backward'
        
    if left_edge+right_edge < qr_edge_size - qr_tolerance :
        move_dir["forward"]=True
        dir_text+='forward'
    
    f=cv2.putText(f, dir_text, (100,200), cv2.FONT_HERSHEY_SIMPLEX,
                          2, (255, 0, 0), 2, cv2.LINE_AA)
    
    #TurningSign(f, left_edge<right_edge)
    # 0x01: up, dn, l, r
    # 0x02: fwd, bwd
    # 0x04: cw, ccw
    # 0x07: all
    MovementSign(f, moving, rect_pt1, rect_pt2, move_dir)
    
    cv2.imshow("Tello View", f)
    sleep(0.25)
    
if isTello:
    tello.streamoff()
else:
    cap.release()  # finish capturing

cv2.destroyWindow('Tello View')
cv2.destroyAllWindows()







