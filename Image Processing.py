'''
* Team Id :           717
* Author List :       Ronak  ,   Rohit P  ,  Shashank Rao M,  Sangameshwar V
* Filename:           task4-main.py
* Theme:              Harvestor bot
* Functions:          get_centre(list,list),get_signal(int),get_size(list,list)
* Global Variables:   pi_enable,pi_disable,tree_flag_1,tree_flag_2,upper_hsv_range,lower_hsv_range,apple_count,blueberry_count,orange_count,required_fruits_table,
                      colour_name,contour_low_limit,contour_high_limit
'''
import cv2
import numpy as np
import argparse

import imutils
import RPi.GPIO as io
import time


io.setmode(io.BCM)
pi_enable=26
pi_disable=21
tree_flag_1=4
tree_flag_2=22

io.setup(pi_enable,io.IN)
io.setup(pi_disable,io.OUT)
io.setup(tree_flag_1,io.OUT)
io.setup(tree_flag_2,io.OUT)


upper_hsv_range=[[120,155,255],[180,255,255],[15,255,255]]
lower_hsv_range=[[100,100,50],[150,50,50],[9, 50, 50]]


contour_low_limit=[[12000,5000,700],[40000,15000,2000],[28000,9000,900]] 
contour_high_limit=[[70000,10000,4000],[80000,28000,8000],[70000,25000,7000]] 
 
apple_count=[1,1,1]                              
blueberry_count=[1,1,0]
orange_count=[0,2,0]

required_fruits_table=[blueberry_count,apple_count,orange_count]

colour_name=['Blueberry','Apple','Orange']

cap=cv2.VideoCapture(-1)





'''
* Function Name:    get_centre
* Input:            upper_hsv(upper hsv range), lower_hsv(lower hsv range)
* Output:           x_centre(x-coordinate of the detected contour centre)
                    y_centre(y-coordinate of the detected contour centre)
* Logic:            The captured frame is cropped so that only the required portion of the frame is taken into consideration for scanning and then it is made to
                    undergo filtering by applying various filters of opencv like erosion,blurring, and opening .Then the centre of the detected contour is found  and
                    returned
* Example Call:     get_centre(U,L) where U and L are the 2 hsv ranges i.e list containing hue, sat and value
'''


def get_centre(upper_hsv,lower_hsv):
    ret,frame=cap.read()
    try:
        frame= frame[0:480 , 150:550]
    except:
        pass
    hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    col_low=np.array(lower_hsv)
    col_hi=np.array(upper_hsv)
    mask=cv2.inRange(hsv,col_low,col_hi)
    res=cv2.bitwise_and(hsv,frame,mask=mask)
    kernel=np.ones((1,1),np.uint8)
    img_e=cv2.erode(mask,kernel,iterations=5)
    op1=cv2.morphologyEx(img_e,cv2.MORPH_OPEN,kernel)
    med1=cv2.medianBlur(op1,15)
    op2=cv2.morphologyEx(med1,cv2.MORPH_OPEN,kernel)
    cnts = cv2.findContours(op2.copy(), cv2.RETR_EXTERNAL,
	cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if imutils.is_cv2() else cnts[1]
    #cv2.imshow('frame',frame)
    #cv2.imshow('med1',img_e)
    try:
            max_contour = max(cnts, key = cv2.contourArea)
            M = cv2.moments(max_contour)
            x_centre = int(M["m10"] / M["m00"])
            y_centre = int(M["m01"] / M["m00"])
    except:
            return(None,None)
    return(x_centre,y_centre)






'''
* Function Name:    get_size
* Input:            upper_hsv(upper hsv range), lower_hsv(lower hsv range)
* Output:           area(area of the maximum detected contour)
* Logic:            The captured frame is cropped so that only the required portion of the frame is taken into consideration for scanning and then it is made to
                    undergo filtering by applying various filters of opencv like erosion,blurring, and opening .Then the area of the detected contour is found  and
                    returned
* Example Call:     get_centre(U,L) where U and L are the 2 hsv ranges i.e list containing hue, sat and value
'''





def get_size(upper_hsv,lower_hsv):
    ret,frame=cap.read()
    try:
        frame= frame[0:480 , 150:500]
    except:
        pass
    hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    col_low=np.array(lower_hsv)
    col_hi=np.array(upper_hsv)
    mask=cv2.inRange(hsv,col_low,col_hi)
    res=cv2.bitwise_and(hsv,frame,mask=mask)
    kernel=np.ones((1,1),np.uint8)
    img_e=cv2.erode(mask,kernel,iterations=5)
    op1=cv2.morphologyEx(img_e,cv2.MORPH_OPEN,kernel)
    med1=cv2.medianBlur(op1,15)
    op2=cv2.morphologyEx(med1,cv2.MORPH_OPEN,kernel)
    cnts = cv2.findContours(op2.copy(), cv2.RETR_EXTERNAL,
	cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if imutils.is_cv2() else cnts[1] 
    try:
    	max_contour = max(cnts, key = cv2.contourArea)
    	area=cv2.contourArea(max_contour)
    	return area
    except:
	return(0)




'''
* Function Name:    get_signal
* Input:            colour_flag( stores the information about the fruits colour) 
* Output:           None
* Logic:            Based on the colour identified by the image processing code tree_flag_1 and tree_flag_2 are sent as output signals to the firebird so that it can
                    go and deposit to its respective deposition zone 
* Example Call:     get_signal(2)
'''

def get_signal(colour_flag):
    if(colour_flag==0):
        io.output(tree_flag_1,io.LOW)
        io.output(tree_flag_2,io.HIGH)
    elif(colour_flag==1):
        io.output(tree_flag_1,io.HIGH)
        io.output(tree_flag_2,io.LOW)
    elif(colour_flag==2):
        io.output(tree_flag_1,io.HIGH)
        io.output(tree_flag_2,io.HIGH)
    return()
        
while(True):
    i=0
    io.output(pi_disable,io.LOW)                        
    cx=None
    cy=None
    fruit_present=False
    while(io.input(pi_enable)==True):                  
                io.output(tree_flag_1,io.LOW)           
                io.output(tree_flag_2,io.LOW)
                i=i%3                                   
                upper_hsv=upper_hsv_range[i]
                lower_hsv=lower_hsv_range[i]
                cx,cy=get_centre(upper_hsv,lower_hsv)   
                fruit_present=False
		if(cx==None and cy==None):
                    i=i+1                              
                    
		elif(cy>=150 and cy<=300):                   
                        x=get_size(upper_hsv,lower_hsv)
                        #print(x)
                        #print(i)
                        if(x>contour_low_limit[i][2] and x<contour_high_limit[i][2]):                           
                            fruit_present=True
			    io.output(pi_disable,io.HIGH)
			    break

                        elif(x>contour_low_limit[i][1] and x<contour_high_limit[i][1]):                           
                            fruit_present=True 
                            io.output(pi_disable,io.HIGH)
			    break

                        elif(x>contour_low_limit[i][0] and x<contour_high_limit[i][0]):                                                           
			    fruit_present=True
			    io.output(pi_disable,io.HIGH)
			    break
                        
			else:                             
                            i=i+1                                                        

		else:
			continue            
	
	
    time.sleep(0.3)
    while(io.input(pi_enable)==False):
		if(fruit_present==False):
			break
		continue
	
	
    time.sleep(1)
    cx=None
    cy=None
    io.output(pi_disable,io.LOW)
    while(io.input(pi_enable)==True and fruit_present):                   
                io.output(tree_flag_1,io.LOW)          
                io.output(tree_flag_2,io.LOW)
                i=i%3                                  
                upper_hsv=upper_hsv_range[i]
                lower_hsv=lower_hsv_range[i]
                cx,cy=get_centre(upper_hsv,lower_hsv)   
                if(cx==None and cy==None):
                    i=i+1                              
                    
		elif(cy>=150 and cy<=300):                   
                        x=get_size(upper_hsv,lower_hsv)
                        #print(x)
                        #print(i)
                        if(x>contour_low_limit[i][2] and x<contour_high_limit[i][2]):                          
                            if(required_fruits_table[i][2]):                         
                                print("Small","-",colour_name[i])
                                get_signal(i)                                
                                io.output(pi_disable,io.HIGH)                   
                                time.sleep(0.5)
                                required_fruits_table[i][2]=required_fruits_table[i][2]-1    
                                break                                                         
                            else:
                                get_signal(i)                                                
                                time.sleep(2)
                                break

                        elif(x>contour_low_limit[i][1] and x<contour_high_limit[i][1]):                            
                                if(required_fruits_table[i][1]):                       
                                    print("Medium","-",colour_name[i])
                                    get_signal(i)                                           
                                    io.output(pi_disable,io.HIGH)              
                                    time.sleep(0.5)
                                    required_fruits_table[i][1]=required_fruits_table[i][1]-1      
                                    break                                                             
                                else:
                                    get_signal(i)                                                      
                                    time.sleep(2)                                  
                                    break

                        elif(x>contour_low_limit[i][0] and x<contour_high_limit[i][0]):                                                            
                                if(required_fruits_table[i][0]):                                 
                                    print("Large","-",colour_name[i])
                                    get_signal(i)                                              
                                    io.output(pi_disable,io.HIGH)                                         
                                    time.sleep(0.5)
                                    required_fruits_table[i][0]=required_fruits_table[i][0]-1       
                                    break                                                             
                                else:
                                    get_signal(i)                                                      
                                    time.sleep(2) 
                                    break

                        else:                             
                            i=i+1                                                         

		else:
		     continue            
                
cap.release()
cv2.destroyAllWindows()    
