#!/usr/bin/env python3


'''
*****************************************************************************************
*
*        		===============================================
*           		    Cosmo Logistic (CL) Theme (eYRC 2023-24)
*        		===============================================
*
*  This script should be used to implement Task 1A of Cosmo Logistic (CL) Theme (eYRC 2023-24).
*
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

## Team ID:          [ CL#2919 ]
# Author List:		[ Bhawna Chaudhary, Arjun S Nair, Ampady B R, Giridhar A P  ]
# Filename:		    task1a.py
# Functions:
#			        [ calculate_rectangle_area, detect_aruco, depthimagecb, colorimagecb, process_image, quaternion_from_euler ]
# Nodes:		    Add your publishing and subscribing node
#                   Example:
#			        Publishing Topics  - [ /tf ]
#                   Subscribing Topics - [ /camera/aligned_depth_to_color/image_raw, /etc... ]


################### IMPORT MODULES #######################

import rclpy
import sys
import cv2
import math
import tf2_ros
import tf2_py
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CompressedImage, Image
import time


##################### FUNCTION DEFINITIONS #######################

def calculate_rectangle_area(coordinates):
    '''
    Description:    Function to calculate area or detected aruco

    Args:
        coordinates (list):     coordinates of detected aruco (4 set of (x,y) coordinates)

    Returns:
        area        (float):    area of detected aruco
        width       (float):    width of detected aruco
    '''

    ############ Function VARIABLES ############

    # You can remove these variables after reading the instructions. These are just for sample.

    area = None
    width = None
    #print(len(coordinates))

    ############ ADD YOUR CODE HERE ############
    if len(coordinates) == 1:
         coordinates = np.array(coordinates).reshape(-1, 2)
         simple_coordinates = np.zeros_like(coordinates)
         center = coordinates.mean(axis = 0)#pixels
        #  print(center)
    	 
         for j, vertex in enumerate(coordinates):
            if vertex[0] < center[0]:
                if vertex[1] < center[1]:
                     simple_coordinates[0] = vertex
                else:
                    simple_coordinates[3] = vertex
            # 
                    #    
    		    # 
    			    #  
    			    #  
            else :
                if vertex[1] < center[1]:
                    simple_coordinates[1] = vertex
		    # 
                else :
                    simple_coordinates[2] = vertex
                        # 
    	# ->  Extract values from input set of 4 (x,y) coordinates 
        #   and formulate width and height of aruco detected to return 'area' and 'width'.
#    
            width = np.linalg.norm(simple_coordinates[0] - simple_coordinates[1])
            height = np.linalg.norm(simple_coordinates[1] - simple_coordinates[2])
            area = width * height
#  

    # INSTRUCTIONS & HELP : 
    #	->  Recevice coordiantes from 'detectMarkers' using cv2.aruco library 
    #       and use these coordinates to calculate area and width of aruco detected.
    #	->  Extract values from input set of 4 (x,y) coordinates 
    #       and formulate width and height of aruco detected to return 'area' and 'width'.

    ############################################

    return area, width, center


def detect_aruco(image,d_img):
    '''
    Description:    Function to perform aruco detection and return each detail of aruco detected 
                    such as marker ID, distance, angle, width, center point location, etc.

    Args:
        image                   (Image):    Input image frame received from respective camera topic

    Returns:
        center_aruco_list       (list):     Center points of all aruco markers detected
        distance_from_rgb_list  (list):     Distance value of each aruco markers detected from RGB camera
        angle_aruco_list        (list):     Angle of all pose estimated for aruco marker
        width_aruco_list        (list):     Width of all detected aruco markers
        ids                     (list):     List of all aruco marker IDs detected in a single frame 
    '''

    ############ Function VARIABLES ############

    # ->  You can remove these variables if needed. These are just for suggestions to let you get started

    # Use this variable as a threshold value to detect aruco markers of certain size.
    # Ex: avoid markers/boxes placed far away from arm's reach position  
    aruco_area_threshold = 1500

    # The camera matrix is defined as per camera info loaded from the plugin used. 
    # You may get this from /camer_info topic when camera is spawned in gazebo.
    # Make sure you verify this matrix once if there are calibration issues.
    cam_mat = np.array([[931.1829833984375, 0.0, 640.0], [0.0, 931.1829833984375, 360.0], [0.0, 0.0, 1.0]])

    # The distortion matrix is currently set to 0. 
    # We will be using it during Stage 2 hardware as Intel Realsense Camera provides these camera info.
    dist_mat = np.array([0.0,0.0,0.0,0.0,0.0])

    # We are using 150x150 aruco marker size
    size_of_aruco_m = 0.15

    # You can remove these variables after reading the instructions. These are just for sample.
    center_aruco_list = []
    distance_from_rgb_list = []
    angle_aruco_list = []
    width_aruco_list = []
    marker_ids = []
    area_aruco_list =[]
 
    ############ ADD YOUR CODE HERE ############
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    #cv2.imshow('g',gray_image)
    #   ->  Use these aruco parameters-
    #       ->  Dictionary: 4x4_50 (4x4 only until 50 aruco IDs)
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    aruco_params = cv2.aruco.DetectorParameters_create()
 
    #   ->  Detect aruco marker in the image and store 'corners' and 'ids'
    #       ->  HINT: Handle cases for empty markers detection. 
    marker_corners, marker_ids, rejected_candidates = cv2.aruco.detectMarkers(gray_image, aruco_dict, parameters=aruco_params)
    # print(marker_ids)

    in_image_with_markers = []

    tvecj = []


    if marker_ids is not None:
        #   ->  Draw detected marker on the image frame which will be shown later
        
        in_image_with_markers = cv2.aruco.drawDetectedMarkers(image, marker_corners, marker_ids) # drawn detected markers on input image
        #cv2.imshow('d',image)
        #cv2.imshow('first', input_image_with_markers)

        #cv2.imshow('Detected Markers', input_image_with_markers)
    	#cv2.waitKey(0)
     	#cv2.destroyAllWindows()
        
        
    #   ->  Loop over each marker ID detected in frame and calculate area using function defined above (calculate_rectangle_area(coordinates))
        if len(marker_ids) > 0 :
            for i in range(0, len(marker_ids)):
                marker_area, marker_width, marker_center =  calculate_rectangle_area(marker_corners[i])
                # print(marker_corners[i][0][0][1])
                #z = (d_img[int(marker_corners[i][0][0][1])][int(marker_corners[i][0][0][0])]/1000)
                #print(z)

                r1 = (d_img[int(marker_corners[i][0][0][1])][int(marker_corners[i][0][0][0])]/1000)
                r2 = (d_img[int(marker_corners[i][0][1][1])][int(marker_corners[i][0][1][0])]/1000)
                r3 = (d_img[int(marker_corners[i][0][2][1])][int(marker_corners[i][0][2][0])]/1000)
                r4 = (d_img[int(marker_corners[i][0][3][1])][int(marker_corners[i][0][3][0])]/1000)
                # print(r1,r2,r3,r4)

                center_y = (marker_corners[i][0][0][1]+marker_corners[i][0][1][1]+marker_corners[i][0][2][1]+marker_corners[i][0][3][1])/4
                center_x = (marker_corners[i][0][0][0]+marker_corners[i][0][1][0]+marker_corners[i][0][2][0]+marker_corners[i][0][3][0])/4
                #print(center_x)
                d_c = (d_img[int(center_y)][int(center_x)])/1000
                # print(d_c)

            
                area_aruco_list.append(marker_area)
                width_aruco_list.append(marker_width)
                center_aruco_list.append((center_x,center_y))
                
        #   ->  Calculate center points aruco list using math and distance from RGB camera using pose estimation of aruco marker
    #       ->  HINT: You may use numpy for center points and 'estimatePoseSingleMarkers' from cv2 aruco library for pose estimation
                rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(marker_corners[i], size_of_aruco_m, cam_mat, dist_mat)
                # print(rvec)
                
                tvecj.append(tvec)

                # if( i < len(marker_ids)):
                    # print("-", marker_ids[i] , tvec)
                    # print("-", marker_ids[i] , rvec)
                distance = np.linalg.norm(tvec)
                print(distance)

                #print(distance)
                #tvec[i][0][2] = -tvec[i][0][2] 
                rotation_matrix, _ = cv2.Rodrigues(rvec)
                #print(rotation_matrix)
                yaw, pitch, roll = cv2.RQDecomp3x3(rotation_matrix)[0]
                # roll_degrees =np.degrees(roll)
                # pitch_degrees = np.degrees(pitch)
                # yaw_degrees = np.degrees(yaw)
                
                angle_aruco_list.append((roll, pitch, yaw))
                distance_from_rgb_list.append(d_c)
            
        #   ->  Draw frame axes from coordinates received using pose estimation
        #       ->  HINT: You may use 'cv2.drawFrameAxes'
                if(distance<2):
                    
                    cv2.drawFrameAxes(image, cam_mat, dist_mat, rvec, tvec, 1)

                else:
                    marker_ids = np.delete(marker_ids,i,0)

                #cv2.imshow("image", image)
                #cv2.waitKey(0)
            #print(center_aruco_list)
                    
            # print(area_aruco_list)
    return rotation_matrix,tvecj ,center_aruco_list, distance_from_rgb_list, angle_aruco_list, width_aruco_list, marker_ids, in_image_with_markers

    # INSTRUCTIONS & HELP : 

    #	->  Convert input BGR image to GRAYSCALE for aruco detection

    #   ->  Use these aruco parameters-
    #       ->  Dictionary: 4x4_50 (4x4 only until 50 aruco IDs)

    #   ->  Detect aruco marker in the image and store 'corners' and 'ids'
    #       ->  HINT: Handle cases for empty markers detection. 

    #   ->  Draw detected marker on the image frame which will be shown later

    #   ->  Loop over each marker ID detected in frame and calculate area using function defined above (calculate_rectangle_area(coordinates))

    #   ->  Remove tags which are far away from arm's reach positon based on some threshold defined

    #   ->  Calculate center points aruco list using math and distance from RGB camera using pose estimation of aruco marker
    #       ->  HINT: You may use numpy for center points and 'estimatePoseSingleMarkers' from cv2 aruco library for pose estimation

    #   ->  Draw frame axes from coordinates received using pose estimation
    #       ->  HINT: You may use 'cv2.drawFrameAxes'

    ############################################

    #return center_aruco_list, distance_from_rgb_list, angle_aruco_list, width_aruco_list, ids

def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion (w in last place)
    quat = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4
    q[0] = cy * cp * cr + sy * sp * sr
    q[1] = cy * cp * sr - sy * sp * cr
    q[2] = sy * cp * sr + cy * sp * cr
    q[3] = sy * cp * cr - cy * sp * sr

    return q

##################### CLASS DEFINITION #######################

class aruco_tf(Node):
    '''
    ___CLASS___

    Description:    Class which servers purpose to define process for detecting aruco marker and publishing tf on pose estimated.
    '''

    def __init__(self):
        '''
        Description:    Initialization of class aruco_tf
                        All classes have a function called __init__(), which is always executed when the class is being initiated.
                        The __init__() function is called automatically every time the class is being used to create a new object.
                        You can find more on this topic here -> https://www.w3schools.com/python/python_classes.asp
        '''

        super().__init__('aruco_tf_publisher')                                          # registering node

        ############ Topic SUBSCRIPTIONS ############

        self.color_cam_sub = self.create_subscription(Image, '/camera/color/image_raw', self.colorimagecb, 10)
        self.depth_cam_sub = self.create_subscription(Image, '/camera/aligned_depth_to_color/image_raw', self.depthimagecb, 10)

        ############ Constructor VARIABLES/OBJECTS ############

        image_processing_rate = 0.2                                                     # rate of time to process image (seconds)
        self.bridge = CvBridge()                                                        # initialise CvBridge object for image conversion
        self.tf_buffer = tf2_ros.buffer.Buffer()                                        # buffer time used for listening transforms
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.br = tf2_ros.TransformBroadcaster(self)                                    # object as transform broadcaster to send transform wrt some frame_id
        self.timer = self.create_timer(image_processing_rate, self.process_image)       # creating a timer based function which gets called on every 0.2 seconds (as defined by 'image_processing_rate' variable)
        
        self.cv_image = None                                                            # colour raw image variable (from colorimagecb())
        self.depth_image = None                                                         # depth image variable (from depthimagecb())




    def depthimagecb(self, data):
        '''
        Description:    Callback function for aligned depth camera topic. 
                        Use this function to receive image depth data and convert to CV2 image

        Args:
            data (Image):    Input depth image frame received from aligned depth camera topic

        Returns:
        '''

        ############ ADD YOUR CODE HERE ############
        try:
            # Convert the ROS Image message to a CV2 image
            self.depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
            cv2.imshow('depth',self.depth_image)
            cv2.waitKey(1)
            # Now, you have the depth image in CV2 format and can perform further processing on it
            # print(self.depth_image.dtype)
            # print(self.depth_image.shape)
            # print(self.depth_image.size)
        except CvBridgeError as e:
            print(e)

        # INSTRUCTIONS & HELP : 

        #	->  Use data variable to convert ROS Image message to CV2 Image type

        #   ->  HINT: You may use CvBridge to do the same

        ############################################


    def colorimagecb(self, data):
        '''
        Description:    Callback function for colour camera raw topic.
                        Use this function to receive raw image data and convert to CV2 image

        Args:
            data (Image):    Input coloured raw image frame received from image_raw camera topic

        Returns:
        '''

        ############ ADD YOUR CODE HERE ############
        try:
            # Convert the ROS Image message to a CV2 image
            self.cv_image  = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
            #cv2.imshow("real",self.cv_image)
            #cv2.imshow("Image window", self.cv_image)
            
            # Now, you have the color image in CV2 format and can perform further processing on it

        except CvBridgeError as e:
            print(e)

        # INSTRUCTIONS & HELP : 

        #	->  Use data variable to convert ROS Image message to CV2 Image type

        #   ->  HINT:   You may use CvBridge to do the same
        #               Check if you need any rotation or flipping image as input data maybe different than what you expect to be.
        #               You may use cv2 functions such as 'flip' and 'rotate' to do the same

        ############################################


    def process_image(self):
        '''
        Description:    Timer function used to detect aruco markers and publish tf on estimated poses.

        Args:
        Returns:
        '''

        ############ Function VARIABLES ############

        # These are the variables defined from camera info topic such as image pixel size, focalX, focalY, etc.
        # Make sure you verify these variable values once. As it may affect your result.
        # You can find more on these variables here -> http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html
        
        sizeCamX = 1280
        sizeCamY = 720
        centerCamX = 640 
        centerCamY = 360
        focalX = 931.1829833984375
        focalY = 931.1829833984375
            

        ############ ADD YOUR CODE HERE ############
        rotation_matix,tvecj , center_aruco_list, distance_from_rgb_list, angle_aruco_list, width_aruco_list, marker_ids,input_image_with_markers = detect_aruco(self.cv_image,self.depth_image)
        #   ->  Loop over detected box ids received to calculate position and orientation transform to publish TF 
        if(marker_ids is not None):
            for j, id in enumerate(marker_ids) :
            

                #   ->  Use this equation to correct the input aruco angle received from cv2 aruco function 'estimatePoseSingleMarkers' here
                #       It's a correction formula- 
                #       angle_aruco = (0.788*angle_aruco) - ((angle_aruco**2)/3160)



                #angle_aruco_list[j][2] = (0.788*angle_aruco_list[j][2]) - ((angle_aruco_list[j][2]*2)/3160)
                # angle_aruco_list[j] = ((0.788 * angle_aruco_list[j][0]) - ((angle_aruco_list[j][0] ** 2) / 3160), (0.788 * angle_aruco_list[j][1]) - ((angle_aruco_list[j][1] ** 2) / 3160), (0.788 * angle_aruco_list[j][2]) - ((angle_aruco_list[j][2] ** 2) / 3160))



                #   ->  Use center_aruco_list to get realsense depth and log them down. (divide by 1000 to convert mm to m)
                depth = distance_from_rgb_list[j]
                

    #   ->  Then calculate quaternions from    roll pitch yaw (where, roll and pitch are 0 while yaw is corrected aruco_angle)
                # roll = angle_aruco_list[j][2]#angle_aruco_list[j][0]#angle_aruco_list[j][0] # Assuming roll is 0
                # pitch =angle_aruco_list[j][0]#angle_aruco_list[j][1]  # Assuming pitch is 0
                yaw = (0.788*angle_aruco_list[j][2]) - ((angle_aruco_list[j][2]**2)/3160) # Corrected ArUco angle

                quaternion = quaternion_from_euler(0,0,yaw)#quaternion_from_euler(roll, pitch, yaw)
                # quaternion = quaternion*rotation_matix
                print(quaternion)

            #   ->  Use this formula to rectify x, y, z based on focal length, center value and size of image
            #       x = distance_from_rgb * (sizeCamX - cX - centerCamX) / focalX
            #       y = distance_from_rgb * (sizeCamY - cY - centerCamY) / focalY
            #       z = distance_from_rgb
            #       where, 
            #               cX, and cY from 'center_aruco_list'
            #               distance_from_rgb is depth of object calculated in previous step
            #               sizeCamX, sizeCamY, centerCamX, centerCamY, focalX and focalY are defined above 
                cX = center_aruco_list[j][0]
                cY = center_aruco_list[j][1]
                # som=(sizeCamX - cX - centerCamX) / focalX
                #print(som,'-')
                # x = (depth * (sizeCamX - cX - centerCamX) / focalX)/1000
                # y = (depth * (sizeCamY - cY - centerCamY) / focalY)/1000


                # z = (self.depth_image[int(cY)][int(cX)]/1000)
                # print(x)
                # print(y)
                # print(depth)

                # cv2.imshow(' Markers', input_image_with_markers)

                circle_marked_image = cv2.circle(input_image_with_markers, (int(cX),int(cY)), 0, (0, 0 , 255), -1)
                cv2.imshow('Detected Markers', circle_marked_image)
                cv2.waitKey(1)

                marker_id = int(marker_ids[j])       
                t = TransformStamped()
                t.header.stamp = self.get_clock().now().to_msg()                        # select transform time stamp as current clock time
            # frame IDs

                t.header.frame_id = 'camera_link'                                         # parent frame link with whom to send transform
                t.child_frame_id = f'cam_{marker_id}'                                             # child frame link from where to send transfrom
                                                

                t.transform.translation.x = float(depth)#tvecj[j][0][0][2]*10 # tvecj[j][0][0][2]
                # print(marker_ids[j],tvecj[j][0][0][2]*10 , -1*tvecj[j][0][0][0] *10, depth)
                t.transform.translation.y = -1*tvecj[j][0][0][0] # + 640 #float(y)
                t.transform.translation.z = -1*tvecj[j][0][0][1]# + 360

                t.transform.rotation.x = -quaternion[0]
                t.transform.rotation.y = quaternion[1]
                t.transform.rotation.z = quaternion[2]
                t.transform.rotation.w = -quaternion[3]

                self.br.sendTransform(t) 

                #time.sleep(10)




            #   ->  Then finally lookup transform between base_link and obj frame to publish the TF
            #       You may use 'lookup_transform' function to pose of obj frame w.r.t base_link 

            #   ->  And now publish TF between object frame and base_link
            #       Use the following frame_id-
            #           frame_id = 'base_link'
            #           child_frame_id = 'obj_<marker_id>'          Ex: obj_20, where 20 is aruco marker ID

                try:
                    # Get the transformation from 'camera_link' to 'cam_<marker_id>'
                    transform_base_to_marker = self.tf_buffer.lookup_transform('base_link', f'cam_{marker_id}', rclpy.time.Time())
                    # transform_base_to_camera = self.tf_buffer.lookup_transform('base_link', 'camera_link', rclpy.time.Time())

                    # Calculate the transformation from 'base_link' to 'obj_<marker_id>' using the transformation received above
                    #transform_base_to_obj = transform_camera_to_marker.transform

                    # Modify the frame IDs to match your naming convention
                    #transform_base_to_obj.header.frame_id = 'base_link'
                    #transform_base_to_obj.child_frame_id = f'obj_{marker_id}'

                    # Publish the transformation
                    #self.br.sendTransform(transform_base_to_obj)

                    transform_base_to_obj = TransformStamped()
                    transform_base_to_obj.header.stamp = self.get_clock().now().to_msg()   
                    transform_base_to_obj.header.frame_id = 'base_link'
                    transform_base_to_obj.child_frame_id = f'obj_{marker_id}'
                    transform_base_to_obj.transform = transform_base_to_marker.transform #+ transform_base_to_camera.transform
                    
                    transform_base_to_obj.transform.rotation.x = -quaternion[0]
                    transform_base_to_obj.transform.rotation.y = quaternion[1]
                    transform_base_to_obj.transform.rotation.z = quaternion[2]
                    transform_base_to_obj.transform.rotation.w = -quaternion[3]
                    # print(transform_camera_to_marker.transform)

                    # Publish the transformation
                    self.br.sendTransform(transform_base_to_obj)

                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    self.get_logger().warn("TF lookup failed")


            #   ->  At last show cv2 image window having detected markers drawn and center points located using 'cv2.imshow' function.
            #       Refer MD book on portal for sample image -> https://portal.e-yantra.org/

                
            # cv2.waitKey(0)


        # INSTRUCTIONS & HELP : 

        #	->  Get aruco center, distance from rgb, angle, width and ids list from 'detect_aruco_center' defined above

        #   ->  Loop over detected box ids received to calculate position and orientation transform to publish TF 

        #   ->  Use this equation to correct the input aruco angle received from cv2 aruco function 'estimatePoseSingleMarkers' here
        #       It's a correction formula- 
        #       angle_aruco = (0.788*angle_aruco) - ((angle_aruco**2)/3160)

        #   ->  Then calculate quaternions from roll pitch yaw (where, roll and pitch are 0 while yaw is corrected aruco_angle)

        #   ->  Use center_aruco_list to get realsense depth and log them down. (divide by 1000 to convert mm to m)

        #   ->  Use this formula to rectify x, y, z based on focal length, center value and size of image
        #       x = distance_from_rgb * (sizeCamX - cX - centerCamX) / focalX
        #       y = distance_from_rgb * (sizeCamY - cY - centerCamY) / focalY
        #       z = distance_from_rgb
        #       where, 
        #               cX, and cY from 'center_aruco_list'
        #               distance_from_rgb is depth of object calculated in previous step
        #               sizeCamX, sizeCamY, centerCamX, centerCamY, focalX and focalY are defined above

        #   ->  Now, mark the center points on image frame using cX and cY variables with help of 'cv2.cirle' function 

        #   ->  Here, till now you receive coordinates from camera_link to aruco marker center position. 
        #       So, publish this transform w.r.t. camera_link using Geometry Message - TransformStamped 
        #       so that we will collect it's position w.r.t base_link in next step.
        #       Use the following frame_id-
        #           frame_id = 'camera_link'
        #           child_frame_id = 'cam_<marker_id>'          Ex: cam_20, where 20 is aruco marker ID

        #   ->  Then finally lookup transform between base_link and obj frame to publish the TF
        #       You may use 'lookup_transform' function to pose of obj frame w.r.t base_link 

        #   ->  And now publish TF between object frame and base_link
        #       Use the following frame_id-
        #           frame_id = 'base_link'
        #           child_frame_id = 'obj_<marker_id>'          Ex: obj_20, where 20 is aruco marker ID

        #   ->  At last show cv2 image window having detected markers drawn and center points located using 'cv2.imshow' function.
        #       Refer MD book on portal for sample image -> https://portal.e-yantra.org/

        #   ->  NOTE:   The Z axis of TF should be pointing inside the box (Purpose of this will be known in task 1B)
        #               Also, auto eval script will be judging angular difference aswell. So, make sure that Z axis is inside the box (Refer sample images on Portal - MD book)

        ############################################


##################### FUNCTION DEFINITION #######################

def main():
    '''
    Description:    Main function which creates a ROS node and spin around for the aruco_tf class to perform it's task
    '''

    rclpy.init(args=sys.argv)                                       # initialisation

    node = rclpy.create_node('aruco_tf_process')                    # creating ROS node

    node.get_logger().info('Node created: Aruco tf process')        # logging information

    aruco_tf_class = aruco_tf()                                     # creating a new object for class 'aruco_tf'

    rclpy.spin(aruco_tf_class)                                      # spining on the object to make it alive in ROS 2 DDS
    

    aruco_tf_class.destroy_node()                                   # destroy node after spin ends

    rclpy.shutdown()                                                # shutdown process


if __name__ == '__main__':
    '''
    Description:    If the python interpreter is running that module (the source file) as the main program, 
                    it sets the special __name__ variable to have a value “__main__”. 
                    If this file is being imported from another module, __name__ will be set to the module’s name.
                    You can find more on this here -> https://www.geeksforgeeks.org/what-does-the-if-__name__-__main__-do/
    '''

    main()
