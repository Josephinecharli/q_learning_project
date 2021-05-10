#!/usr/bin/env python3

# HOW TO RUN
# open the gazebo: roslaunch q_learning_project turtlebot3_intro_robo_manipulation.launch
# open the debugger: roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch
# open the arm-mover: roslaunch turtlebot3_manipulation_moveit_config move_group.launch 
# run our code: rosrun q_learning_project sensors.py 

# TODO
# figure out this error: [ INFO] [1620640383.531923153, 1527.144000000]: ABORTED: No motion plan found. No execution attempted.
# it goes with this one: 
    # [ERROR] [1620640417.810517690, 1555.693000000]: arm/arm: Unable to sample any valid states for goal tree
    # [ INFO] [1620640417.810575192, 1555.693000000]: arm/arm: Created 1 states (1 start + 0 goal)
    # [ INFO] [1620640417.810596611, 1555.693000000]: No solution found after 0.040542 seconds
    # [ INFO] [1620640417.810624034, 1555.693000000]: Unable to solve the planning problem
# There is some issue with the move arm launch file/connection that is preventing arm movements from happening



import rospy

import os, sys
import keras_ocr
import cv2, cv_bridge
import numpy as np
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
import moveit_commander


# Path of directory on where this file is located
path_prefix = os.path.dirname(__file__) + "/action_states/"

class Move_Robot(object):
    def __init__(self):
        # Initialize this node
        rospy.init_node("Move_Robot")

        file = open("qmatrix.csv")
        self.qmatrix = np.loadtxt(file, delimiter=',')

        print(self.qmatrix)

        print("set color")
        self.color = 'red' # set color we are currently looking for


        # set up camera stuff
        # set up ROS / OpenCV bridge
        self.bridge = cv_bridge.CvBridge()
        rospy.sleep(2) # wait for setup to complete to avoid errors

        # set up keras stuff
        # download pre-trained model
        self.pipeline = keras_ocr.pipeline.Pipeline()
        rospy.sleep(1) # idk if we need this

        # Once you have the pipeline, you can use it to recognize characters,

        # call the recognizer on the list of images
        self.prediction_groups = []

        # prediction_groups is a list of predictions for each image
        # prediction_groups[0] is a list of tuples for recognized characters for img1
        # the tuples are of the formate (word, box), where word is the word
        # recognized by the recognizer and box is a rectangle in the image where the recognized words reside
        

        rospy.sleep(2)

        # publish to cmd_vel
        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size = 10)

        # subscribe to scan and camera
        rospy.Subscriber("scan", LaserScan, self.recieved_scan)
        rospy.Subscriber("camera/rgb/image_raw", Image, self.recieved_image)    

        # set up robot stuff
        # the interface to the group of joints making up the turtlebot3
        # openmanipulator arm
        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator gripper
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")
        
        rospy.sleep(1)
        # Reset arm position
        self.move_group_arm.go([0,0,0,0], wait=True)
        print("ready")

        # set our global variables
        
        self.recognition_time = 3 #??? time we should wait for numbers to be recognized


    def recieved_scan(self, data):
        #callback for getting LIDAR
        return

    def recieved_image(self, data):
        #callback for getting image
        # converts the incoming ROS message to OpenCV format and HSV (hue, saturation, value)
        image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # COLOR
        # define the upper and lower bounds for colors
        lower_red = np.array([0, 70, 50]) 
        upper_red = np.array([70, 255, 255])

        lower_blue = np.array([110, 50, 50]) 
        upper_blue = np.array([130, 255, 255])

        lower_greed = np.array([36, 25, 25]) 
        upper_green = np.array([70, 255,255])

        if self.color == 'red':
            lower = lower_red
            upper = upper_red
        elif self.color == 'blue':
            lower = lower_blue
            upper = upper_blue
        else:
            lower = lower_green
            upper = upper_green

        mask = cv2.inRange(hsv, lower, upper)
        print("mask = "+ str(mask))

        # this erases all pixels that aren't yellow
        h, w, d = image.shape
        
        #still need to figure out what part of the img we want to search
        search_top = int(h/2)
        search_bot = int(0)
        #mask[0:search_top, 0:w] = 0

        # using moments() function, the center of the yellow pixels is determined
        M = cv2.moments(mask)
        # if there are any color pixels found
        if M['m00'] > 0:
           # center of the yellow pixels in the image
           cx = int(M['m10']/M['m00'])
           cy = int(M['m01']/M['m00'])

        
        # DIGITS
        self.prediction_groups = self.pipeline.recognize([image])
        try: 
            print(self.prediction_groups[0][0][0])
            num = str(self.prediction_groups[0][0][0])
            if num == '1':
                self.move_group_arm.go([0,0,0,0], wait=True) # these lines are causing errors bc error with move_group_arm
                print("1")
            elif num == '2':
                self.move_group_arm.go([0,0,0,0], wait=True)
                print("2")
            elif num == '3':
                self.move_group_arm.go([0,0,0,0], wait=True)
                print("3")
        except:
            print("no number found")


    #def recieved_DB_to_Block(self, data):
    #    #callback for subscriber
    #    continue

    def pick_dumbell_by_color(self, color):
        #use scan to locate dumbells
        #use image to determine color position
        #move to position and pick up dumbell colored color
        return

    def put_dumbell_at_num(self, num):
        #use scan to locate blocks
        #use image to determine block order
        #move dumbell to block labeled num
        #deposit dumbell
        return

    def read_matrix(self):
        #given current state and qmatrix, what action do I take
        #returns (color, block_num)
        return

    def run(self):
        #self.read_matrix()
        #self.pick_dumbell_by_color()
        #self.put_dumbell_at_num()
        #repeat 2 more times
        return


if __name__ == "__main__":
    node = Move_Robot()
    rospy.spin()
