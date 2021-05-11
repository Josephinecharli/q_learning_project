#!/usr/bin/env python3

# HOW TO RUN
# open the gazebo: roslaunch q_learning_project turtlebot3_intro_robo_manipulation.launch
# open the debugger: roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch
# open the arm-mover: roslaunch turtlebot3_manipulation_moveit_config move_group.launch 
# run our code: rosrun q_learning_project sensors.py 
# roslaunch turtlebot3_manipulation_gui turtlebot3_manipulation_gui.launch


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


        # Import qmatrix, action matrix and actions
        file = open("qmatrix.csv")
        self.qmatrix = np.loadtxt(file, delimiter=',')

        # Fetch pre-built action matrix. This is a 2d numpy array where row indexes
        # correspond to the starting state and column indexes are the next states.
        #
        # A value of -1 indicates that it is not possible to get to the next state
        # from the starting state. Values 0-9 correspond to what action is needed
        # to go to the next state.
        #
        # e.g. self.action_matrix[0][12] = 5
        self.action_matrix = np.loadtxt(path_prefix + "action_matrix.txt")

        # Fetch actions. These are the only 9 possible actions the system can take.
        # self.actions is an array of dictionaries where the row index corresponds
        # to the action number, and the value has the following form:
        # { dumbbell: "red", block: 1}
        colors = ["red", "green", "blue"]
        self.actions = np.loadtxt(path_prefix + "actions.txt")
        self.actions = list(map(
            lambda x: {"dumbbell": colors[int(x[0])], "block": int(x[1])},
            self.actions
        ))


        # initialize global  variables
        self.color = 'red' # set color we are currently looking for
        self.block = 1 # set block we want to move to
        self.stop_receiving = True # set variable to control image callback function
        self.action_list = [] # create a list of the actions we want to do
        self.empty_arm_pose = [0,0.25,0.75,-1]
        self.full_arm_pose = [0,-0.25, 0.75, -1]
        self.open_gripper = [0.019,0.019]
        self.closed_gripper = [0.003,0.003]
        self.dumbbell_dist = np.inf
        self.dumbbell_x_dist = np.inf
        self.dumbbell_x_range = 0

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
        rospy.sleep(2)


        # set up publishers and subscribers
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
        self.move_group_arm.go(self.empty_arm_pose, wait=True)
        self.move_group_gripper.go(self.open_gripper, wait=True)
        print("ready")

        # run functions
        self.run()


    def recieved_scan(self, data):
        #callback for getting LIDAR
        #return
        #print("scan")
        if abs(self.dumbbell_x_dist) < 5:
            # TODO: change to min of [3,0,-3]
            self.dumbbell_dist = data.ranges[0]
        #if abs(self.num_x_dist) < 5;
        #   self.num_dist = data.ranges[0]

    def recieved_image(self, data):
        if self.stop_receiving:
            return
        #else:
            #print(self.stop_receiving)
        #self.stop_receiving = True
        #callback for getting image
        # converts the incoming ROS message to OpenCV format and HSV (hue, saturation, value)
        image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # COLOR
        # define the upper and lower bounds for colors
        lower_red = np.array([0, 70, 50]) 
        upper_red = np.array([40, 255, 255])

        lower_blue = np.array([110, 50, 50]) 
        upper_blue = np.array([130, 255, 255])

        lower_green = np.array([36, 25, 25]) 
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
        #print("mask = "+ str(mask))

        # this erases all pixels that aren't yellow
        h, w, d = image.shape
        self.dumbbell_x_range = w
        #still need to figure out what part of the img we want to search
        search_top = int(h/2)
        search_bot = int(0)
        #mask[0:search_top, 0:w] = 0

        # using moments() function, the center of the yellow pixels is determined
        M = cv2.moments(mask)
        #print("mask made")
        # if there are any color pixels found
        if M['m00'] > 0:
           # center of the yellow pixels in the image
           cx = int(M['m10']/M['m00'])
           cy = int(M['m01']/M['m00'])
           #print("cx: " + str(cx))
           #print("w: " + str(w))
           #print("d: "+ str(d))
           self.dumbbell_x_dist = ( w // 2) - cx # if this is negative its on the other side
           #print("dist: " + str(self.dumbbell_x_dist))
        cv2.imshow("window", mask)
        cv2.waitKey(3)
        

        # # DIGITS
        # self.prediction_groups = self.pipeline.recognize([image])
        # try: 
        #     #print(list(map(lambda x : x[0], self.prediction_groups[0])))
        #     num = str(self.prediction_groups[0][0][0])
        #     if num == '1':
        #         self.move_group_arm.go([0,0,0,0], wait=True) # these lines are causing errors bc error with move_group_arm
        #         print("1")
        #     elif num == '2':
        #         self.move_group_arm.go([0,0,0,0], wait=True)
        #         print("2")
        #     elif num == '3':
        #         self.move_group_arm.go([0,0,0,0], wait=True)
        #         print("3")
        # except:
        #     print("no number found")

        self.stop_receiving = True
        self.pick_dumbbell_by_color()

    #def recieved_DB_to_Block(self, data):
    #    #callback for subscriber
    #    continue

    def pick_dumbbell_by_color(self):
        #use scan to locate dumbells
        #use image to determine color position
        #move to position and pick up dumbell colored color
        #print("moving to dumbbell")
        #print("x_dist: " + str(self.dumbbell_x_dist))
        #print("dist: " + str(self.dumbbell_dist))
        twist = Twist()
        if abs(self.dumbbell_x_dist) < 7:
            twist.angular.z = 0
            if self.dumbbell_dist > 0.175:
                #drive forward
                twist.linear.x = 0.1
                self.vel_pub.publish(twist)
            else:
                twist.linear.x = 0
                self.vel_pub.publish(twist)
                self.move_group_gripper.go(self.closed_gripper, wait=True)
                print('gripper closed')
                self.move_group_arm.go(self.full_arm_pose, wait=True)
                #close arm
                #raise arm
                #call put_dumbbell_at_num()
        else:
            #rotate in the direction of blocks
            #twist.angular.z = 0.3 * self.dumbbell_x_dist // (self.dumbbell_x_range // 2)
            twist.angular.z = 0.1 * (self.dumbbell_x_dist // abs(self.dumbbell_x_dist))
            self.vel_pub.publish(twist)
        self.stop_receiving = False

    def put_dumbbell_at_num(self):
        #use scan to locate blocks
        #use image to determine block order
        #move dumbell to block labeled num
        #deposit dumbell
        return

    def read_matrix(self):
        #given current state and qmatrix, what action do I take
        #returns (color, block_num)
        final_state = False
        curr_state = 0
        while not final_state:
            poss_actions = self.qmatrix[curr_state]
            next_action = np.argmax(poss_actions)
            self.action_list.append(next_action)
            if int(self.qmatrix[curr_state][next_action]) == 100:
                final_state = True
            curr_state = np.where(self.action_matrix[curr_state] == next_action)[0][0]
        print(self.action_list)

    def run(self):
        #self.stop_receiving = False
        self.read_matrix()
        self.color = self.actions[self.action_list[0]]['dumbbell']
        self.block = self.actions[self.action_list[0]]['block']
        print(self.color)
        print(self.block)
        
        self.stop_receiving = False
        
        #self.pick_dumbbell_by_color()
        #self.pick_dumbbell_by_color()
        #self.put_dumbbell_at_num()
        #repeat 2 more times


if __name__ == "__main__":
    node = Move_Robot()
    rospy.spin()
