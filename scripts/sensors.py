#!/usr/bin/env python3

# HOW TO RUN
# open the gazebo: roslaunch q_learning_project turtlebot3_intro_robo_manipulation.launch
# open the debugger: roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch
# open the arm-mover: roslaunch turtlebot3_manipulation_moveit_config move_group.launch 
# run our code: rosrun q_learning_project sensors.py 
# roslaunch turtlebot3_manipulation_gui turtlebot3_manipulation_gui.launch

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

        # import qmatrix
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
        self.action_list = [7] # create a list of the actions we want to do
        # set poses and grips for the robot
        self.empty_arm_pose = [0,0.25,0.75,-1] 
        self.full_arm_pose = [0,-0.60, 0.50, -0.75]
        self.open_gripper = [0.019,0.019]
        self.closed_gripper = [0.003,0.003]
        self.dumbbell_dist = np.inf # distance from scan to dumbell
        self.dumbbell_x_dist = np.inf # x distance to target dumbell
        self.dumbbell_x_range = 0 # width of camera
        self.moving = False # set boolean to control image recognition 
        self.picking = True # boolean to control color recognition
        self.block_dist = np.inf # set scan distance
        self.block_x_dist = 0 # set image recgonition distance from target block
        self.move_now = False # boolean to control move function
        self.rotate = False # rotate more

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

        # set up robot arm
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

    # recieve the lidar scan values from the robot
    # update block and dumbell distances
    def recieved_scan(self, data):
        # update block distance variable
        self.block_dist = data.ranges[0]

        # if the target dumell is close enough, update the scan value
        if abs(self.dumbbell_x_dist) < 5:
            # update dumbell distance to be the min value between the 3 ranges
            self.dumbbell_dist = min(data.ranges[0], data.ranges[1], data.ranges[-1])

    # callback function for camera
    def recieved_image(self, data):
        # check to make sure the previous call has been fully processed and we want to call this fn again
        #if self.stop_receiving:
        #    return
        # stop receiving until we set this to false again
        #self.stop_receiving = True
        self.prediction_groups = []
        # callback for getting image
        # converts the incoming ROS message to OpenCV format and HSV (hue, saturation, value)
        image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # set the dimensions of the image
        h, w, d = image.shape
        print("ok image found")
        cv2.imshow("window", image)
        cv2.waitKey(1)

        # if we want to look at the color of image
        if self.picking:

            # COLOR
            # define the upper and lower bounds for colors
            lower_red = np.array([0, 70, 50]) 
            upper_red = np.array([40, 255, 255])

            lower_blue = np.array([110, 50, 50]) 
            upper_blue = np.array([130, 255, 255])

            lower_green = np.array([36, 25, 25]) 
            upper_green = np.array([70, 255,255])

            # pick which color to look at based on our global variable
            if self.color == 'red':
                lower = lower_red
                upper = upper_red
            elif self.color == 'blue':
                lower = lower_blue
                upper = upper_blue
            else:
                lower = lower_green
                upper = upper_green

            # set the mask for the image
            mask = cv2.inRange(hsv, lower, upper)

            # update our width variable
            self.dumbbell_x_range = w

            #still need to figure out what part of the img we want to search (dont end up using)
            search_top = int(h/2)
            search_bot = int(0)

            # using moments() function, the center of the yellow pixels is determined
            M = cv2.moments(mask)

            # if there are any color pixels found
            if M['m00'] > 0:
                # center of the color pixels in the image
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                # update our dist variable to the target dumbell
                self.dumbbell_x_dist = ( w // 2) - cx # if this is negative its on the other side

            # open debugging window
            # cv2.imshow("window", mask)
            # cv2.waitKey(3)

            # call function to move to dumbell
            self.pick_dumbbell_by_color()
        
        # if we want to look at the image recognition data
        elif self.moving:
            print("starting image recognition")
            # DIGITS
            # read image in from ml algo
            self.prediction_groups = self.pipeline.recognize([image])
            print(self.prediction_groups)
            #cv2.imshow("window", image)
            #cv2.waitKey(3)
            # attempt to access data
            # if there is no image recognition data, call move fn again
            try: 
                print(list(map(lambda x : x[0], self.prediction_groups[0])))
                # create list of recognized numbers
                poss_nums = list(map(lambda x : x[0], self.prediction_groups[0]))

                # if target block has been recognized, extract values from it
                if str(self.block) in poss_nums:
                    print("block found")
                    # calculate the x dist to block and update global variable
                    cx = w // 2 - self.prediction_groups[0][poss_nums.index(str(self.block))][1][0][0]
                    print(cx)
                    self.block_x_dist = cx
                    
                    # set variables to stop image recognition and start turning towards target black
                    self.moving = False
                    self.move_now = True
                    print("set to true")

                # if target block hasn't been found, try again
                else:
                    print("block not found")
                    # if no numbers were detected,
                    # run again and wait till numbers were detected
                    if poss_nums == []:
                        print("empty")
                        self.put_dumbbell_at_num()

                    # if numbers were detected, but not our target number,
                    # rotate robot slightly and search again
                    else:
                        print("need to rotate more")
                        self.rotate = True

                        # call fun again
                        self.put_dumbbell_at_num()

            # if no numbers are recognized, try again
            except:
                print("error")
                #self.put_dumbbell_at_num()
            
            # if we have identified location of target block, 
            # start move function to it
            if self.move_now:
                # reset move function to false again
                self.move_now = False
                print("move to block")
                # call move function
                self.move_to_block()
        else:
            self.turn_to_blocks()


    # once the target dumbell has been identified: 
    # move to it and pick it up
    def pick_dumbbell_by_color(self):
        print("pick by color")
        # initialize twist variable
        twist = Twist()

        # if the robot is pointed towards the robot, start moving towards it
        if abs(self.dumbbell_x_dist) < 7:
            # stop rotation of the robot
            twist.angular.z = 0

            # if the robot is too far from the robot, keep moving forwards
            if self.dumbbell_dist > 0.18:
                #drive forward
                twist.linear.x = 0.1
                self.vel_pub.publish(twist)

                # run image callback function again
                #self.stop_receiving = False

            # otherwise, begin trying to pick the robot up
            else:
                #print("close to dumbbell")
                #stop moving forward
                twist.linear.x = 0
                self.vel_pub.publish(twist)

                # close gripper and raise arm
                self.move_group_gripper.go(self.closed_gripper, wait=True)
                print('gripper closed')
                self.move_group_arm.go(self.full_arm_pose, wait=True)

                # move backwards for 2 seconds to be closer to the blocks
                # necessary to recognize first block
                #rospy.sleep(1)
                twist.linear.x = -0.2
                self.vel_pub.publish(twist)
                #rospy.sleep(2)
                twist.linear.x = 0
                self.vel_pub.publish(twist)

                # stop looking for color of robot
                self.picking = False
                # stop using image callback fn for now
                #print("stopping image from pick_by_color")
                #self.stop_receiving = True

                # turn robot to first block
                self.turn_to_blocks()
    
        else:
            #rotate in the direction of blocks
            twist.angular.z = 0.1 * (self.dumbbell_x_dist // abs(self.dumbbell_x_dist))
            self.vel_pub.publish(twist)

            # run image callback function again
            #self.stop_receiving = False


    # once the dumbell has been picked up, 
    # turn until first block is found
    def turn_to_blocks(self):
        #print("turning to block")
        # initialize twist variable
        twist = Twist()

        # set linear speed to 0 and rotation to .1
        twist.linear.x = 0
        twist.angular.z = 0.1
        self.vel_pub.publish(twist)
        #print("turning")
        #print(self.block_dist)

        # wait until we see a block and then stop
        # if we dont see a block, call the function again
        if (self.block_dist < 1.5) or (str(self.block_dist) == 'inf'):
            # sleep for 1 second to not break code
            #rospy.sleep(1)
            #self.turn_to_blocks()
            print("test")
        # if block has been found stop turning
        else:
            #print("stop")
        #stop turning
            twist.angular.z = 0
            self.vel_pub.publish(twist)
        # tell robot to find block
            self.put_dumbbell_at_num()


    # start robot looking for correct block number
    def put_dumbbell_at_num(self):
        # start looking for a block number
        print("moving dumbbell to block")   
        if (self.rotate):   
            # create a twist variable
            twist = Twist()

            # rotate for 1 second
            twist.linear.x = 0
            twist.angular.z = 0.1
            self.vel_pub.publish(twist)
            print("rotating")
            rospy.sleep(1)

            # stop turning
            print("stop rotating")
            twist.angular.z = 0
            self.vel_pub.publish(twist)
            self.rotate = False
        print("calling img function again")
        self.stop_receiving = False
        self.moving = True

    # when the target block has been found:
    # move to the block
    def move_to_block(self):
        print("moving to block")
        # initialize twist variable
        twist = Twist()

        # if the robot is pointed towards the block
        # begin moving towards it
        if abs(self.block_x_dist) < 7:
            # stop rotating robot
            twist.angular.z = 0

            # if robot is not close enough to block
            if self.block_dist > 0.4:
                #drive forward
                twist.linear.x = 0.1
                self.vel_pub.publish(twist)
                print(self.block_dist)
                # sleep and call function again to check if robot is close enough now
                rospy.sleep(1)
                self.move_to_block()

            # stop moving and deposit robot
            else:
                twist.linear.x = 0
                self.vel_pub.publish(twist)

        # otherwise keep rotating until pointed towards the block
        else:
            # rotate in the direction of blocks
            twist.angular.z = 0.1 * (self.block_x_dist // abs(self.block_x_dist))
            self.vel_pub.publish(twist)
            sleep_time = abs(self.block_x_dist)/100
            print(sleep_time)
            rospy.sleep(sleep_time)
            print("done")

            # stop rotating and call image call back function again
            twist.angular.z = 0
            self.vel_pub.publish(twist)
            self.stop_receiving = False
            self.moving = True

    # read the qmatrix file and update action list with the list of actions the robot should perform
    def read_matrix(self):
        # initialize state to 0
        final_state = False
        curr_state = 0

        # while the reward of 100 hasn't been found:
        # find the next action
        while not final_state:
            # create an array of possible actions at the state and chose the action with the highest reward
            poss_actions = self.qmatrix[curr_state]
            next_action = np.argmax(poss_actions)

            # add the action to our action list
            self.action_list.append(next_action)

            # if the reward for this action is 100, we are done
            if int(self.qmatrix[curr_state][next_action]) == 100:
                final_state = True

            # calculate the next state of the robot after perfoming the previous action
            curr_state = np.where(self.action_matrix[curr_state] == next_action)[0][0]

        print(self.action_list)

    # run function
    # start reading the qmatrix
    # update the target block and color and go from there
    def run(self):
        #self.stop_receiving = False
        self.read_matrix()
        self.color = self.actions[self.action_list[0]]['dumbbell']
        self.block = self.actions[self.action_list[0]]['block']       
        self.stop_receiving = False

# run our node and keep rospy spinnning
if __name__ == "__main__":
    node = Move_Robot()
    rospy.spin()
