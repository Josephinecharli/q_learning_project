#!/usr/bin/env python3

import rospy

import os
import keras_ocr
import cv2
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
import moveit_commander


# Path of directory on where this file is located
path_prefix = os.path.dirname(__file__) + "/action_states/"

class Move_Robot(object):
    def __init__(self):
        # Initialize this node
        rospy.init_node("Move_Robot")

        # download pre-trained model
        self.pipeline = keras_ocr.pipeline.Pipeline()

        # Once you have the pipeline, you can use it to recognize characters,
        # images is a list of images in the cv2 format
        #self.images = []

        # call the recognizer on the list of images
        #prediction_groups = self.pipeline.recognize(self.images)

        # prediction_groups is a list of predictions for each image
        # prediction_groups[0] is a list of tuples for recognized characters for img1
        # the tuples are of the formate (word, box), where word is the word
        # recognized by the recognizer and box is a rectangle in the image where the recognized words reside
        
        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size = 10)

        rospy.Subscriber("scan", LaserScan, self.recieved_scan)
        rospy.Subscriber("camera/rgb/image_raw", Image, self.recieved_image)
        #rospy.Subscriber("q_learning/robot_action",  RobotMoveDBToBlock, self.recieved_image)
    
        # the interface to the group of joints making up the turtlebot3
        # openmanipulator arm
        #self.move_group_arm = moveit_commander.MoveGroupCommander("arm")

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator gripper
        #self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")

        # Reset arm position
        #self.move_group_arm.go([0,0,0,0], wait=True)
        #print("ready")

        #self.qmatrix = 


    def recieved_scan(self, data):
        #callback for getting LIDAR
        return

    def recieved_image(self, data):
        #callback for getting image
        # converts the incoming ROS message to OpenCV format and HSV (hue, saturation, value)
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # TODO: define the upper and lower bounds for what should be considered 'yellow'
        #lower_yellow = numpy.array([0, 0, 0]) #TODO
        #upper_yellow = numpy.array([0, 0, 0]) #TODO
        #mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

                # this erases all pixels that aren't yellow
        #h, w, d = image.shape
        #search_top = int(3*h/4)
        #search_bot = int(3*h/4 + 20)
        #mask[0:search_top, 0:w] = 0
        #mask[search_bot:h, 0:w] = 0

        # using moments() function, the center of the yellow pixels is determined
        #M = cv2.moments(hsv)
        # if there are any yellow pixels found
        #if M['m00'] > 0:
            # center of the yellow pixels in the image
        #    cx = int(M['m10']/M['m00'])
        #    cy = int(M['m01']/M['m00'])

            # a red circle is visualized in the debugging window to indicate
            # the center point of the yellow pixels
         #   cv2.circle(image, (cx, cy), 20, (0,0,255), -1)

        # shows the debugging window
        cv2.imshow("window", image)
        cv2.waitKey(3)

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
