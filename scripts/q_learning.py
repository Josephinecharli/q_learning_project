#!/usr/bin/env python3

import rospy
import numpy as np
import os
import random
from q_learning_project.msg import QMatrix, QMatrixRow
from q_learning_project.msg import QLearningReward
from q_learning_project.msg import RobotMoveDBToBlock
from std_msgs.msg import Header, String
import pandas as pd
import copy


# Path of directory on where this file is located
path_prefix = os.path.dirname(__file__) + "/action_states/"

class QLearning(object):
    def __init__(self):
        # Initialize this node
        rospy.init_node("q_learning")

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

        # Fetch states. There are 64 states. Each row index corresponds to the
        # state number, and the value is a list of 3 items indicating the positions
        # of the red, green, blue dumbbells respectively.
        # e.g. [[0, 0, 0], [1, 0 , 0], [2, 0, 0], ..., [3, 3, 3]]
        # e.g. [0, 1, 2] indicates that the green dumbbell is at block 1, and blue at block 2.
        # A value of 0 corresponds to the origin. 1/2/3 corresponds to the block number.
        # Note: that not all states are possible to get to.
        self.states = np.loadtxt(path_prefix + "states.txt")
        self.states = list(map(lambda x: list(map(lambda y: int(y), x)), self.states))
        
        # initialize qmatrix as numpy array
        self.qmatrix = np.zeros((64,9), dtype=float)

        # create our publishers and subscribers for our q_matrix and robot actions
        self.qmatrix_pub = rospy.Publisher("/q_learning/q_matrix", QMatrix, queue_size=10)
        self.db_pub = rospy.Publisher("/q_learning/robot_action", RobotMoveDBToBlock, queue_size = 10)
        rospy.Subscriber("/q_learning/reward", QLearningReward, self.recieved_reward)
        
        # initialize global variables
        self.curr_state = 0 # store current state
        self.curr_action = 0 # store current action
        self.alpha = 1
        self.gamma = 0.8
        self.prevs = np.ones(25, dtype=float) # store our previous delta values
        self.next_state = 0 # store our next calculated state
        self.previous_matrix = copy.deepcopy(self.qmatrix) # store our previous matrix
        self.index = 0 # set a counter for our delta values

        # wait for initialization
        r = rospy.Rate(1)
        r.sleep()

        # call our run function
        self.run()

    # When we recieve a reward, update our qmatrix
    # publish our qmatrix
    # calculate difference between updated matrix and previous matrices
    # if difference is below threshold, save matrix and exit
    # if not, continue iterating
    def recieved_reward(self, data):
        
        # while the matrix hasn't converged (or we haven't completed a sufficient number of iterations)
        # update new qmatrix
        self.qmatrix[self.curr_state][self.curr_action] = self.qmatrix[self.curr_state][self.curr_action] + self.alpha * (data.reward + self.gamma * max(self.qmatrix[self.next_state]) - self.qmatrix[self.curr_state][self.curr_action])
        
        # publish qmatrix
        # create a new qmatrix ms type and translate our qmatrix into it
        new_qm = QMatrix()
        temp_qm = []
        for row in self.qmatrix:
            temp_array = []
            for val in row:
                temp_array.append(int(val))
            temp_qm.append(QMatrixRow(temp_array))
        new_qm.q_matrix = temp_qm
        self.qmatrix_pub.publish(new_qm)

        # calculate difference in qmatrix
        # set variables to store total diff between matrices
        percent_delta = 0
        prev = 0
        curr = 0
        # iterate through prev and current qmatrices and calculate difference
        for i, row in np.ndenumerate(self.qmatrix):
            for j, val in np.ndenumerate(row):
                prev += int(self.previous_matrix[i][j])
                curr += int(val)
        # if there is a meaningfull diff, modify our prev array
        if curr != 0:
            percent_delta = curr - prev
            if(data.reward > 0):
                self.prevs[self.index % 25] = percent_delta
                self.index += 1
                 
                # store this qmatrix in our previous_matrix variable
                self.previous_matrix = copy.deepcopy(self.qmatrix)

        # update our current state
        self.curr_state = self.next_state

        # calculate avg delta of the last 10 meaningful changes, 
        # decide whether matrix has converged or not
        avg_delta = np.mean(self.prevs)
        # if matrix hasn't converged, compute another iteration
        if (data.iteration_num < 300 or avg_delta > 1): #and not rospy.is_shutdown): # this is a placeholder
            self.run()
        # save matrix once it has converged
        else:
            self.save_q_matrix()        


    # execute our q-learning algorithm until our matrix has converged
    # calculate possible actions at our state,
    # if there are none, reset states and rerun fn
    # otherwise, chose a current action, 
    # calculate the next state
    # publish our new action
    def run(self):
        # find a list of the possible actions at our state 
        poss_actions  = list(filter(lambda x : x >= 0, self.action_matrix[self.curr_state]))

        # check if there are any possible actions
        if len(poss_actions) == 0:
            # if there are not, reset states
            self.curr_state = 0
            self.next_state = 0
            self.curr_action = 0
            self.run()
        else:
            # otherwise, calculate next state and action
            self.curr_action = int(poss_actions[random.randrange(len(poss_actions))])
            self.next_state = np.where(self.action_matrix[self.curr_state] == self.curr_action)[0][0]  # TODO: does this actually work?

            # publish msg
            db_color = self.actions[self.curr_action]['dumbbell']
            block_num = self.actions[self.curr_action]['block']
            movement = RobotMoveDBToBlock(robot_db=db_color, block_id=block_num)
            self.db_pub.publish(movement)
        
    # save qmatrix
    def save_q_matrix(self):
        # TODO: You'll want to save your q_matrix to a file once it is done to
        # avoid retraining
        np.savetxt("qmatrix.csv", self.qmatrix, delimiter=",")
        print("qmatrix saved")

if __name__ == "__main__":
    node = QLearning()
    rospy.spin()
    
