#!/usr/bin/env python3

import rospy
import numpy as np
import os
import randomi
from q_learning_project.msg import QMatrix
from q_learning_project.msg import QLearningReward
from q_learning_project.msg import RobotMoveDBToBlock

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
        
        self.qmatrix = [[0]*9]*64

        self.qmatrix_pub = rospy.Publisher("/q_learning/q_matrix", QMatrix, queue_size=10)
        self.db_pub = rospy.Publisher("q_learning/robot_action", RobotMoveDBToBlock, queue_size = 10)
        rospy.subscriber("/q_learning/reward", QLearningReward, self.recieved_reward)
        
        
        self.reward = 0
        self.iterations = 0
        self.curr_state = 0
        self.alpha = 1
        self.gamma = 0.8
        self.run()

    def recieved_reward(self, data):
        self.reward = data.reward
        self.iterations = data.iteration_num

    def run(self):
        poss_actions  = filter(lambda x : x >= 0, self.action_matrix[self.curr_state])
        if len(poss_actions) == 0:
            print("how did we get here")
            self.save_q_matrix()
        else:
            next_action = poss_actions[random.randrange(len(poss_actions))]
            db_color = self.actions[next_action][0]
            block_num = self.actions[next_action][1]
            self.db_pub.publish(RobotMoveDBToBlock(robot_db=db_color, block_id=block_num))
            
            next_state = self.action_matrix[0].index(next_action)# TODO: does this actually work?

            self.qmatrix[self.curr_state][next_action] += self.alpha*(self.reward+self.gamma*max(


    def save_q_matrix(self):
        # TODO: You'll want to save your q_matrix to a file once it is done to
        # avoid retraining
        return

if __name__ == "__main__":
    node = QLearning()
    rospy.spin()
