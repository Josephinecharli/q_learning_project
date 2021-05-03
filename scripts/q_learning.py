#!/usr/bin/env python3

import rospy
import numpy as np
import os
import random
from q_learning_project.msg import QMatrix
from q_learning_project.msg import QLearningReward
from q_learning_project.msg import RobotMoveDBToBlock

from std_msgs.msg import Header, String

import pandas as pd


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
        
        # initialize qmatrix
        self.qmatrix = [[0]*9]*64

        # create our publishers for our q_matrix and robot actions
        self.qmatrix_pub = rospy.Publisher("/q_learning/q_matrix", QMatrix, queue_size=10)
        self.db_pub = rospy.Publisher("/q_learning/robot_action", RobotMoveDBToBlock, queue_size = 10)
        rospy.Subscriber("/q_learning/reward", QLearningReward, self.recieved_reward)
        
        # initialize global variables
        self.reward = 0
        self.iterations = 0
        self.curr_state = 0
        self.alpha = 1
        self.gamma = 0.8
        self.prevs = [1]*5 # probably make this larger
        self.next_action = -1
        self.run()

    # update our reward and iterations variables
    def recieved_reward(self, data):
        self.reward = data.reward
        self.iterations = data.iteration_num
        
        # while the matrix hasn't converged (or we haven't completed a sufficient number of iterations)
        if(sum(self.prevs) > 0 or self.iterations < 5000): #and not rospy.is_shutdown): # this is a placeholder

            # find a list of the possible actions at our state 
            poss_actions  = list(filter(lambda x : x >= 0, self.action_matrix[self.curr_state]))

            # check if there are any possible actions
            if len(poss_actions) == 0:
                print("no possible actions")
                print(self.curr_state)
                self.curr_state = 0
                #self.save_q_matrix()
            else:
                # calculate next state 
                next_state = np.where(self.action_matrix[self.curr_state] == self.next_action)[0][0]  # TODO: does this actually work?
                
                # store previous value from our qmatrix and set new one
                prev_val = self.qmatrix[self.curr_state][self.next_action]
                self.qmatrix[self.curr_state][self.next_action] += self.alpha*(self.reward+self.gamma*max(self.qmatrix[next_state])-self.qmatrix[self.curr_state][self.next_action])
                
                # publish qmatrix
                header = Header(stamp=rospy.Time.now(), frame_id="matrix_update")
                self.qmatrix_pub.publish(QMatrix(header=header, q_matrix = self.qmatrix))

                # calculate difference in qmatrix and update our prev array
                delta = abs(prev_val - self.qmatrix[self.curr_state][self.next_action]) # re-evaluate this later
                self.prevs[self.iterations % 5] = delta

                # TESTING
                print("state: " + str(self.curr_state))
                print("actions: " + str(poss_actions))
                print("next action: " + str(self.next_action))
                print("next state: " + str(next_state))
                
                # update our current state
                self.curr_state = next_state
                
                
                
                # chose next action from possible actions and publish to Robot actions
                self.next_action = int(poss_actions[random.randrange(len(poss_actions))])
                db_color = self.actions[self.next_action]['dumbbell']
                block_num = self.actions[self.next_action]['block']
                
                r = rospy.Rate(1)
                r.sleep()
                
                self.db_pub.publish(RobotMoveDBToBlock(robot_db=db_color, block_id=block_num))
                print("recursive pub")

        else:
        	# save matrix once it has converged
        	print("saving..")
        	self.save_q_matrix()
        r = rospy.Rate(1)
        r.sleep()
        
        #print(self.reward)
        

    # execute our q-learning algorithm until our matrix has converged
    # need to re-evaluate how we are handling rewards and convergence
    def run(self):
        self.next_action = random.randrange(9)
        r = rospy.Rate(1)
        r.sleep()
        db_color = self.actions[self.next_action]['dumbbell']
        block_num = self.actions[self.next_action]['block']
        movement = RobotMoveDBToBlock(robot_db=db_color, block_id=block_num)
        self.db_pub.publish(movement)

        

    def save_q_matrix(self):
        # TODO: You'll want to save your q_matrix to a file once it is done to
        # avoid retraining
        df = pd.DataFrame(self.qmatrix)
        df.to_csv('qmatrix.csv')

        print(df)

if __name__ == "__main__":
    node = QLearning()
    rospy.spin()
    
