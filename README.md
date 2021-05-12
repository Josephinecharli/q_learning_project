# q_learning_project

# Team Members:
Josephine Passananti

Evan Casey

## Objectives
Design a set of programs that create a matrix of actions to take based on a reward and follow these actions and execute them to actually recieve the maximum reward in limited time. To do this, we need to develop a program to create and train a Q-matrix, and a program that reads in a Q-matrix and executes robot actions based on the input.
## High-level description
We implement reinforcement learning by applying the Q-Learning algorithm. We repeatedly tested random possible actions until the robot received a reward. We used an alpha of 1 and a gamma of 0.8 for the function. Alpha of 1 means that each time we repeat an action, we carry down a percentage of the best action to take in the next step without considering the current value of (this is because the formula subtracts alpha*current value). Thus we are able to only carry over values in actions that take us to a successful reward.
## Q-Learning Algorithm
### Selecting Actions

### Updating Q-Matrix

### Determining Convergence
We determine convergence based on the total change in the matrix between two consecutive successful runs. We store the values for 10 successful attemps, and once they are all 0, we can assume the matrix converges. We also check if iterations is less than 300 as a safety net in the small chance we do not hit all possible successful sequences before we get 10 successful attempts that are not unique. There are 3 possible successful first actions, 
### Executing the Correct Path
The robot calls *read_marix()* which takes the already converged Q-matrix and develops a series of actions to take. It selects an acction by considering the rewards of each possible action in state 0, the starting state. Then it picks the action with the highest reward, preferring the first action found if there are ties. Then it uses this action to determine the next state of the world, and repeats the logic to select a second action. This is appended to a list after the first action. Then it does the same for a third action. It would repeat this process until it finds an action with the maximum possible reward, 100, but due to the nature of the Q-matrix, this will only ever be 3 actions.

Then for each action in the list, we use *action_list* to determine the color of the dumbbell to be moved and the number of the block it should be placed at, and call the respective helper functions to actually execute the movements.
## Robot Perception
### Identifying Dumbbells

###Identifying Blocks
## Robot Manipulation and Movement
### Moving Towards Dumbbells

### Picking Up Dumbbells

### Moving to Blocks

### Putting Down Dumbbells

# Challenges

# Future Work

# Takewayas

# Implementation Plan
## Q-learning algorithm
### Executing the Q-learning algorithm
We will create an algorithm based off the one we used in class and code it. We will test on a small number of points, for example the set from class, and verify that it works correctly
### Determining when the Q-matrix has converged
We will set a threshold for the largest change a component makes over one cycle and if all changes are less than this threshold we will consider our matrix to have converged. We will test this by starting with a larger threshold and decreasing it until we reach a reasonable consistency (no more than a few percentage points difference). 
### Once the Q-matrix has converged, how to determine which actions the robot should take to maximize expected reward
Once our matrix has converged, we will move our robot in the direction with the highest reward for each step. Because our Q-matrix has converged, we assume this will lead our robot to the maximum reward in the fewest possible steps. We can test this portion by providing a small already-converged test matrix and ensuring that our robot moves in the desired directions.
## Robot perception
### Determining the identities and locations of the three colored dumbbells
First we will use the /scan topic to locate the dumbells and /camera/rgb/image_raw to identify the colors of the the dumbells. We will base our approach on the code from the class activity, line follower. We will test by visually inspecting our world and ensuring the robots identificsation of the dumbells colors and respective locations are correct.
### Determining the identities and locations of the three numbered blocks
We will use scan again to find the location of the blocks and use the reccomended keras_ocr to identify the numbers of the blocks. We will test this by comparing our visual inspection of the blocks to the robots identification of them.
## Robot manipulation & movement
### Picking up and putting down the dumbbells with the OpenMANIPULATOR arm
We will use the GUIs provided in class to determine joint angles to pick up and move the dumbells with the robot arm. We will test this using visual inspection of whether the robot is able to correctly and consistently pick up and put down dumbells.
### Navigating to the appropriate locations to pick up and put down the dumbbells
We will approach the dumbell until our robot scan[0] is within a certain distance of the robot, and our scans at small angles, for example scan[2] and scan[358] are non-zero values (to make sure our robot is relatively centered in front of the dumbell). Our arm will then move forward to grab the dumbell and in the case where our robot is too close, it will simply push the dumbell back slightly before picking up the dumbell. We will use a similar approach to place the dumbells in front of the blocks. We will use trial and error to modify this approach if necessary. We will test this, again through visual inspection of how consistently the robot is able to pick up the dumbells. We may also write code to make sure this function will execute correctly regardless of the robots relative position to the dumbell. 

# Time Line
We would like to have the Q learning algorithm done by the end of this weekend and we will work on the robot perception next week. We will work on robot manipulation and movement next weekend and hope to have everything done by then so we can spend the following days on debugging and optimization.

