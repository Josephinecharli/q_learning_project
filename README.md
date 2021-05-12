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
We select a new action in *run()*.

We do this by checking *actions_matrix* and creating a list of all values that are not -1. Then we randomly select one of these actions to be our next action, and publish said action to */q_learning/robot_action*. We also check to make sure that there are possible actions to take, and if not we must be in a state where all 3 dumbbells are placed in front of blocks, so we reset our state to 0 and recalculate our next action.

### Updating Q-Matrix
We update our Q-matrix in *received_reward()*.

We use class variables to keep track of the previously executed action that returned the reward to calculate the Q-matrix formula. We simply subsittute values as necessary and save the new value to the correct position that triggered the reward.

### Determining Convergence
We check for convergence in *received_reward()*.

We determine convergence based on the total change in the matrix between two consecutive successful runs. We store the values for 25 successful attemps, and once they are all 0, we can assume the matrix converges. We also check if iterations is less than 300 as a safety net in the small chance we do not hit all possible successful sequences before we get 25 successful attempts that are not unique. There are 6 possible successful series of actions, and the probability of missing 1 in 25 consecutive repeated series of actions is (5/6)^30 or about 0.4%. We raise to the power of 30 instead of 25 because the 5 instances must have been hit at least once each before the 25 consecutive. This alone gives us a very small chance the matrix is not actually converged. This does not consider the chance that the sequence did not occur in the other 70 sequences. The chance of it not occurring at all in the 70 other attempted sequences is (35/36)^70 or about 14%. Thus our total probability that it did not occur at all is about 0.06%. This seem sufficiently unlikely that we continued with the selected numbers.

### Executing the Correct Path
The robot calls *read_matrix()* which takes the already converged Q-matrix and develops a series of actions to take. It selects an acction by considering the rewards of each possible action in state 0, the starting state. Then it picks the action with the highest reward, preferring the first action found if there are ties. Then it uses this action to determine the next state of the world, and repeats the logic to select a second action. This is appended to a list after the first action. Then it does the same for a third action. It would repeat this process until it finds an action with the maximum possible reward, 100, but due to the nature of the Q-matrix, this will only ever be 3 actions.

Then for each action in the list, we use *action_list* to determine the color of the dumbbell to be moved and the number of the block it should be placed at, and call the respective helper functions to actually execute the movements.

## Robot Perception

### Identifying Dumbbells
Occurs in *received_image()*

We use the robot's *image_raw* to check for colors in the robot's field of view. Then we create a mask of this image for the specific color of dumbbell that we are searching for. If we do not find any pixels of the desired color we spin until we do.

###Identifying Blocks
Occurs in *received_image()*

We use the *image_raw* and *keras_ocr* to look for numbers in the robots field of view. Then we check the list of found strings to see if the number we are searching for is listed. If not we spin slowly until it is.

## Robot Manipulation and Movement

### Moving Towards Dumbbells
Occurs in *pick_dumbbell_by_color()*

We've already calculated the center of the highlighted pixels from the *image_raw* mask and use the x position of this to determine whether or not the robot is facing directly towards the dumbbell. If it is not, we turn until the center of the pixels is approximately at the center of the image. The variance here is covered by the fact the gripper opens wider than the handle of the dumbbell. Then once we are centered, we move directly forward towards the dumbbell, correcting our angle as we get closer. Once the robot's LIDAR reading is small enough that the gripper will be around the handle of the dumbbell, we know we are close enough.

### Picking Up Dumbbells
Occurs in *pick_dumbbell_by_color()*

We use *moveit_commander*s to actuate the robot arm. The starting position is such that the gripper is in front of the robot and parallel to the ground at a height that goes betweeen the bottom and top of the dumbbells. Thus once we are in position, we can simply close the gripper and lift the arm to raise the dumbbell.

### Moving to Blocks
Orientation happens in *turn_to_blocks()*, movement happens in *put_dumbbell_at_num()*

Once *keras_ocr* has found a block with the desired number on it, we calculate the x position of the number in the image. If it is not ceneterd, we rotate until it is. Then we drive forward until *scan* gives us a distance that is close enough to the block to be ready to place the dumbbell.

### Putting Down Dumbbell
Occurs in *put_dumbbell_at_num()*

Once we have moved do the correct block at a good distance, we reset the arm position to the starting one. Because the dumbbells are not completely immobilized by the gripper, they will be effected by gravity enough such that they will be placed on their bottoms. Once the arm is in its original position, it opens the gripper and backs away. The dumbbell should already be touching the ground at this point in a proper orientation, so the release should not let it move drastically.

# Challenges

- *keras_ocr* takes a noticible amount of time to recognize the numbers on the blocks. This means the robot may turn too far when attempting to align with the blocks. We fix this by giviing slow movement commands that limit the distance the robot can rotate before the next scanned image is prepared.
- The gripper does not have any sort of feedbaack. This means if the gripper closes too much, the dumbbell will be squeezed out of its hold. If we had some sort of indicator that the gripper is attempting to use a lot of force used but little movement is being made, we know we can stop gripping tighter. This would give us a better grip on the dumbbell without squeezing it out.
- Balancing between raising the dumbbell high enough to not block the scanner, but not so high that the robot topples or drops the dumbbell. This was mitigated by testing various positions manually. If the grip was better on the dumbbells (ie, something like a circular gripper that went all the way around the handle) it might be possible to use the sensor while the lift is occuring, and raise more until the distance to the closest object in front of the robot jumps up drastically, as the dumbbell is no longer blocking. 

# Future Work

-  Adjust our considerations of the Q-learning convergence such that they are more dynamic and would be consistent regardless of the number of dumbbells and blocks. Keeping with our same strategy of using a certain series of successful rewards, we would likely need to dynamically expand the length of this list based on the number of dumbbell block pairs, as well as incrase the minimum number of cycles based on this number as well. That way the chance of missing a successful sequence is minimal.
- Consider the advantages of different values of alpha. If we were to decrease it, break our convergence checks, since the values could potentially go up even on a repeated call to the same sequence that has already completed, thus we will not know based simply on Q-matrix changes whether or not the current sequence was unique or not. As alpha decreases, the total number of cycles to get a converged matrix will also necesarily increase. 

# Takewayas

- Individual robot actions may be easy to code, but dynamically stringing them together without knowing what the transition will look like ahead of time becomes drastically harder. For example, when the robot is in position to lift the dumbbell, it is trivial to raise it into a safe position. Similarly, it is not difficult to recognize and approach a dumbbell based on its color. But converting from an approach into a position that will properly lift the dumbbell without knowing the robot's orientation to the dumbbell ahead of time requires lots of room for error.
- In situations where the robot's sensors are not capable of capturing its entire surroundings, locating objects becomes massively more difficult. Instead of being able to rely on distance sensors we need to use image processing that is slower and is difficult to give a sense of distance. This makes the robot need to move around to get a good understanding of its environment.

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

