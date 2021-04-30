# q_learning_project

# Team Members:
Josephine Passananti
Evan Casey

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

