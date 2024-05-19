# q_learning_project
Names: Todd Tan, Jake Brown

## Q-learning Algorithm Implementation

### Objective Description
This intermediate deliverable is to train a Q-matrix for later programming so that the robot knows what the expected result is. The goal is also to ensure that a Q-matrix is stored within a CSV so that the real robot can know which actions lead to the best reward. The CSV contains a converged Q-matrix which allows for the following two goals of the immediate deliverable to be realistic.

### High-level Description
The robot will be able to know which color object belongs to which AR tag by referring to the Q-matrix we developed. When the robot starts at row = 0, it will refer to the maximum value within the given row, and the corresponding action within the `action_matrix` would be able to tell the robot which specific color object should go to which AR tag. The robot is only rewarded when it has successfully placed the correct colored item at the correct tag. At that, the correct tag is fed to the training algorithm through an assigned dictionary and if the tag has an item the robot placed in front of it that is not the correct item, it will not give reward. The only way for the robot to recieve reward is to successfully place all objects in front of the appropriate tags. Order does not matter as long as the correct items are in front of the correct tags given by the dictionary of items and tags. 

### Q-learning algorithm description
- Selecting and Executing actions for the robot: we parsed the `action_matrix` and `actions` into numpy arrays, and from the `action_matrix`, we created a dictionary to store available actions given the current state and next state that are not negative ones. So the dictionary looks like: dict[current_state][next_state] = action_index
- Updating the Q-matrix:
1) we initialize the Q-matrix so that it is the same shape as the `action_matrix` and the values to 0s.
2) For every iteration, the robot is always starting at the origin(row 0). Then, we randomly draw an available action from the dictionary we constructed: `dict[current_state][next_state] = action_index` to get the `current_state`, `next_state`, and the selected `action_index`.
3) Then, we perform the action in the `perform_action` function within the `q_data.py`. We publish the action and wait for 0.5 seconds, and our callback function `receive_reward` in the reward topic's subscriber should be triggered. The `receive_reward` function would then update the `self.reward_amount` based on the reward amount it gets.
4) Then, using the formula `Q(s_t, a_t) + alpha * (r_t + gamma * max_a Q(s_{t+1}, a) - Q(s_t, a_t))`, we update the corresponding values within the Q-matrix in the function `update_q_matrix` with a learning rate = 1 and discount factor = 0.8
5) We repeat the above process until the maximum number of iterations hit.
- Determining when to stop: we choose a relatively large number to represent the maximum iterations(200) the algorithm could go. Since the q-matrix is small(64x64), there shouldn't be too many iterations. Technically, we could set the maximum number of iterations to 1000 just to be safe.
  
## Implementation Plan
### Q-learning Algorithm
The robot is going to start at the origin with the script `virtual_reset_world.py`. We iteratively run the following algorithm until the Q-matrix converges. When each value in the updated Q-matrix only has updated less than 0.001 compared to the prior matrix, then we say that the Q-matrix has converged. Here is the algorithm:
0) Initialize a Q-matrix with the same shape as the action_matrix. Initialize the values to 0s.
1) Start at row 0 in the action_matrix
2) Randomly select a column index that contains a non-negative value.
3) From the column index and the corresponding value, we could determine the next state and the action the robot is going to take.
4) We determine the rewards based on the current object and the new state. We only give a reward of 100 if the current object is moved to the corresponding tag in the new state. Based on the reward value, we update the previous state's reward using the formula `Q(s_t, a_t) + alpha * (r_t + gamma * max_a Q(s_{t+1}, a) - Q(s_t, a_t))`.
5) Then starting at the new state, we repeat step (2) to (4) until the robot enters into a state with all -1 values.

### Robot Perception
#### Three Color Objects and Their Locations
We will use a mix of `/scan` and `/camera/rgb/image_raw` to determine the location of each object. We will leverage `/camera/rgb/image_raw` to determine which direction the robot should go to reach the objects based on the sensed colors. We will use `/scan` to determine whether the robot is close enough to a desired object. We will use the `cvtColor` in addition to `\scan` to help us determine which color object we are targeting.

#### Three AR Tags
Using aruco from openCV we will have to identify which AR tag we are looking at by immediately assigning a tag value to it and putting it in an aruco dictionary. We will use `/scan` to determine if the robot is close enough to the AR-tag to drop or pick an object.

### Robot Manipulation & Movement

#### Arm Manipulation
We will leverage code from `Line_Follower.py` from Lab B to use the OpenCV, as well as leveraging `/scan` and `/camera/rbg/image_raw` to help us precisely control the orientation and position of the arm in order to grab the object. We will also test angles beforehand to find the best values when grabbing an object.

#### Robot Movement
See `Robot Perception` section on how we leverage `/scan` and `/camera/rgb/image_raw` for sensing the objects and AR tags and moving towards them.

### Timeline
- Friday: Finished Q-matrix with the `training.launch` file
- Sunday: The robot should be able to follow according to the Q-matrix and reach the right location(AR Tags and objects).
- Wednesday: The robot should be able to grab the objects.
- Friday: fine-tuning before final submission.
