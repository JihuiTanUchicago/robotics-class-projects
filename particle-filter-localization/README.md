# particle_filter_project

## Team Members
- Jeff Zhong
- Todd Tan

## Check Point 2: Final Write-up for the Particle Filtering Project

### Objectives Description
The primary objective of this Checkpoint is to continue with the previous Checkpoint on Particle Cloud Initialization and Update Motion and finish a final deliverable of robot localization using the particle filter algorithm. With the Particles Initialization and Particle Updates based on Robot Movement in place, we continue our work on assigning a weight to particles based on the likelihood field approach, and using the resampling techniques (based on the weights we calculate) to update our estimate of robot's location.

### High-Level Description
Our approach involved the previous checkpoint of recording a map of the Maze for the particle initialization and updating each particle's positional information based on the real robot's movements in real-time. In addition, we assign weights to each particle according to the sensor data similarity, and resampling particles based on the weights to converge towards the most likely robot position. This approach will allow the simulation to continuously refine its estimated position within the environment, using the new data coming from the sensors and robotic movements.

### Code Explanation
- num_particles is set to 1000 given the computational resource constraint on the robot.

- Previously, we have implemented the Initialization of Particle Cloud(`initialize_particle_cloud`), Movement Model(`update_particles_with_motion_model`) methods in Checkpoint1. These would be the base methods that we use to continue our work in this checkpoint. However, we modified the movement mode update mechanism by excluding points that are out of the map after the movement update. We found that this help the robot consider more correct points for guessing its location.

- Measurement Model(`update_particle_weights_with_measurement_model`) method: In this function, we update the position weights of the particles based on how closely their predicted sensor readings match the actual sensor readings of the robot. This is the key function we use to align the particle cloud with the robot's perceived environment. We first take 8 positional sensor measurements covering all major directions(front, back, left, right, up right, up left, bottom left, bottom right). We calculate how far is the measurement in terms of units of cells within the map instead of expressing the values as meters. For each particle, we determine its x, y, and yaw coordinates, and calculate the corresponding cells within the map that the particle + the sensor measurement have reached. Probabilities on these positions are multiplied altogether, and that will become the new weight of the particle. 

- Helper Function (`get_probability_map`) Transform the occupancy grid data from the robot's map into a probability map that could be used by the measurement model to calculate particle weights: In this function, we convert the occupancy's grid into a grid where each cell represents the likelihood of the hindrance at the point based on sensor readings. A BFS algorithm is used to efficiently compute each cell's closest distance to a hindrance, and a 0-centered gaussian distribution with std = 0.1 is used to compute the corresponding probabilities.
  
- Resampling Method (`resample_particles`) function: After we update the particle weights, we wrote the resampling method function that resamples the particle cloud, favoring the particles with higher weights. Basically, if the sum of weights is 1, and a given particle's weight is 0.13, then the likelihood of it being drawn each time is 13%. This method focuses on the particle cloud around areas of higher probability, narrowing down the likely position of the robot.

- Updating Estimated Robot Pose (`update_estimated_robot_pose` function): In this function, we computed the average of the particle positions and orientations to estimate the robot's current pose. The estimate will keep improving as the particle cloud converges to the true location of the robots.

### Video
![IMG_1985](https://github.com/Intro-Robotics-UChicago-Spring-2024/particle-filter-project-todd_jeff/assets/91858809/0f031afb-8593-44f8-86c8-b53e4b23bf8a)

Note that we were standing on the upper side of the map, so you have to flip the map on the computer to match the actual robot's location. Our code did fairly well in predicting the robot's exact location. For the X and Y coordinates, the prediction is perfect; it is completely correct and couldn't be more precise. However, for the yaw value, the actual orientation of the robot vs. what was shown on the map was slightly mismatched. This might be due to the delay of the update(i.e. the update is always using an older value than the robot's current state due to computation limitation)

### Challenges
- **Computational Load**: After we wrote the measurement model and resampling method, we encounted the `tf2.TransformException` Error, which as Sarah and Timmy said, is an error that the code is doing too much computation within each cycle of the particle filter algorithm so the program couldn't finish the computation before it was asked to do the next computation. This indeed poses a challenge because the likelihood method requires us to pre-compute the likelihodd within the occupancy grid area (384 * 384) before we get the sensor data and do the resampling etc. We optimized our code by adjuting the number of particles and resampling strategy.

- **Out of Bound particles or Robot Pose**: Another challenge we encountered during the testing is that when we first tested out our code, as the robot progresses its movement, the particles has driven out of the bound and eventually led the robot pose to an area that is unreachable. We solved this issue by setting the bound on both particles and the robot pose.

- **Getting the workable parameters to the functions**: As like other robotics experiments/functions, we spent a lot of time on iterating the parameters of the functions to get the robotics to work in real-world physical environment. For example, one of the parameters we have to test is the standard devitaion of the gaussian function (`compute_prob_zero_centered_gaussian`)

### Future Work
- **Trying out the Ray casting Approach**: In this project, we only have enough time to write and test the likelihood field approach. However, if we are provided more time, we would like to try the ray casting approach and compare the performance results etc.

- **Optimized Performance**: Our current method `get_probability_map` is computational expensive (we have to generate a 2D Grip of width and height of 384.) If for larger maps, this computations could be very expensive and potentially hinder the robot's performance, and the memory usage could be a concern. So we might consider using optimization strategies such as caching the map or reducing the resolution.

- **Other advanced resampling techniques**: We currently used the basic version of resampling method for our test, if we have more time, we could explore more advanced resampling methods such as `Low Variance Resampling` or `Adapative Resampling`

### Takeaways
- **Iterative Testing**: Like other robotics homework and checkpoint, iterative testing could be crucial in the robotics development process. At the beginning, before implementing the `update_particle_weights_with_measurement_model` function, we made sure we address other functions/feature before we added complexity (as the `update_particle_weights_with_measurement_model` is the most difficult to implement). We made sure we tested out each part of the particle filter is performed as expected (using print and debugging messages) before we finally incorporated this function into our code.  This approach not only simplified the debugging but also helped us in understanding the impact of each component to the system overall.

- **Team work and Collaboration**: Clear communication and divisions of tasks was the cornerstone to this project's success. We made sure the tasks was divided in a way that could leverage each member's strength. In addition, Git's Version Control has been extremely helpful in coordinating our efforts to the project. We used the `test` branch when we are testing out different versions of the program and make sure it works before we merged back to the `main` branch.


## Check Point 1: Particle_Cloud Initialization and Updating Motions on Rviz

### Objectives Description
The primary objective of this Checkpoint is to develop an algorithm that is capable of localize robot within a known environment, provided with the sensor data. We leverage probablistic models to estimate the robot's position and orientation, thereby increasing the accuracy of the robot's awareness of its movement in real-time.

### High-Level Description
Recording a map of the environment that the robot is in and attempting to initialize the first batch of particles to start the localization process. Also updating each particle's positional information based on the real robot's movements in real-time.

### Code Explanation
- Initialization of Particle Cloud(`initialize_particle_cloud`) method: This is located in `particle_filter.py` script, from line 138-170. In this function, we scatters a predefined number of particles across the navigatable area (The area that have light grey cells on the map as opposed to black obstacles/walls or areas outside of the map). Each particle is assigned a random position within the navigable space and random orientation, as part of the robot's initial state.

- Movement Model(`update_particles_with_motion_model`) method: This is located in `particle_filter.py` script, from line 303-327. In this function, we update each particle's state based on the robot's observed movements. It shifts the particles by the precomputed difference in robot's position and orientation (yaw), with added Gaussian Noise (`self.linear_noise` and `self.rotation_noise`) to simulate the real-world uncertainty. This function demonstrates how to update the particle state based on the robot's motion and real-world scenarios.

- Incorporation of Noise: Noise is integrated within the `update_particles_with_motion_model` function, using `np.random.normal` to generate the Gaussian noise based on standard deviation. This noise is applied to the differences of the position and orientation changes, introducing simulation that mimics real-world imprecision.

### Video
![IMG_1946](https://github.com/Intro-Robotics-UChicago-Spring-2024/particle-filter-project-todd_jeff/assets/91858809/2bece9d1-5b95-4ce3-a5de-0fa04102a764)

### Challenges
1. Initializing Particles on Map: one of the challenges we encounter is that when we lower the number of initial particles (lower than 5000), the graph did not initialize successfully. Later we incorporate the rospy.sleep(1) command into the initialization code (according to the instruction provided), so enough time were provided to allow ROS node as well as its publishers and subscribers to set up, solving this problem.

2. Updating and integrating sensor data with Gaussian noise to simulate environmental uncertainties. We have to test and fine-tune the standard deviation values for the noise iteratively to get an efficient parameters for managing the particle movement.

3. Getting the accurate Mapping was importance of the particle initialization: Initialize when we first record the map, we didn't put it into the Maze before we start making a map of the maze environment. During the code testing phrase we realize the importance of the map and we re-mapped the maze to make sure the particles are intialized correctly within the navigable areas.

### Future Work
With additional time, potential further improvements could be used to constraint the initial particles within the constraints of the maze. We thought about implementing that but according to the TA we wouldn't need to prevent particles from moving out of the boundary as the particles will eventually be resampled out later in the project. Other potential improvement could include dynamically adjusting the number of the particles we initialize based on the required precision or computational resources.

### Takeaways
- **Understanding Probablistic Models**: Through this project we understand the importance of how the probablistic model in robotics could do for handling real-world uncertainties. Thus, it is important to understand how the mathematical models could be useful to the robot localization application.
- **Iterative Development and Testing**: Iterative nature of testing the initialization number, as well as noise parameters tuning and accuracy mapping of the Maze, highlighted the importance of continouous testing in robotics development.

# Particle Filter Localization Implementation Plan

## Implementation Details

### Initialize Particle Cloud (`initialize_particle_cloud`)
- **Description:** Initialize the particle cloud by distributing 100 points \([x_i, y_i]\) with a uniform distribution throughout the free space of the map and assigning a uniformly distributed orientation \(0 ≤ θ < 2π\) to each point. The exact number of initial points needs further testing.
- **Testing:** Visualize the particle distribution in RViz.

### Update Particles with Motion Model (`update_particles_with_motion_model`)
- **Description:** Assuming translational and rotational noises follow a Gaussian distribution, update each particle’s \([x, y, θ]\) by adding the corresponding motion change plus Gaussian noise. Appropriate values for translational or rotational noise will be determined through testing.
- **Testing:** Control robot movements and observe particle movements in RViz to ensure they follow similar trajectories as the robot but with slight variation due to noise.

### Compute Importance Weights with Measurement Model (`update_particle_weights_with_measurement_model`)
- **Description:** Calculate theoretical sensor data for each particle given its position and the map, and compare it with actual sensor data. Compute weights using the formula from class exercises or a better alternative if found.
- **Testing:** Verify that particles closer to the robot’s actual position receive higher weights.

### Normalize and Resample Particles (`normalize_particles`, `resample_particles`)
- **Description:** Normalize weights \([w_1, …, w_k]\) using `normalized_w_i = w_i / sum([w_1,...,w_k])`. Use these normalized weights as probabilities to resample a new set of 100 particles from the previous set.
- **Testing:** Check if particles converge towards the robot’s actual location over time, as reflected in the distribution of resampling inspected in RViz.

### Update Estimated Robot Pose (`update_estimated_robot_pose`)
- **Description:** Update the robot’s estimated pose by calculating the weighted average of all particles' positions and directions.
- **Testing:** Compare the estimated pose of the robot with the true pose to measure pose estimation accuracy.

### Incorporate Noise into Particle Filter Localization
- **Description:** Gaussian Noise will be incorporated into the movement model.
- **Testing:** Test and iterate the noise distribution to ensure the spread of particles allows the robot to handle real-world scenarios.

## Timeline

- **Friday, April 5th, 5:00 pm CST**
  - Intermediate Deliverable: Particle Cloud Initialization & Movement

- **Friday, April 8th**
  - Complete Measurement Model, Normalize/Resample Model, and update estimated robot pose code

- **Friday, April 10th**
  - Extensively test the model to ensure it can handle real-world scenarios. Begin Writeup after testing.

- **Friday, April 12th, 5:00 pm CST**
  - Final Deadline: Submit Code, Writeup, gif/video, rosbag, and Partner Contributions Survey
