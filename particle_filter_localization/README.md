# particle_filter_project

# Particle Filter Localization Implementation Plan

## Team Members
- Jeff Zhong
- Todd Tan

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
