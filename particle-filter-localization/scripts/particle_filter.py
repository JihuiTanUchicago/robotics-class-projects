#!/usr/bin/env python3

import rospy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Quaternion, Point, Pose, PoseArray, PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header, String

import tf
from tf import TransformListener
from tf import TransformBroadcaster
from tf.transformations import quaternion_from_euler, euler_from_quaternion

import numpy as np
from numpy.random import random_sample
import math

from random import randint, random, uniform
from collections import deque

from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

def compute_prob_zero_centered_gaussian(dist, sd):
    """ Takes in distance from zero (dist) and standard deviation (sd) for gaussian
        and returns probability (likelihood) of observation """
    c = 1.0 / (sd * math.sqrt(2 * math.pi))
    prob = c * math.exp((-math.pow(dist,2))/(2 * math.pow(sd, 2)))
    return prob
    

def get_yaw_from_pose(p):
    """ A helper function that takes in a Pose object (geometry_msgs) and returns yaw"""

    yaw = (euler_from_quaternion([
            p.orientation.x,
            p.orientation.y,
            p.orientation.z,
            p.orientation.w])
            [2])

    return yaw



class Particle:

    def __init__(self, pose, w):

        # particle pose (Pose object from geometry_msgs)
        self.pose = pose

        # particle weight
        self.w = w



class ParticleFilter:


    def __init__(self):

        # once everything is setup initialized will be set to true
        self.initialized = False


        # initialize this particle filter node
        rospy.init_node('turtlebot3_particle_filter')

        # set the topic names and frame names
        self.base_frame = "base_footprint"
        self.map_topic = "map"
        self.odom_frame = "odom"
        self.scan_topic = "scan"

        # inialize our map
        self.map = OccupancyGrid()

        # the number of particles used in the particle filter
        self.num_particles = 5000

        # initialize the particle cloud array
        self.particle_cloud = []

        # initialize the estimated robot pose
        self.robot_estimate = Pose()

        # set threshold values for linear and angular movement before we preform an update
        self.lin_mvmt_threshold = 0.2
        self.ang_mvmt_threshold = (np.pi / 6)

        self.odom_pose_last_motion_update = None


        # Setup publishers and subscribers

        # publish the current particle cloud
        self.particles_pub = rospy.Publisher("particle_cloud", PoseArray, queue_size=50)

        # publish the estimated robot pose
        self.robot_estimate_pub = rospy.Publisher("estimated_robot_pose", PoseStamped, queue_size=10)

        # subscribe to the map server
        rospy.Subscriber(self.map_topic, OccupancyGrid, self.get_map)
        rospy.sleep(1)

        # subscribe to the lidar scan from the robot
        rospy.Subscriber(self.scan_topic, LaserScan, self.robot_scan_received)

        # enable listening for and broadcasting corodinate transforms
        self.tf_listener = TransformListener()
        self.tf_broadcaster = TransformBroadcaster()

        # gaussian noises for motion
        self.linear_noise = 0.1 # in meters
        self.rotation_noise = 0.01 # in radian


        # intialize the particle cloud
        self.initialize_particle_cloud()

        self.initialized = True



    def get_map(self, data):
        self.map = data
        self.pmap = self.get_probability_map()
        print(self.pmap)


    def initialize_particle_cloud(self):
        width = self.map.info.width
        height = self.map.info.height
        resolution = self.map.info.resolution
        origin = self.map.info.origin


        # Initialize an empty list to hold particles
        self.particle_cloud = []

        for _ in range(self.num_particles):
            while True:
                # Randomly select a point within the map grid
                x_map = randint(0, width - 1)
                y_map = randint(0, height - 1)

                # Check if the selected point is in a navigable space
                if self.map.data[y_map * width + x_map] == 0:
                    # Convert map grid coordinates to world coordinates
                    x_world = (x_map * resolution) + origin.position.x
                    y_world = (y_map * resolution) + origin.position.y

                    # Random orientation
                    theta = random() * 2 * math.pi

                    # Create pose and particle, then add to the cloud
                    pose = Pose(Point(x_world, y_world, 0), Quaternion(*quaternion_from_euler(0, 0, theta)))
                    self.particle_cloud.append(Particle(pose, 1.0))

                    break

        self.normalize_particles()
        self.publish_particle_cloud()


    def normalize_particles(self):
        # make all the particle weights sum to 1.0
        sum_weights = 0
        
        for particle in self.particle_cloud:
            sum_weights += particle.w
        print(sum_weights)
        for i in range(len(self.particle_cloud)):
            self.particle_cloud[i].w = self.particle_cloud[i].w / sum_weights
        print(len(self.particle_cloud))

    def publish_particle_cloud(self):

        particle_cloud_pose_array = PoseArray()
        particle_cloud_pose_array.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)
        particle_cloud_pose_array.poses

        for part in self.particle_cloud:
            particle_cloud_pose_array.poses.append(part.pose)

        self.particles_pub.publish(particle_cloud_pose_array)




    def publish_estimated_robot_pose(self):

        robot_pose_estimate_stamped = PoseStamped()
        robot_pose_estimate_stamped.pose = self.robot_estimate
        robot_pose_estimate_stamped.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)
        self.robot_estimate_pub.publish(robot_pose_estimate_stamped)



    def resample_particles(self):
        # Normalize particle weights
        self.normalize_particles()
        
        # Get the current weights
        weights = np.array([particle.w for particle in self.particle_cloud])
        print(f"weights: {weights}")  
        # Compute the cumulative sum of the weights
        cumulative_weights = np.cumsum(weights)
        print(f"cumulative_weights: {cumulative_weights}")
        # Generate random thresholds to pick new particles
        thresholds = np.random.uniform(0, cumulative_weights[-1], self.num_particles)
        
        # Get the indices of the first occurrence where the cumulative weight exceeds the threshold
        indices = np.searchsorted(cumulative_weights, thresholds)
        
        # Use these indices to create the resampled particle list
        resampled_particles = [self.particle_cloud[index] for index in indices]
        
        # Replace the old particles with the new resampled particles
        self.particle_cloud = resampled_particles
        
        # After resampling, reset all weights to be uniform
        uniform_weight = 1.0 / len(self.particle_cloud)
        for i in range(len(self.particle_cloud)):
            self.particle_cloud[i].w = uniform_weight

        print("resample_partcles: completed")



    def robot_scan_received(self, data):

        # wait until initialization is complete
        if not(self.initialized):
            return

        # we need to be able to transfrom the laser frame to the base frame
        if not(self.tf_listener.canTransform(self.base_frame, data.header.frame_id, data.header.stamp)):
            return

        # wait for a little bit for the transform to become avaliable (in case the scan arrives
        # a little bit before the odom to base_footprint transform was updated)
        self.tf_listener.waitForTransform(self.base_frame, self.odom_frame, data.header.stamp, rospy.Duration(0.5))
        if not(self.tf_listener.canTransform(self.base_frame, data.header.frame_id, data.header.stamp)):
            return

        # calculate the pose of the laser distance sensor
        p = PoseStamped(
            header=Header(stamp=rospy.Time(0),
                          frame_id=data.header.frame_id))

        self.laser_pose = self.tf_listener.transformPose(self.base_frame, p)

        # determine where the robot thinks it is based on its odometry
        p = PoseStamped(
            header=Header(stamp=data.header.stamp,
                          frame_id=self.base_frame),
            pose=Pose())

        self.odom_pose = self.tf_listener.transformPose(self.odom_frame, p)

        # we need to be able to compare the current odom pose to the prior odom pose
        # if there isn't a prior odom pose, set the odom_pose variable to the current pose
        if not self.odom_pose_last_motion_update:
            self.odom_pose_last_motion_update = self.odom_pose
            return


        if self.particle_cloud:

            # check to see if we've moved far enough to perform an update

            curr_x = self.odom_pose.pose.position.x
            old_x = self.odom_pose_last_motion_update.pose.position.x
            curr_y = self.odom_pose.pose.position.y
            old_y = self.odom_pose_last_motion_update.pose.position.y
            curr_yaw = get_yaw_from_pose(self.odom_pose.pose)
            old_yaw = get_yaw_from_pose(self.odom_pose_last_motion_update.pose)

            if (np.abs(curr_x - old_x) > self.lin_mvmt_threshold or
                np.abs(curr_y - old_y) > self.lin_mvmt_threshold or
                np.abs(curr_yaw - old_yaw) > self.ang_mvmt_threshold):

                # store dealta changes in positional values, adding gaussian noises
                pos_delta_dict = {
                    "del_x": curr_x - old_x + np.random.normal(0, self.linear_noise),
                    "del_y": curr_y - old_y + np.random.normal(0, self.linear_noise),
                    "del_yaw": curr_yaw - old_yaw + np.random.normal(0, self.rotation_noise)
                }

                # This is where the main logic of the particle filter is carried out
                self.update_particles_with_motion_model(pos_delta_dict)

                self.update_particle_weights_with_measurement_model(data)

                self.normalize_particles()

                self.resample_particles()

                self.update_estimated_robot_pose()

                self.publish_particle_cloud()
                self.publish_estimated_robot_pose()

                self.odom_pose_last_motion_update = self.odom_pose



    def update_estimated_robot_pose(self):
        print("update_estimated_robot_post: begin")
        # Initialize sum variables for the weighted mean calculation
        x_sum = 0.0
        y_sum = 0.0
        orientation_list = []
        

        for particle in self.particle_cloud:
            # Accumulate the sum of positions
            x_sum += particle.pose.position.x
            y_sum += particle.pose.position.y

            # For orientation, we need to handle the wrap-around at 2*pi
            # Convert quaternion to yaw angle and add it to the list
            _, _, yaw = euler_from_quaternion([
                particle.pose.orientation.x,
                particle.pose.orientation.y,
                particle.pose.orientation.z,
                particle.pose.orientation.w
            ])
            orientation_list.append(yaw)

        # Compute the mean position
        x_mean = x_sum / self.num_particles
        y_mean = y_sum / self.num_particles
        
        # Compute the mean orientation using circular mean
        sin_sum = sum(math.sin(angle) for angle in orientation_list)
        cos_sum = sum(math.cos(angle) for angle in orientation_list)
        yaw_mean = math.atan2(sin_sum, cos_sum)
        
        # Update the robot's estimated pose
        self.robot_estimate.position.x = x_mean
        self.robot_estimate.position.y = y_mean
        quaternion = quaternion_from_euler(0, 0, yaw_mean)
        self.robot_estimate.orientation = Quaternion(*quaternion)

        print("update_estimated_robot_pose: completed")


    def get_probability_map(self):
        width = self.map.info.width
        height = self.map.info.height
        
        #convert 1D to 2D array
        grid = np.array(self.map.data)
        grid = grid.astype(np.float64)
        grid = grid.reshape(height, width)
        
        # Replace 100s with infinity
        grid[grid == 100] = np.inf

        # Directions for moving to adjacent cells
        directions = [(-1,0), (1,0), (0,-1), (0,1)]

        # Queue to hold cells to process
        queue = deque()

        # Initialize the queue with all the free space cells (value 0)
        for i in range(height):
            for j in range(width):
                if grid[i][j] == 0:
                    queue.append((i,j))

        # Process the queue until empty
        while queue:
            x, y = queue.popleft()

            # Explore all adjacent cells
            for dx, dy in directions:
                nx, ny = x + dx, y + dy

                # Check if the new positions is within bounds and not unavailable
                if 0 <= nx < height and 0 <= ny < width and grid[nx][ny] != -1:
                    # Calculate potential new value
                    new_value = grid[x][y] + 1

                    # Update the cell if new value is smaller
                    if new_value < grid[nx][ny]:
                        grid[nx][ny] = new_value
                        queue.append((nx,ny))
        # Treat -1 space as obstacles
        grid[grid == -1] = 0

        # Convert to probability
        for i in range(grid.shape[0]):
            for j in range(grid.shape[1]):
                grid[i][j] = compute_prob_zero_centered_gaussian(grid[i][j], 0.1)
        np.set_printoptions(threshold=np.inf, linewidth=200)
        print(grid)
        return grid
        

    def update_particle_weights_with_measurement_model(self, data):
        print("update_particles_weights_with_measurement_model: Begin")
        resolution = self.map.info.resolution
        origin = self.map.info.origin
        width = self.map.info.width
        height = self.map.info.height
        
        ranges = data.ranges
        fd = ranges[0]
        ur = ranges[45]
        r = ranges[90]
        br = ranges[135]
        b = ranges[180]
        bl = ranges[225]
        l = ranges[270]
        ul = ranges[315]
        directions_data = np.array([fd, ur, r, br, b, bl, l, ul]) / resolution
        print(f"directions_data: {directions_data}")
        for i in range(len(self.particle_cloud)):
            q = 1.0 # initialize the likelihood of this particle

            # Get the orientation of the particle
            quarternion = [
                            self.particle_cloud[i].pose.orientation.x, 
                            self.particle_cloud[i].pose.orientation.y, 
                            self.particle_cloud[i].pose.orientation.z, 
                            self.particle_cloud[i].pose.orientation.w
                          ]
            _,_,yaw = euler_from_quaternion(quarternion)

            for d in range(len(directions_data)):
                # Calculate the coordinates in the map where the laser hits
                rad = yaw + 0.25 * d * math.pi
                hit_x = self.particle_cloud[i].pose.position.x/resolution + directions_data[d] * math.cos(rad)
                hit_y = self.particle_cloud[i].pose.position.y/resolution + directions_data[d] * math.sin(rad)
                print(f"hit x: {hit_x}, hit y: {hit_y}")
                # Convert to map coordinates
                map_x = int(hit_x - origin.position.x/resolution)
                map_y = int(hit_y - origin.position.y/resolution)
                print(f"map_x: {map_x}, map_y: {map_y}")
                if 0 <= map_x < width and 0 <= map_y < height:
                    prob = self.pmap[map_x, map_y]
                    q *= prob
            self.particle_cloud[i].w = q


        print("update_particles_weights_with_measurement_model: Completed")


    def update_particles_with_motion_model(self, pos_delta_dict):
        print(f"update_particles_with_motion_model: begin")
        del_x = pos_delta_dict["del_x"]
        del_y = pos_delta_dict["del_y"]
        del_yaw = pos_delta_dict["del_yaw"]

        for i in range(len(self.particle_cloud)):
            # linear positions update
            curr_x = self.particle_cloud[i].pose.position.x
            curr_y = self.particle_cloud[i].pose.position.y
            if 0 <= curr_x < self.map.info.width:
                self.particle_cloud[i].pose.position.x += del_x
            if 0 <= curr_y < self.map.info.height:
                self.particle_cloud[i].pose.position.y += del_y
            print("updata motion: x: {self.particle_cloud[i].pose.position.x}; y: {self.particle_cloud[i].pose.position.y}")

            # update rotational value(yaw)
            quarternion = [
                            self.particle_cloud[i].pose.orientation.x, 
                            self.particle_cloud[i].pose.orientation.y, 
                            self.particle_cloud[i].pose.orientation.z, 
                            self.particle_cloud[i].pose.orientation.w
                          ]
            _,_,curr_yaw = euler_from_quaternion(quarternion)
            curr_yaw += del_yaw
            quarternion = quaternion_from_euler(0,0,curr_yaw)
            self.particle_cloud[i].pose.orientation.x = quarternion[0]
            self.particle_cloud[i].pose.orientation.y = quarternion[1]
            self.particle_cloud[i].pose.orientation.z = quarternion[2]
            self.particle_cloud[i].pose.orientation.w = quarternion[3]
        print(f"update_particles_with_motion_model: complete")


        



if __name__=="__main__":


    pf = ParticleFilter()

    rospy.spin()
