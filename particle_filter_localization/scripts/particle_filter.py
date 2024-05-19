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
        self.num_particles = 1000

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
        
        # Compute the cumulative sum of the weights
        cumulative_weights = np.cumsum(weights)
        
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


    def precompute_map(self, data):

        for row in ranges:
            for column in range(len(ranges[0])):
                data_point = ranges

    def update_particle_weights_with_measurement_model(self, data):
        print("update_particles_weights_with_measurement_model: Begin")
        
        
        # ------------------------------------------------------------
        average_weight = 0.25
        for i in range(len(self.particle_cloud)):
            if i < 4:
                self.particle_cloud[i].w = average_weight
            else:
                self.particle_cloud[i].w = 0.01
        print("update_particles_weights_with_measurement_model: Completed")




    def update_particles_with_motion_model(self, pos_delta_dict):
        print(f"update_particles_with_motion_model: begin")
        del_x = pos_delta_dict["del_x"]
        del_y = pos_delta_dict["del_y"]
        del_yaw = pos_delta_dict["del_yaw"]

        for i in range(len(self.particle_cloud)):
            # linear positions update
            self.particle_cloud[i].pose.position.x += del_x
            self.particle_cloud[i].pose.position.y += del_y
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
