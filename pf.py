#!/usr/bin/python
from geometry_msgs.msg import Pose, PoseArray, Quaternion
from nav_msgs.msg import Odometry
from pf_base import PFLocaliserBase
import math
import rospy
from util import rotateQuaternion, getHeading
from random import random
from time import time
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import random
from random import gauss, vonmisesvariate
from numpy import random
import numpy as np
import copy
import tf

# Batuhan Ozgur Basal
# P2681345
# Intelligent Mobile Robotics Assignment Task 2 

class PFLocaliser(PFLocaliserBase):
       
    def __init__(self):
        # Call the superclass constructor
        super(PFLocaliser, self).__init__()
        
        # Set motion model parameters
        self.ODOM_ROTATION_NOISE = 0.02 # Odometry model rotation noise
        self.ODOM_TRANSLATION_NOISE = 0.02 # Odometry x axis (forward) noise
        self.ODOM_DRIFT_NOISE = 0.02 # Odometry y axis (side-side) noise
 
        # Sensor model parameters
        self.NUMBER_PREDICTED_READINGS = 50 	# Number of readings to predict
        
        # Set motion model paramaters to add noise while resampling
        self.UPDATED_NOISE = np.random.uniform(100, 120) 
        self.UPDATED_ANGULAR_NOISE = np.random.uniform(1, 120) 
    
    
    def obtained_value(self, prob_array):
        
        pose_arrays = PoseArray() 
        for each_pose in range(len(self.particlecloud.poses)):
            total_value = random.random() * sum(prob_array)
            total_weight_probability = 0 
            indicator = 0
            while total_weight_probability < total_value:
	            total_weight_probability += prob_array[indicator] 
	            indicator = indicator + 1 
            pose_arrays.poses.append(copy.deepcopy(self.particlecloud.poses[indicator - 1])) 
        return pose_arrays 
        
    
    def updated_noise(self, pose_object):
        
        if self.UPDATED_NOISE > 1.0:
            self.UPDATED_NOISE -= 0.02
        else:
            self.UPDATED_NOISE = np.random.uniform(0.02, 1) 
        
        self.ODOM_ROTATION_NOISE = np.random.uniform(0.02, 0.1)
        self.ODOM_TRANSLATION_NOISE = np.random.uniform(0.02, 0.1)
        self.ODOM_DRIFT_NOISE = np.random.uniform(0.02, 0.1)

        pose_object.position.x += gauss(0, self.UPDATED_NOISE) * self.ODOM_TRANSLATION_NOISE   
        pose_object.position.y += gauss(0, self.UPDATED_NOISE) * self.ODOM_DRIFT_NOISE   
        pose_object.orientation = rotateQuaternion(pose_object.orientation,(vonmisesvariate(0, self.UPDATED_ANGULAR_NOISE) - math.pi) * self.ODOM_ROTATION_NOISE) 
        return pose_object
        
        
    def initialise_particle_cloud(self, initialpose):

        pose_arrays = PoseArray()
        i = 0
        while i < 500:
            random_gauss_number = gauss(0, 1)
            rotational_dist = vonmisesvariate(0, 5)
            pose_objects = Pose()
            pose_objects.position.x = initialpose.pose.pose.position.x + (random_gauss_number * self.ODOM_TRANSLATION_NOISE)
            pose_objects.position.y = initialpose.pose.pose.position.y + (random_gauss_number * self.ODOM_DRIFT_NOISE)
            pose_objects.orientation = rotateQuaternion(initialpose.pose.pose.orientation,((rotational_dist - math.pi) * self.ODOM_ROTATION_NOISE))
            pose_arrays.poses.append(pose_objects)
            i += 1
        return pose_arrays


    def update_particle_cloud(self, scan):
        
        prob_of_weight = []
        
        for pose_object in self.particlecloud.poses:
            prob_of_weight.append(self.sensor_model.get_weight(scan, pose_object))

        obtained_pose_arrays = self.obtained_value(prob_of_weight)
                
        for pose_object in obtained_pose_arrays.poses:
            pose_object = self.updated_noise(pose_object)
         
        self.particlecloud = obtained_pose_arrays
 
    
    def estimate_pose(self):
        predicted_pose = Pose()
        x_value = 0
        y_value = 0
        z_value = 0  
        w_value = 0  
        
        for each_object in self.particlecloud.poses:
            x_value += each_object.position.x 
            y_value += each_object.position.y 
            z_value += each_object.orientation.z 
            w_value += each_object.orientation.w 

        predicted_pose.position.x = x_value / 500
        predicted_pose.position.y = y_value / 500
        predicted_pose.orientation.z = z_value / 500
        predicted_pose.orientation.w = w_value / 500

        #rpy = tf.transformations.euler_from_quaternion(predicted_pose.position.x , predicted_pose.position.y, predicted_pose.orientation.z, predicted_pose.orientation.w)
        orientation_list = [predicted_pose.position.x, predicted_pose.position.y, predicted_pose.orientation.z, predicted_pose.orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        
        print('Robots X position: ', predicted_pose.position.x)
        print('Robots Y position: ', predicted_pose.position.y)
        print('Robots Heading: ', math.degrees(yaw))
        return predicted_pose
