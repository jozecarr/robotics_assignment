from geometry_msgs.msg import Pose, PoseArray, Quaternion, Point
from . pf_base import PFLocaliserBase
import math
import rospy
from . util import rotateQuaternion, getHeading
import random
import numpy as np
from time import time
import tf.transformations as transformations
from .sensor_model import SensorModel
 
 
class PFLocaliser(PFLocaliserBase):
       
    def __init__(self):
        # ----- Call the superclass constructor
        super(PFLocaliser, self).__init__()
        
        # ----- Set motion model parameters
        self.latest_odom_x = 0
        self.latest_odom_y = 0
        self.latest_odom_heading = 0
        
        # ----- Sensor model parameters
        self.NUMBER_PREDICTED_READINGS = 20     # Number of readings to predict
        self.sensor_model = SensorModel()
        
    #    self.estimatedpose = PoseWithCovarianceStamped()
    #    self.estimatedpose.pose = PoseWithCovariance()
        self.NUM_PARTICLES = 300
        self.resample_threshold = 0.69 * self.NUM_PARTICLES
        self.Random_Particles_Ratio = 0.01
        self.particlecloud = PoseArray()
        
    def initialise_particle_cloud(self, initialpose):
        """
        Set particle cloud to initialpose plus noise
 
        Called whenever an initialpose message is received (to change the
        starting location of the robot), or a new occupancy_map is received.
        self.particlecloud can be initialised here. Initial pose of the robot
        is also set here.
        
        :Args:
            | initialpose: the initial pose estimate
        :Return:
            | (geometry_msgs.msg.PoseArray) poses of the particles
        """
 
        particle_cloud = PoseArray()
        
        for _ in range(self.NUM_PARTICLES):
            distribution_x = random.gauss(initialpose.pose.pose.position.x, 1)
            distribution_y = random.gauss(initialpose.pose.pose.position.y, 1)
            
            pose = Pose()
            pose.position.x = distribution_x
            pose.position.y = distribution_y
            
            spinnn = random.gauss(0, math.pi)
            
            quaternion_message = Quaternion(distribution_x, distribution_y, 0, 0)
            pose.orientation = rotateQuaternion(quaternion_message, spinnn)
            
            particle_cloud.poses.append(pose)
            
        self.particlecloud = particle_cloud
        return particle_cloud
    
    
    def update_particle_cloud(self, scan):
        
        if all([
            self.prev_odom_x == self.latest_odom_x,
            self.prev_odom_y == self.latest_odom_y,
            self.prev_odom_heading == self.latest_odom_heading
        ]):
            
            return
        self.latest_odom_x, self.latest_odom_y, self.latest_odom_heading  = self.prev_odom_x, self.prev_odom_y, self.prev_odom_heading
        
        particle = self.particlecloud
 
        weights = []
 
        for i, particle in enumerate(particle.poses):
            weight = self.sensor_model.get_weight(scan, particle)  
            weights.append(weight)  
 
        total = sum(weights)  
 
        normalised_weights = [w / total for w in weights]
 
        N = self.NUM_PARTICLES
 
        cumulative_distribution = []
        for i, n in enumerate(normalised_weights):
            if i == 0:
                cumulative_distribution.append(n)  
            else:
                cumulative_distribution.append(cumulative_distribution[-1] + n)  

        
        step = 1 / N  
        u = random.uniform(0, step)  
        i = 0  # for the while loop
        updated_particles = PoseArray()  

        
        for _ in range(N):
            while u > cumulative_distribution[i]:
                i = i + 1
 
            particles = self.particlecloud.poses[i]
 
            position_x = particles.position.x
            position_y = particles.position.y
            orientation = particles.orientation
            heading = getHeading(orientation)  
           
            random_x = random.gauss(position_x, normalised_weights[i])
            random_y = random.gauss(position_y, normalised_weights[i])
            random_orientation = random.gauss(heading, normalised_weights[i])
 
            random_Point = Point(random_x, random_y, 0.0)  
            rotateQ = rotateQuaternion(orientation, random_orientation - heading)
            new_pose = Pose(random_Point, rotateQ)
 
            u = u + step
 
            updated_particles.poses.append(new_pose)
 
        self.particlecloud = updated_particles
        return updated_particles
        
    pass       

 
    def estimate_pose(self):
        """
        This should calculate and return an updated robot pose estimate based
        on the particle cloud (self.particlecloud).
        
        Create new estimated pose, given particle cloud
        E.g. just average the location and orientation values of each of
        the particles and return this.
        
        Better approximations could be made by doing some simple clustering,
        e.g. taking the average location of half the particles after
        throwing away any which are outliers
 
        :Return:
            | (geometry_msgs.msg.Pose) robot's estimated pose.
         """
        
        avg_x = sum(particle.position.x for particle in self.particlecloud.poses) / len(self.particlecloud.poses)
        avg_y = sum(particle.position.y for particle in self.particlecloud.poses) / len(self.particlecloud.poses)
        avg_orientation = sum(getHeading(particle.orientation) for particle in self.particlecloud.poses) / len(self.particlecloud.poses)
        
        estimated_pose = Pose()
        estimated_pose.position.x += avg_x
        estimated_pose.position.y = avg_y  
        estimated_pose.orientation = rotateQuaternion(Quaternion(w=10), avg_orientation)
        
        return estimated_pose
    
    pass
