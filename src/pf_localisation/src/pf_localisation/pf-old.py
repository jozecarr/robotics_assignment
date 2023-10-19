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
        self.latest_odom_x, self.latest_odom_y, self.latest_odom_heading = self.prev_odom_x, self.prev_odom_y, self.prev_odom_heading

        num_samples = 100

        particles = []
        for p in self.particlecloud.poses:
            particles.append(p)
            #print(particles)

        weights = []

        for p in self.particlecloud.poses:
            self.weight = self.sensor_model.get_weight(scan, p)

            weights.append(self.weight)

        total_weight = sum(weights)

        merged = [(particles[i],weights[i]) for i in range(0,len(particles))]

        new = sorted(merged, key = lambda element : element[1], reverse = True)
        #print(new)
        num_particles_to_keep = int(0.8*len(new))
        selected_particles = new[:num_particles_to_keep]
        
        new_gen = PoseArray()
        for x in selected_particles:
            p = Pose()
            p = x
            new_gen.poses.append(p)
        return new_gen

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

        if not self.particlecloud.poses:
            raise ValueError("Particle cloud is empty")

        sorted_particles = sorted(self.particlecloud.poses, key = lambda particle: particle.orientation.w, reverse=True)



        best_particles = sorted_particles[:len(sorted_particles) // 2]

        if not best_particles:
            return Pose()

        estimated_pose = Pose()
        estimated_pose.position.x = sum(particle.position.x for particle in best_particles) / len(best_particles)
        estimated_pose.position.y = sum(particle.position.y for particle in best_particles) / len(best_particles)

        estimated_pose.orientation.x = sum(particle.orientation.x for particle in best_particles) / len(best_particles)
        estimated_pose.orientation.y = sum(particle.orientation.y for particle in best_particles) / len(best_particles)
        estimated_pose.orientation.z = sum(particle.orientation.z for particle in best_particles) / len(best_particles)
        estimated_pose.orientation.w = sum(particle.orientation.w for particle in best_particles) / len(best_particles)


        return estimated_pose

    pass
