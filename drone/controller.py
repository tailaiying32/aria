import random
import pybullet as p
import math
import numpy as np
from enum import Enum

class SensorPosition(Enum):
    FRONT = 0
    BACK = 1
    TOP = 2
    BOTTOM = 3
    LEFT = 4
    RIGHT = 5 

class Controller:
    # constructor
    def __init__(self, drone):
        self.drone = drone
        self.dt = 240./240.
        self.model = [3, 3, 0, -3, 3, 3, 3, 3, -0, -3, 3, 3]
    
    # capability model - start with randomized values
    def capability_model(self, sensor_input):
        # roll, pitch, yaw = p.getEulerFromQuaternion(p.getBasePositionAndOrientation(self.drone.drone_id)[1])
        # if sensor_input is True:
        #     droll = -0 * self.dt
        #     dpitch = -0 * self.dt
        #     dyaw = 0 * self.dt
        #     speed = 3
        # else:
        #     droll = 4 * self.dt
        #     dpitch = 6 * self.dt
        #     dyaw = 8 * self.dt
        #     speed = 0
        # return [droll, dpitch, dyaw, speed]

        # controller model based on two sensor states and yaw, forward velocity, and vertical velocity
        if sensor_input == [0, 0]:
            dyaw = self.model[0]
            vx = self.model[1]
            vz = self.model[2]
        elif sensor_input == [1, 0]:
            dyaw = self.model[3]
            vx = self.model[4]
            vz = self.model[5]
        elif sensor_input == [0, 1]:
            dyaw = self.model[6]
            vx = self.model[7]
            vz = self.model[8]
        elif sensor_input == [1, 1]:
            dyaw = self.model[9]
            vx = self.model[10]
            vz = self.model[11]
        else: 
            dyaw = 0
            vx = 0
            vz = 0 
        return [dyaw, vx, vz]

    # applies velocity and rotation to drone   
    # def apply_capability_model(self, capability_model):
    #     drone = self.drone
    #     drone_id = drone.drone_id

    #     droll, dpitch, dyaw, speed = capability_model

    #     # Set angular velocity as before
    #     linear_vel, _ = p.getBaseVelocity(drone_id)
    #     p.resetBaseVelocity(drone_id, linear_vel, [droll, dpitch, dyaw])

    #     # Get current orientation
    #     rot_matrix = drone.get_drone_position()[1]  # 3x3 rotation matrix
    #     new_speed = np.matmul(rot_matrix, np.array([speed, 0, 0]))  # local x-axis

    #     # p.applyExternalForce(drone_id, -1, new_thrust.tolist(), [0, 0, 0], p.WORLD_FRAME)
    #     p.resetBaseVelocity(drone_id, new_speed.tolist(), [droll, dpitch, dyaw])

    def apply_capability_model(self, capability_model):
        drone = self.drone
        drone_id = drone.drone_id

        dyaw, vx, vz = capability_model

        # Get current orientation
        rot_matrix = drone.get_drone_position()[1]  # 3x3 rotation matrix
        new_speed = np.matmul(rot_matrix, np.array([vx, 0, vz]))  # the local velocity

        p.resetBaseVelocity(drone_id, new_speed.tolist(), [0, 0, dyaw])
        new_position = drone.get_drone_position()[0]
        drone.position = new_position


    def apply_force(self, capability_model):
        drone = self.drone
        drone_id = drone.drone_id

        droll, dpitch, dyaw, thrust = capability_model

        # apply torque
        p.applyExternalTorque(drone_id, -1, [droll, dpitch, dyaw], p.WORLD_FRAME)
        
        # Get current orientation
        rot_matrix = drone.get_drone_position()[1]  # 3x3 rotation matrix
        new_thrust = np.matmul(rot_matrix, np.array([thrust, 0, 0]))  # local x-axis

        # p.applyExternalForce(drone_id, -1, new_thrust.tolist(), [0, 0, 0], p.WORLD_FRAME)
        p.applyExternalForce(drone_id, -1, new_thrust.tolist(), [0, 0, 0], p.WORLD_FRAME)