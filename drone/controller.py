import random
import pybullet as p
import math
import numpy as np

class Controller:
    # constructor
    def __init__(self, drone):
        self.drone = drone
        self.dt = 240./240.
    
    # capability model - start with randomized values
    def capability_model(self, sensor_input : bool):
        # roll, pitch, yaw = p.getEulerFromQuaternion(p.getBasePositionAndOrientation(self.drone.drone_id)[1])
        if sensor_input is True:
            droll = -5 * self.dt
            dpitch = 5 * self.dt
            dyaw = 5 * self.dt
            speed = 2
        else:
            droll = 5 * self.dt
            dpitch = -5 * self.dt
            dyaw = -5 * self.dt
            speed = 5
        return [droll, dpitch, dyaw, speed]

    # applies velocity and rotation to drone
    def apply_capability_model(self, capability_model):
        drone = self.drone
        drone_id = drone.drone_id

        droll, dpitch, dyaw, speed = capability_model

        # Set angular velocity as before
        linear_vel, _ = p.getBaseVelocity(drone_id)
        p.resetBaseVelocity(drone_id, linear_vel, [droll, dpitch, dyaw])

        # Get current orientation
        rot_matrix = drone.get_drone_position()[1]  # 3x3 rotation matrix
        new_speed = np.matmul(rot_matrix, np.array([speed, 0, 0]))  # local x-axis

        # p.applyExternalForce(drone_id, -1, new_thrust.tolist(), [0, 0, 0], p.WORLD_FRAME)
        p.resetBaseVelocity(drone_id, new_speed.tolist(), [droll, dpitch, dyaw])


        
        # Calculate forces based on current orientation
        # position, orientation = p.getBasePositionAndOrientation(drone_id)
        # roll, pitch, yaw = p.getEulerFromQuaternion(orientation)
        
        # # Calculate local force components
        # fx_local = thrust * math.sin(pitch)
        # fy_local = -thrust * math.sin(roll) 
        # fz = thrust * math.cos(pitch) * math.cos(roll) + 10  # +10 to fight gravity
        
        # # Apply yaw rotation to the (fx, fy) vector
        # fx = fx_local * math.cos(yaw) - fy_local * math.sin(yaw)
        # fy = fx_local * math.sin(yaw) + fy_local * math.cos(yaw)
        
        # p.applyExternalForce(drone_id, -1, [fx, fy, fz], [0, 0, 0], p.WORLD_FRAME)



