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
            droll = -3 * self.dt
            dpitch = -2 * self.dt
            dyaw = 2 * self.dt
            speed = 1
        else:
            droll = 30 * self.dt
            dpitch = 20 * self.dt
            dyaw = -20 * self.dt
            speed = 1
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




