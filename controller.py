import random
import pybullet as p
import math
from drone import Drone

class Controller:
    # constructor
    def __init__(self, drone : Drone):
        self.drone = drone
    
    # capability model - start with randomized values
    # takes in self and a binary sensor_input
    # returns set of [pitch, roll, yaw, thrust] values
    def capability_model(self, sensor_input : bool):
        if sensor_input is True:
            pitch: int = random.randint(-30, 30)
            roll: int = random.randint(-30, 30)
            yaw = random.randint(-30, 30)
            thrust = random.randint(0, self.drone.max_thrust)
        else: # if nothing detected, just go forward
            pitch = -10
            roll = 0
            yaw = 0
            thrust = random.randint(0, self.drone.max_thrust)
        return [pitch, roll, yaw, thrust]


    # applies velocity to drone
    # takes in self and float array capability_model[p,r,y,t] and applies to drone
    def apply_force(self, capability_model : list):
        pitch : int = capability_model[0]
        roll : int = capability_model[1]
        yaw : int = capability_model[2]
        thrust : int = capability_model[3]

        def deg_to_rad(degrees : int):
            return degrees * math.pi / 180

        # calculate local force components
        fx_local = thrust * math.sin (deg_to_rad(pitch))
        fy_local = -thrust * math.sin (deg_to_rad(roll))
        fz = thrust * math.cos (deg_to_rad(pitch)) * math.cos (deg_to_rad(roll))

        # apply yaw rotation to the (fx, fy) vector
        yaw_rad = deg_to_rad(yaw)
        fx = fx_local * math.cos(yaw_rad) - fy_local * math.sin(yaw_rad)
        fy = fx_local * math.sin(yaw_rad) + fy_local * math.cos(yaw_rad)
        
        p.applyExternalForce(self.drone.drone_id, -1, [fx, fy, fz], [0, 0, 0], p.WORLD_FRAME)

        
        


            
