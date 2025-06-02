import random
import pybullet as p
import math

class Controller:
    # constructor
    def __init__(self, drone):
        self.drone = drone
    
    # capability model - start with randomized values
    # takes in self and a binary sensor_input
    # returns set of [pitch, roll, yaw, thrust] values
    def capability_model(self, sensor_input : bool):
        if sensor_input is True:
            pitch: int = random.randint(-90, 90)
            roll: int = random.randint(-90, 90)
            yaw = random.randint(-90, 90)
            thrust = 25
        else: # if nothing detected, just move randomly
            pitch: int = random.randint(-90, 90)
            roll: int = random.randint(-90, 90)
            yaw = random.randint(-90, 90)
            thrust = 25
        return [pitch, roll, yaw, thrust]


    # applies velocity to drone
    # takes in self and float array capability_model[p,r,y,t] and applies to drone
    def apply_force(self, capability_model : list):
        pitch : int = capability_model[0]
        roll : int = capability_model[1]
        yaw : int = capability_model[2]
        thrust : int = capability_model[3]

        # calculate local force components
        fx_local = thrust * math.sin (math.radians(pitch))
        fy_local = -thrust * math.sin (math.radians(roll))
        fz = thrust * math.cos (math.radians(pitch)) * math.cos (math.radians(roll))

        # apply yaw rotation to the (fx, fy) vector
        yaw_rad = math.radians(yaw)
        fx = fx_local * math.cos(yaw_rad) - fy_local * math.sin(yaw_rad)
        fy = fx_local * math.sin(yaw_rad) + fy_local * math.cos(yaw_rad)
        
        p.applyExternalForce(self.drone.drone_id, -1, [fx, fy, fz], [0, 0, 0], p.WORLD_FRAME)

        
        


            
