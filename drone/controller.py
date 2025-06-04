import random
import pybullet as p
import math
# from drone.drone import Drone

class Controller:
    # constructor
    def __init__(self, drone):
        self.drone = drone
    
    # capability model - start with randomized values
    # takes in self and a binary sensor_input
    def capability_model(self, sensor_input : bool):
        if sensor_input is True:
            roll_rate = 0 
            pitch_rate = 0
            yaw_rate = 0.05
            thrust = 10
        else:
            roll_rate = 0 
            pitch_rate = 0
            yaw_rate = -0.05
            thrust = 10
        return [roll_rate, pitch_rate, yaw_rate, thrust]

    # applies velocity to drone - idealized dynamics
    def apply_rotation_thrust(self, capability_model):
        drone = self.drone
        roll, pitch, yaw = drone.euler_orientation
        roll_rate, pitch_rate, yaw_rate, thrust = capability_model
        drone.euler_orientation = [roll + roll_rate, pitch + pitch_rate, yaw + yaw_rate]
        drone.thrust = thrust
        orientation = p.getQuaternionFromEuler(drone.euler_orientation)

        # adjust orientation based
        p.resetBasePositionAndOrientation(drone.drone_id, drone.position, orientation)


    # applies force to drone
    # takes in self and applies force to drone
    def apply_force(self):
        drone = self.drone
        roll, pitch, yaw = self.drone.euler_orientation
        thrust = drone.thrust

        # calculate local force components
        fx_local = thrust * math.sin(pitch)
        fy_local = -thrust * math.sin(roll)
        fz = thrust * math.cos(pitch) * math.cos(roll)

        # apply yaw rotation to the (fx, fy) vector
        fx = fx_local * math.cos(yaw) - fy_local * math.sin(yaw)
        fy = fx_local * math.sin(yaw) + fy_local * math.cos(yaw)
        
        p.applyExternalForce(self.drone.drone_id, -1, [fx, fy, fz], [0, 0, 0], p.WORLD_FRAME)

        
        


            
