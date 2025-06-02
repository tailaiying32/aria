import math 
import numpy as np
import pybullet as p

# we leverage this rule - cos(theta) = (a dot b)/(||a|| x ||b||)

class Sensor:
    # constructor
    # holds the drone this sensor is attached to, the fov, and whether to display debuggging lines
    def __init__(self, drone, fov = 60.0, debug_lines = False):
        self.drone = drone
        self.fov: float = fov
        self.half_fov = fov / 2
        self.debug_lines = debug_lines

    
    # returns the position vector of a target relative to self
    def get_target_pos(self, target):
        return target.get_drone_position()[0]

    # takes in self and returns whether or not a target is in its fov
    def is_in_fov(self, target_pos): 
        # first calculate the target vector relative to self
        self_pos = self.drone.get_drone_position()[0]
        target_x, target_y, target_z = target_pos


        relative_target_vector = np.array([
            target_x - self_pos[0],
            target_y - self_pos[1],
            target_z - self_pos[2]
        ])
        
        # then calculate the forward vector relative to the drone's orientation
        rotation_matrix = self.drone.get_drone_position()[1]
        global_forward = np.array([1, 0, 0])
        relative_forward = np.matmul(rotation_matrix, global_forward)

        # finally, find the angle between the two vectors
        magnitude_forward = np.linalg.norm(relative_forward)
        magnitude_target = np.linalg.norm(relative_target_vector)

        dot_product = np.dot(relative_forward, relative_target_vector)
        theta = math.acos(dot_product / (magnitude_forward * magnitude_target))

        # if the calculated angle is less than half the fov, than the target is within the field of view
        if theta < math.radians(self.fov / 2):
            return True
        else:
            return False
        
    # takes in list of other drones and returns whether or not the sensor is activated
    def detect(self, drone_list):
        for drone in drone_list:
            if self.is_in_fov(drone.get_drone_position()[0]) is True:
                return True
        
        return False
    
