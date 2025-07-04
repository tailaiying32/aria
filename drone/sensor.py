import math 
import numpy as np
import pybullet as p
from drone.controller import SensorPosition

# we leverage this rule - cos(theta) = (a dot b)/(||a|| x ||b||)

class Sensor:
    # constructor
    # holds the drone this sensor is attached to, the fov, and whether to display debuggging lines
    # position indicates where the sensor is on the drone - one of (0, 1, 2, 3, 4, 5) corresponding to (front, back, top, bottom, left, right)
    def __init__(self, drone, fov, debug_lines, position):
        self.drone = drone
        self.fov: float = fov
        self.half_fov = fov / 2
        self.debug_lines = debug_lines
        self.position = position

    
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
        
        # then calculate the forward vector relative to the drone's orientation and sensor position
        rotation_matrix = self.drone.get_drone_position()[1]

        match (self.position):
            case SensorPosition.FRONT: global_forward = np.array([1, 0, 0])
            case SensorPosition.BACK: global_forward = np.array([-1, 0, 0])
            case SensorPosition.TOP: global_forward = np.array([0, 0, 1])
            case SensorPosition.BOTTOM: global_forward = np.array([0, 0, -1])
            case SensorPosition.LEFT: global_forward = np.array([0, 1, 0])
            case SensorPosition.RIGHT: global_forward = np.array([0, -1, 0])
            case _ : global_forward = np.array([0, 0, 0])

        relative_forward = np.matmul(rotation_matrix, global_forward)

        # finally, find the angle between the two vectors
        magnitude_forward = np.linalg.norm(relative_forward)
        magnitude_target = np.linalg.norm(relative_target_vector)

        # check to make sure that drone is within range of the sensor
        if magnitude_target > 2:
            return False

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
    
