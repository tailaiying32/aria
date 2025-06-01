from drone.drone import Drone
import math 
import numpy as np

class Sensor:
    # constructor
    # holds the drone this sensor is attached to, the fov, and whether to display debuggging lines
    def __init__(self, drone, fov = 60.0, debug_lines = False):
        self.drone : Drone = drone
        self.fov: float = fov
        self.half_fov = fov / 2
        self.debug_lines = debug_lines

    # takes in the drone and returns its coordinate position and the forward direction relative to the drone
    def get_drone_position(self):
        return NotImplemented
    
    # 

