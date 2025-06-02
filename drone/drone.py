import pybullet as p
from drone.controller import Controller
from drone.sensor import Sensor
import numpy as np

class Drone:
    # constructor
    def __init__(self):
        self.drone_id = Drone.create_drone(self)
        self.max_thrust = 15
        self.controller = Controller(self)
        self.sensor = Sensor(self)

    # create and load point mass
    def create_drone(self, position=[0, 0, 2], color=[1, 0, 0, 1]):
        sphereRadius = 0.1
        colSphereId = p.createCollisionShape(p.GEOM_SPHERE, radius=sphereRadius)
        visualShapeId = p.createVisualShape(p.GEOM_SPHERE, radius=sphereRadius, rgbaColor=color)
        
        mass = 1.0
        droneId = p.createMultiBody(mass, colSphereId, visualShapeId, position)
        return droneId

    
    # takes in the drone and returns its coordinate position and the forward direction relative to the drone
    def get_drone_position(self):
        # get drone's position and orientation as a pair tuple
        position, orientation = p.getBasePositionAndOrientation(self.drone_id)

        # 3 by 3 rotation matrix representing the drone's orientation
        rotation_matrix = np.array(p.getMatrixFromQuaternion(orientation)).reshape(3, 3)

        return (position, rotation_matrix)

