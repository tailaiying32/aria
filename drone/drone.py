import pybullet as p
from drone.controller import Controller
from drone.sensor import Sensor
import numpy as np
import random
import math
from drone.controller import SensorPosition

class Drone:
    # constructor
    def __init__(self, pos):
        """ generate random values for position, color, and orientation """
        x, y, z = pos
        r, g, b = [random.random() for _ in range(3)]
        roll, pitch, yaw = [random.uniform(0, 2 * math.pi) for _ in range(3)]
        # roll, pitch, yaw = [0, 0, 0]
        orientation = p.getQuaternionFromEuler([roll, pitch, yaw])

        self.drone_id = Drone.create_drone(self, [x, y, z], [r, g, b, 1])

        self.max_thrust = 30
        # self.thrust = 0
        self.controller = Controller(self)
        self.sensors = [Sensor(self, 60.0, False, SensorPosition.FRONT), Sensor(self, 60.0, False, SensorPosition.TOP)]
        self.position = [x, y, z]
        self.collided = False
        self.out_of_bounds = False
        # self.orientation = [roll, pitch, yaw]

        p.resetBasePositionAndOrientation(self.drone_id, [x, y, 2], orientation)

    # create and load point mass
    def create_drone(self, position, color):
        sphereRadius = 0.1
        colSphereId = p.createCollisionShape(p.GEOM_SPHERE, radius=sphereRadius)
        visualShapeId = p.createVisualShape(p.GEOM_SPHERE, radius=sphereRadius, rgbaColor=color)
        
        mass = 1.0
        # droneId = p.createMultiBody(mass, colSphereId, visualShapeId, position)
        droneId = p.loadURDF("models/eyeball.urdf", position, [0, 0, 0, 1])
        return droneId

    
    # takes in the drone and returns its coordinate position and the forward direction relative to the drone
    def get_drone_position(self):
        # get drone's position and orientation as a pair tuple
        position, orientation = p.getBasePositionAndOrientation(self.drone_id)

        # 3 by 3 rotation matrix representing the drone's orientation
        rotation_matrix = np.array(p.getMatrixFromQuaternion(orientation)).reshape(3, 3)

        return (position, rotation_matrix)



