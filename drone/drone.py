import pybullet as p
from drone.controller import Controller

class Drone:
    # constructor
    def __init__(self):
        self.drone_id = Drone.create_drone(self)
        self.max_thrust = 15
        self.controller = Controller(self)


    # create and load point mass
    def create_drone(self, position=[0, 0, 2], color=[1, 0, 0, 1]):
        sphereRadius = 0.1
        colSphereId = p.createCollisionShape(p.GEOM_SPHERE, radius=sphereRadius)
        visualShapeId = p.createVisualShape(p.GEOM_SPHERE, radius=sphereRadius, rgbaColor=color)
        
        mass = 1.0
        droneId = p.createMultiBody(mass, colSphereId, visualShapeId, position)
        return droneId

