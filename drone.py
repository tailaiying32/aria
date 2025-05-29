import pybullet as p
from controller import Controller

class Drone:
    # constructor
    def __init__(self, max_thrust : int, controller : Controller):
        self.drone_id = Drone.create_drone(self)
        self.max_thrust = max_thrust
        self.controller = Controller


    # create and load point mass
    def create_drone(self, position=[0, 0, 2], color=[1, 0, 0, 1]):
        sphereRadius = 0.1
        colSphereId = p.createCollisionShape(p.GEOM_SPHERE, radius=sphereRadius)
        visualShapeId = p.createVisualShape(p.GEOM_SPHERE, radius=sphereRadius, rgbaColor=color)
        
        mass = 1.0
        droneId = p.createMultiBody(mass, colSphereId, visualShapeId, position)
        return droneId

