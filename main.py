import pybullet as p
import time
import pybullet_data

# connect to physics server
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-9.81)

# create ground
ground = p.loadURDF("plane.urdf")

# create and load point mass
def create_drone(position=[0, 0, 2], color=[1, 0, 0, 1]):
    sphereRadius = 0.1
    colSphereId = p.createCollisionShape(p.GEOM_SPHERE, radius=sphereRadius)
    visualShapeId = p.createVisualShape(p.GEOM_SPHERE, radius=sphereRadius, rgbaColor=color)
    
    mass = 1.0
    droneId = p.createMultiBody(mass, colSphereId, visualShapeId, position)
    return droneId

droneId = create_drone([0,0,1], [1,0,0,1])

while True:
    p.stepSimulation()
    time.sleep(1./240.)