import pybullet as p
import time
import pybullet_data
from drone.drone import Drone

# connect to physics server
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-9.81)

# create ground
ground = p.loadURDF("plane.urdf")

drone = Drone()
droneId = drone.create_drone()

while True:
    p.stepSimulation()
    time.sleep(1./240.)