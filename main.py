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

# create list of drones
drones = []

for i in range(1, 10):
    drones.append(Drone())

while True:
    for drone in drones:
        sensor_input = drone.sensor.detect([])
        capability_model = drone.controller.capability_model(sensor_input)
        drone.controller.apply_force(capability_model)

    p.stepSimulation()
    time.sleep(1./240.)