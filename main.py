import pybullet as p
import time
import pybullet_data
from drone.drone import Drone
from drone.controller import Controller

# connect to physics server
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,0)

# create ground
ground = p.loadURDF("plane.urdf")

# create list of drones
drones: list[Drone] = []

for i in range(0, 10):
    drones.append(Drone())

while True:
    for drone in drones:
        other_drones = [other for other in drones if other != drone]
        sensor_input = drone.sensor.detect(other_drones)
        capability_model = drone.controller.capability_model(sensor_input)
        drone.controller.apply_rotation_thrust(capability_model)
        drone.controller.apply_force()

    p.stepSimulation()
    time.sleep(1./240.)