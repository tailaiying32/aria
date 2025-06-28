import csv
import pprint
import pybullet as p
import time
import pybullet_data
from drone.drone import Drone
from drone.controller import Controller

# connect to physics server
# physicsClient = p.connect(p.GUI)
physicsClient = p.connect(p.DIRECT)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
p.configureDebugVisualizer(p.COV_ENABLE_MOUSE_PICKING, 1)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,0)

# Set up camera for better initial view
p.resetDebugVisualizerCamera(
    cameraDistance=8.0,
    cameraYaw=45,
    cameraPitch=-30,
    cameraTargetPosition=[0, 0, 2]
)

# create ground
# ground = p.loadURDF("plane.urdf")

# create list of drones
drones: list[Drone] = []
positions = {}
time_steps = 0

for i in range(0, 2):
    drones.append(Drone())

# open csv before loop
with open("drone_positions.csv", "w", newline="") as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow(["Controller model: " + " ".join(str(x) for x in drones[0].controller.model)])
    writer.writerow(["time_step", "drone_id", "x", "y", "z"])

    for step in range(100):
        for drone in drones:
            p.changeDynamics(drone.drone_id, -1, linearDamping=0.0, angularDamping=0.0)
            other_drones = [other for other in drones if other != drone]
            sensor_input = []
            for sensor in drone.sensors:
                sensor_input.append(sensor.detect(other_drones))
            capability_model = drone.controller.capability_model(sensor_input)
            drone.controller.apply_capability_model(capability_model)

        p.stepSimulation()
        time.sleep(1./240.)

        # write positions for each drone at this time step
        for drone in drones:
            pos = drone.position
            writer.writerow([step, drone.drone_id] + list(pos))

print("Positions saved to drone_positions.csv\n")