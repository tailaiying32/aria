import csv
import pybullet as p
import time
import pybullet_data
from drone.drone import Drone
import sys
import ast

if len(sys.argv) < 4:
    print("\nPlease run python main.py <num_drones> <sim_length> <log_length>, where sim_length >= log_length\n")
    sys.exit()

num_drones = int(sys.argv[1])
sim_length = int(sys.argv[2]) * 240
log_length = int(sys.argv[3]) * 240

if sim_length < log_length:
    print("\nPlease ensure sim_length is greater than log_length\n")
    sys.exit()


# connect to physics server
physicsClient = p.connect(p.DIRECT) # change to p.GUI if you want to render the GUI and watch the drones
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

for i in range(0, num_drones):
    drones.append(Drone())

# open csv before loop
with open("drone_positions.csv", "w", newline="") as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow(["Controller model: " + " ".join(str(x) for x in drones[0].controller.model)])
    writer.writerow(["time_step", "drone_id", "x", "y", "z"])

    for step in range(sim_length + 1): # add 1 to make sure last time step position is recorded
        for drone in drones:
            p.changeDynamics(drone.drone_id, -1, linearDamping=0.0, angularDamping=0.0)
            other_drones = [other for other in drones if other != drone]
            sensor_input = []
            for sensor in drone.sensors:
                sensor_input.append(sensor.detect(other_drones))
            capability_model = drone.controller.capability_model(sensor_input)
            drone.controller.apply_capability_model(capability_model)

        p.stepSimulation()
        # time.sleep(1./240.)

        # write positions for each drone for the last [log_length] seconds
        if step >= (sim_length - log_length) and step % 120 == 0:
            for drone in drones:
                pos = drone.get_drone_position()[0]
                writer.writerow([step / 240., drone.drone_id] + list(pos))

print("\n")
print("Positions saved to drone_positions.csv\n")