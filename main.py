import csv
import pybullet as p
import time
import pybullet_data
from drone.drone import Drone
import sys
import ast

import numpy as np
KP= 10.0  # Proportional gain for upright control
KD= 1.0   # Derivative gain for upright control
def apply_upright_control(drone_id):
    # Get current state
    _, orn = p.getBasePositionAndOrientation(drone_id)
    _, _, yaw = p.getEulerFromQuaternion(orn)
    _, ang_vel = p.getBaseVelocity(drone_id)

    # Desired orientation (zero roll/pitch, keep yaw)
    desired_euler = [0.0, 0.0, yaw]
    desired_quat = p.getQuaternionFromEuler(desired_euler)

    # Orientation error quaternion: q_err = q_desired * q_current_conj
    q1 = desired_quat
    q2 = orn
    error_quat = p.getDifferenceQuaternion(q1, q2)
    # if error_quat[0] != 0:
        # print(f"Error quaternion: {error_quat}, {drone_id}")

    # Clamp scalar part to avoid invalid arccos
    w = np.clip(error_quat[3], -1.0, 1.0)
    angle = 2 * np.arccos(w)

    # Avoid division by zero when angle is small
    sin_half_angle_sq = max(1e-8, 1.0 - w ** 2)
    sin_half_angle = np.sqrt(sin_half_angle_sq)

    # Axis of rotation (normalized)
    if sin_half_angle > 1e-5:
        axis = np.array(error_quat[:3]) / sin_half_angle
    else:
        axis = np.zeros(3)

    # PD torque = -KP * angle * axis - KD * angular_velocity
    torque = -KP * angle * axis - KD * np.array(ang_vel)
    

    # Apply torque in world frame
    p.applyExternalTorque(drone_id, -1, torque.tolist(), p.WORLD_FRAME)
class Main:
    def __init__(self, num_drones = 8, sim_length = 20, log_length = 10, render_GUI = False, env_size = 5):
        # simulation parameters
        self.num_drones = num_drones
        self.sim_length = sim_length
        self.log_length = log_length
        self.render_GUI = render_GUI
        self.env_size = env_size

        # out of bounds flag
        self.out_of_bounds = False 

    def main(self):
        if len(sys.argv) < 4:
            print("\nPlease run python main.py <num_drones> <sim_length> <log_length>, where sim_length >= log_length\n")
            sys.exit()

        if self.sim_length < self.log_length:
            print("\nPlease ensure sim_length is greater than log_length\n")
            sys.exit()


        # connect to physics server
        physicsClient = p.connect(p.GUI) if self.render_GUI else p.connect(p.DIRECT) 
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

        for _ in range(0, self.num_drones):
            drones.append(Drone())

        # open csv before loop
        with open("drone_positions.csv", "w", newline="") as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(["time_step", "drone_id", "x", "y", "z","controller"])

            for step in range(self.sim_length + 1): # add 1 to make sure last time step position is recorded
                for drone in drones:
                    p.changeDynamics(drone.drone_id, -1, linearDamping=0.0, angularDamping=0.0)
                    other_drones = [other for other in drones if other != drone]
                    sensor_input = []
                    for sensor in drone.sensors:
                        sensor_input.append(sensor.detect(other_drones))
                    capability_model = drone.controller.capability_model(sensor_input)
                    drone.controller.apply_capability_model(capability_model)
                    
                    # check if drones are out of bounds
                    x, y, z = drone.get_drone_position()[0]
                    if abs(x) >= self.env_size or abs(y) >= self.env_size or abs(z) >= self.env_size:
                        # print(f"Drone {drone.drone_id} out of bounds at step {step}: ({x}, {y}, {z})")
                        # p.resetBasePositionAndOrientation(drone.drone_id, [0, 0, 2], [0, 0, 0, 1])
                        self.out_of_bounds = True
                    
                    apply_upright_control(drone.drone_id)
                    
                    # check if roll and pitch are non-zero

                    roll, pitch, yaw = p.getEulerFromQuaternion(p.getBasePositionAndOrientation(drone.drone_id)[1])
                    if abs(roll) > 0.1 or abs(pitch) > 0.1:
                        print(f"Drone {drone.drone_id} has non-zero roll/pitch at step {step}: roll={roll}, pitch={pitch}")
                    


                if self.out_of_bounds:
                    print("\nDrone out of bounds!")
                    break


                # write positions for each drone for the last [log_length] seconds
                if step >= (self.sim_length - self.log_length) and step % 120 == 0:
                    for drone in drones:
                        pos = drone.get_drone_position()[0]
                        writer.writerow([step / 240., drone.drone_id] + list(pos) + [str(drone.controller.model)])

                p.stepSimulation()

                if self.render_GUI:
                    time.sleep(1./240.)

        print("\n")
        print("Positions saved to drone_positions.csv\n")

if __name__ == "__main__":
    num_drones = int(sys.argv[1])
    sim_length = int(sys.argv[2]) * 240
    log_length = int(sys.argv[3]) * 240
    render_GUI = True 
    env_size = 10

    main_instance = Main(num_drones, sim_length, log_length, render_GUI, env_size)
    main_instance.main()