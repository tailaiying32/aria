import csv
import pybullet as p
import time
import pybullet_data
from drone.drone import Drone
import sys
import random
import numpy as np

class Main:
    def __init__(self, num_drones = 8, sim_length = 20, log_length = 10, render_GUI = False, env_size = 5., sim_speed = 1.0):
        # simulation parameters
        self.num_drones = num_drones
        self.sim_length = sim_length
        self.log_length = log_length
        self.render_GUI = render_GUI
        self.env_size = env_size
        self.sim_speed = sim_speed

    def main(self):
        # connect to physics server, set up environment and camera
        physicsClient = p.connect(p.GUI) if self.render_GUI else p.connect(p.DIRECT) 
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
        p.configureDebugVisualizer(p.COV_ENABLE_MOUSE_PICKING, 1)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0,0,0)
        self.create_walls()
        p.resetDebugVisualizerCamera(
            cameraDistance=8.0,
            cameraYaw=45,
            cameraPitch=-30,
            cameraTargetPosition=[0, 0, 2]
        )

        # create list of drones
        drones: list[Drone] = []

        axis_nodes = int(np.ceil(self.num_drones ** (1/3))) + 1
        lattice_pos = self.generate_lattice(axis_nodes)

        for i in range(0, self.num_drones):
            position = lattice_pos[i]
            drones.append(Drone(position))

        # open csv before loop
        with open("drone_positions.csv", "w", newline="") as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(["time_step", "drone_id", "x", "y", "z","controller", "collision"])

            for step in range(self.sim_length + 1): # add 1 to make sure last time step position is recorded
                for drone in drones:
                    p.changeDynamics(drone.drone_id, -1, linearDamping=0.0, angularDamping=0.0)
                    other_drones = [other for other in drones if other != drone]
                    sensor_input = []
                    for sensor in drone.sensors:
                        sensor_input.append(sensor.detect(other_drones))
                    capability_model = drone.controller.capability_model(sensor_input)
                    drone.controller.apply_capability_model(capability_model)
            

                self.check_collision(drones)

                # write positions for each drone for the last [log_length] seconds
                if step >= (self.sim_length - self.log_length) and step % 120 == 0:
                    for drone in drones:
                        pos = drone.get_drone_position()[0]
                        writer.writerow([step / 240., drone.drone_id] + list(pos) + [str(drone.controller.model)] + [drone.out_of_bounds])

                p.stepSimulation()

                if self.render_GUI:
                    time.sleep(1./(240. * self.sim_speed))

        print("\n")
        print("Positions saved to drone_positions.csv\n")
    
    def create_walls(self):
        wall_thickness = 0
        
        wall_shape_x = p.createCollisionShape(p.GEOM_BOX, 
                                            halfExtents=[wall_thickness, self.env_size, self.env_size])
        wall_visual_x = p.createVisualShape(p.GEOM_BOX, 
                                        halfExtents=[wall_thickness, self.env_size, self.env_size],
                                        rgbaColor=[0.5, 0.5, 0.5, 0])
        
        wall_shape_y = p.createCollisionShape(p.GEOM_BOX, 
                                            halfExtents=[self.env_size, wall_thickness, self.env_size])
        wall_visual_y = p.createVisualShape(p.GEOM_BOX, 
                                        halfExtents=[self.env_size, wall_thickness, self.env_size],
                                        rgbaColor=[0.5, 0.5, 0.5, 0])
        
        wall_shape_z = p.createCollisionShape(p.GEOM_BOX, 
                                            halfExtents=[self.env_size, self.env_size, wall_thickness])
        wall_visual_z = p.createVisualShape(p.GEOM_BOX, 
                                        halfExtents=[self.env_size, self.env_size, wall_thickness],
                                        rgbaColor=[0.5, 0.5, 0.5, 0])
        
        walls = [
            # front and back walls
            {"shape": wall_shape_x, "visual": wall_visual_x, "pos": [self.env_size, 0, 0]},  
            {"shape": wall_shape_x, "visual": wall_visual_x, "pos": [-self.env_size, 0, 0]},
            
            # left and right walls
            {"shape": wall_shape_y, "visual": wall_visual_y, "pos": [0, self.env_size, 0]}, 
            {"shape": wall_shape_y, "visual": wall_visual_y, "pos": [0, -self.env_size, 0]},

            # top and bottom walls
            {"shape": wall_shape_z, "visual": wall_visual_z, "pos": [0, 0, self.env_size]},
            {"shape": wall_shape_z, "visual": wall_visual_z, "pos": [0, 0, -self.env_size]}
        ]
        
        for wall in walls:
            p.createMultiBody(baseMass=0, 
                            baseCollisionShapeIndex=wall["shape"],
                            baseVisualShapeIndex=wall["visual"],
                            basePosition=wall["pos"])
                     
    def check_collision(self, drones):
        for drone in drones:
            contact_points = p.getContactPoints(bodyA=drone.drone_id)
            if contact_points:
                drone.out_of_bounds = True

    def generate_lattice(self, axis_nodes):
        positions = []

        axis_range = self.env_size
        spacing = axis_range / (axis_nodes - 1)

        for i in range(axis_nodes):
            for j in range(axis_nodes):
                for k in range(axis_nodes):
                    x = -self.env_size/2 + i * spacing
                    y = -self.env_size/2 + j * spacing
                    z = -self.env_size/2 + k * spacing
                    positions.append([x, y, z])

        print(positions)
        random.shuffle(positions)

        return positions

if __name__ == "__main__":
    if len(sys.argv) < 6:
            print("\nPlease run python main.py <num_drones> <sim_length> <log_length> <sim_speed> <render_GUI (T/False)>, where sim_length >= log_length\n")
            sys.exit()

    num_drones = int(sys.argv[1])
    sim_length = int(sys.argv[2]) * 240
    log_length = int(sys.argv[3]) * 240
    sim_speed = float(sys.argv[4]) 
    render_GUI =  True if sys.argv[5] == "T" else False
    env_size = 5

    if sim_length < log_length:
        print("\nPlease ensure sim_length is greater than log_length\n")
        sys.exit()


    main_instance = Main(num_drones, sim_length, log_length, render_GUI, env_size, sim_speed)
    main_instance.main()
