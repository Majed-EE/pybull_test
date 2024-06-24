
import pybullet as p
import time

p.connect(p.GUI)  


plane_id = p.loadURDF("plane.urdf")


try:
    while True:
        p.stepSimulation()
        time.sleep(1.0 / 24.0)  # Simulate at 240 Hz
except KeyboardInterrupt:
    pass

p.disconnect()
