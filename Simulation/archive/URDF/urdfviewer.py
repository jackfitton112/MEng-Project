#URDF Viewer with PyBullet

import pybullet as p
import pybullet_data
import time
import os


#load tripteron.urdf into pybullet
p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")

robot = p.loadURDF("tripteron.urdf", [0, 0, 0], useFixedBase=True)

# Find link indices:
end_effector_index =  p.getBodyInfo(robot)[0].decode('UTF-8')
num_joints = p.getNumJoints(robot)
link_name_to_index = {p.getJointInfo(robot, i)[12].decode('UTF-8'): i for i in range(num_joints)}


#set camera position and orientation
p.resetDebugVisualizerCamera(cameraDistance=1, cameraYaw=0, cameraPitch=-40, cameraTargetPosition=[0, 0, 0])

#set gravity
p.setGravity(0, 0, -10)

#set time step
p.setTimeStep(1/240)

#set real time simulation
p.setRealTimeSimulation(1)

#set simulation time
p.setTimeStep(1/240)

while (1):
    p.stepSimulation()
    time.sleep(1/240)