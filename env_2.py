import os
import numpy as np
import pybullet as p
import pybullet_data
from panda import Panda
from objects import YCBObject, InteractiveObj, RBOObject


class DoubleEnv():

    def __init__(self):
        # create simulation (GUI)
        self.urdfRootPath = pybullet_data.getDataPath()
        p.connect(p.GUI)
        p.setGravity(0, 0, -9.81)

        # set up camera
        self._set_camera()

        # load some scene objects
        p.loadURDF(os.path.join(self.urdfRootPath, "plane.urdf"), basePosition=[0, 0, -0.65])
        p.loadURDF(os.path.join(self.urdfRootPath, "table/table.urdf"), basePosition=[0.5, 0, -0.65])
        p.loadURDF(os.path.join(self.urdfRootPath, "table/table.urdf"), basePosition=[0.5, 1, -0.65])

        # example YCB object
        obj1 = YCBObject('003_cracker_box')
        obj1.load()
        p.resetBasePositionAndOrientation(obj1.body_id, [0.7, -0.2, 0.1], [0, 0, 0, 1])

        # example RBO object
        obj2 = RBOObject('book')
        obj2.load()
        p.resetBasePositionAndOrientation(obj2.body_id, [0.8, 1.1, 0.5], [0, 0, 1, 1])

        # load a panda robot
        self.panda1 = Panda([0, 0, 0])
        self.panda2 = Panda([0, 1, 0])

    def reset(self):
        self.panda1.reset()
        self.panda2.reset()
        return [self.panda1.state, self.panda2.state]

    def close(self):
        p.disconnect()

    def step(self, action):
        # get current state
        state = [self.panda1.state, self.panda2.state]

        # action in this example is the end-effector velocity
        action1 = [action[0], action[1], action[2]]
        action2 = [action[3], action[4], action[5]]
        self.panda1.step(dposition=action1)
        self.panda2.step(dposition=action2)

        # take simulation step
        p.stepSimulation()

        # return next_state, reward, done, info
        next_state = [self.panda1.state, self.panda2.state]
        reward = 0.0
        done = False
        info = {}
        return next_state, reward, done, info

    def render(self):
        (width, height, pxl, depth, segmentation) = p.getCameraImage(width=self.camera_width,
                                                                     height=self.camera_height,
                                                                     viewMatrix=self.view_matrix,
                                                                     projectionMatrix=self.proj_matrix)
        rgb_array = np.array(pxl, dtype=np.uint8)
        rgb_array = np.reshape(rgb_array, (self.camera_height, self.camera_width, 4))
        rgb_array = rgb_array[:, :, :3]
        return rgb_array

    def _set_camera(self):
        self.camera_width = 256
        self.camera_height = 256
        p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=20, cameraPitch=-30,
                                     cameraTargetPosition=[0.5, -0.2, 0.0])
        self.view_matrix = p.computeViewMatrixFromYawPitchRoll(cameraTargetPosition=[0.5, 0, 0],
                                                               distance=1.0,
                                                               yaw=90,
                                                               pitch=-50,
                                                               roll=0,
                                                               upAxisIndex=2)
        self.proj_matrix = p.computeProjectionMatrixFOV(fov=60,
                                                        aspect=float(self.camera_width) / self.camera_height,
                                                        nearVal=0.1,
                                                        farVal=100.0)
