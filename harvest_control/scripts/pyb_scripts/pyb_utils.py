import numpy as np
import pybullet as p
import pybullet_data
from pybullet_utils import bullet_client as bc


class PybUtils:
    def __init__(self, env, renders: bool = False) -> None:
        """ Base class for the PyBullet Client

        Args:
            env (class): initialized class that starts this base class
            renders (bool, optional): visualize the env with the PyBullet GUI. Defaults to False.
        """
        self.renders = renders
        self.env = env
        self.step_time = 1 / 240

        self.con = None

        self.setup_pybullet()
    
    def setup_pybullet(self) -> None:
        # New class for pybullet
        if self.renders:
            self.con = bc.BulletClient(connection_mode=p.GUI)
            self.con.configureDebugVisualizer(self.con.COV_ENABLE_GUI, 0)
        else:
            self.con = bc.BulletClient(connection_mode=p.DIRECT)

        self.con.setAdditionalSearchPath(pybullet_data.getDataPath())

        self.con.setTimeStep(self.step_time)

        self.enable_gravity()

        self.con.resetDebugVisualizerCamera(cameraDistance=2, cameraYaw=90, cameraPitch=-10,
                                            cameraTargetPosition=[0, 0.75, 0.75])

    def disable_gravity(self):
        self.con.setGravity(0, 0, 0)

    def enable_gravity(self):
        self.con.setGravity(0, 0, -10)

    def disconnect(self):
        self.con.disconnect()