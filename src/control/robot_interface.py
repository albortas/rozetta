# src/control/robot_interface.py
import numpy as np
import copy
import sys
import os

sys.path.append(os.path.expanduser("~/robot"))

from src.kinematics.RobotModel import RobotModel
from src.hardware.ServoController import ServoController

class RobotInterface:
    def __init__(self, config_dir="~/robot/src/config"):
        self.robot_model = RobotModel()
        self.servo_controller = ServoController(config_dir=config_dir)
        self.T_bf_base = copy.deepcopy(self.robot_model.WorldToFoot)
        self.leg_order = ["FL", "FR", "RL", "RR"]

    def send_joint_angles(self, joint_angles_rad):
        """Envía ángulos (radianes) a los servos."""
        angles_deg = np.degrees(joint_angles_rad)
        for i, leg in enumerate(self.leg_order):
            self.servo_controller.set_leg_angles(leg, angles_deg[i])

    def get_neutral_angles(self):
        return self.robot_model.IK(
            orn=np.array([0.0, 0.0, 0.0]),
            pos=np.array([0.0, 0.0, 0.0]),
            T_bf=self.T_bf_base
        )

    def get_posture_angles(self, roll, pitch, yaw):
        orn = np.array([roll, pitch, yaw])
        return self.robot_model.IK(orn, np.array([0,0,0]), self.T_bf_base)

    def get_gait_angles(self, T_bf_gait):
        return self.robot_model.IK(np.array([0,0,0]), np.array([0,0,0]), T_bf_gait)

    def go_to_neutral(self):
        angles = self.get_neutral_angles()
        self.send_joint_angles(angles)

    def shutdown(self):
        self.go_to_neutral()
        self.servo_controller.deinit()