# src/control/gait_controller.py
import numpy as np
import copy
import sys
import os

sys.path.append(os.path.expanduser("~/robot"))

from src.gait.bezier_gait import BezierGait
from src.kinematics.RobotModel import RobotModel

class GaitController:
    def __init__(self, Tswing=0.25, dt=0.02):
        self.gait_gen = BezierGait(Tswing=Tswing, dt=dt)
        self.robot_model = RobotModel()
        self.T_bf_base = copy.deepcopy(self.robot_model.WorldToFoot)
        self.dt = dt

        # Parámetros ajustables
        self.max_vel_xy = 0.15    # m/s
        self.max_yaw_rate = 1.0   # rad/s
        self.clearance_height = 0.04
        self.penetration_depth = 0.005

    def compute_gait(self, vel_x, vel_y, yaw_rate):
        L = np.hypot(vel_x, vel_y) / 2.0
        LateralFraction = np.arctan2(vel_y, vel_x) if L > 0.01 else 0.0
        vel = np.hypot(vel_x, vel_y)

        return self.gait_gen.GenerateTrajectory(
            L=L,
            LateralFraction=LateralFraction,
            YawRate=yaw_rate,
            vel=vel,
            T_bf_=self.T_bf_base,
            T_bf_curr=self.T_bf_base,
            clearance_height=self.clearance_height,
            penetration_depth=self.penetration_depth,
            contacts=[1,1,1,1],
            dt=self.dt
        )