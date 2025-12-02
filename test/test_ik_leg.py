# ~/robot/src/tests/test_ik_leg.py

from src.kinematics.RobotModel import RobotModel
from src.hardware.ServoController import ServoController
from src.kinematics.LieAlgebra import RpToTrans, TransToRp
import numpy as np


def main():
    controller = ServoController()
    robot = RobotModel()

    try:
        while True:
            leg = input("\nPata (FL/FR/RL/RR) o 'q': ").strip().upper()
            if leg == "Q":
                break
            if leg not in ["FL", "FR", "RL", "RR"]:
                continue

            x = float(input("x (m): "))
            y = float(input("y (m): "))
            z = float(input("z (m): "))

            # IK para una sola pata
            from collections import OrderedDict

            T_bf = OrderedDict()
            R = np.eye(3)
            T_bf[leg] = RpToTrans(R, np.array([x, y, z]))
            for key in ["FL", "FR", "RL", "RR"]:
                if key != leg:
                    _, p = TransToRp(robot.WorldToFoot[key])
                    T_bf[key] = RpToTrans(R, p)

            angles_rad = robot.IK(np.array([0, 0, 0]), np.array([0, 0, 0]), T_bf)
            leg_idx = ["FL", "FR", "RL", "RR"].index(leg)
            angles_deg = np.degrees(angles_rad[leg_idx])

            controller.set_leg_angles(leg, angles_deg)
            print(f"✅ {leg}: {angles_deg}")

    finally:
        controller.deinit()


if __name__ == "__main__":
    main()
