#!/usr/bin/env python3
"""
Ejecuta la marcha de Bézier en el robot físico.
Requiere: ServoController, RobotModel, BezierGait.
"""

import time
import numpy as np
import copy
import sys
import os

# Asegurar que src está en el path
sys.path.append(os.path.expanduser("~/robot"))

from src.kinematics.RobotModel import RobotModel
from src.hardware.ServoController import ServoController
from src.gait.bezier_gait import BezierGait


def main():
    print("Iniciando marcha en robot físico...")

    # === Configuración ===
    Tswing = 0.25          # tiempo de swing (segundos)
    L = 0.02               # mitad de la longitud del paso (metros)
    LateralFraction = 0.0  # 0 = recto, >0 = lateral
    YawRate = 0.0          # rad/s
    vel = 0.05              # m/s
    clearance_height = 0.02
    penetration_depth = 0.005
    dt = 0.02              # tiempo entre actualizaciones

    # === Inicializar componentes ===
    robot_model = RobotModel()
    gait_gen = BezierGait(Tswing=Tswing, dt=dt)
    servo_controller = ServoController(config_dir="~/robot/src/config")

    # Estado inicial: postura neutral
    T_bf = copy.deepcopy(robot_model.WorldToFoot)
    leg_order = ["FL", "FR", "RL", "RR"]

    # Contactos simulados (en hardware real, deberías usar sensores)
    contacts = [1, 1, 1, 1]

    print("Robot en postura neutral. Presiona Ctrl+C para detener.")
    time.sleep(2)

    try:
        start_time = time.time()
        while True:
            # Generar nueva postura de pies
            T_bf_ordered = {leg: T_bf[leg] for leg in leg_order}
            T_bf = gait_gen.GenerateTrajectory(
                L=L,
                LateralFraction=LateralFraction,
                YawRate=YawRate,
                vel=vel,
                T_bf_=T_bf_ordered,
                T_bf_curr=T_bf_ordered,
                clearance_height=clearance_height,
                penetration_depth=penetration_depth,
                contacts=contacts,
                dt=dt
            )

            # Calcular ángulos IK para todas las patas
            joint_angles_rad = robot_model.IK(
                orn=np.array([0.0, 0.0, 0.0]),  # sin inclinación
                pos=np.array([0.0, 0.0, 0.0]),  # cuerpo fijo
                T_bf=T_bf
            )
            joint_angles_deg = np.degrees(joint_angles_rad)

            # Enviar a servos
            for i, leg in enumerate(leg_order):
                servo_controller.set_leg_angles(leg, joint_angles_deg[i])

            # Esperar el tiempo de ciclo
            elapsed = time.time() - start_time
            sleep_time = dt - (elapsed % dt)
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\nDeteniendo marcha...")
    finally:
        # Regresar a postura neutral
        neutral_angles = robot_model.IK(
            orn=np.array([0,0,0]),
            pos=np.array([0,0,0]),
            T_bf=robot_model.WorldToFoot
        )
        neutral_deg = np.degrees(neutral_angles)
        for i, leg in enumerate(leg_order):
            servo_controller.set_leg_angles(leg, neutral_deg[i])
        servo_controller.deinit()
        print("Robot en postura neutral. ¡Listo!")


if __name__ == "__main__":
    main()