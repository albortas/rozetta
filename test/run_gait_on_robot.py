#!/usr/bin/env python3
"""
Ejecuta la marcha de Bézier en el robot físico.
Corregido: NO acumula desplazamientos → siempre parte de la postura neutral.
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
    print("Iniciando marcha en robot físico (¡versión corregida!)...")

    # === Parámetros de marcha ===
    Tswing = 0.25          # tiempo de swing (segundos)
    L = 0.05               # mitad de la longitud del paso (metros) → total = 10 cm
    LateralFraction = 0.0  # 0 = recto
    YawRate = 0.0          # sin giro
    vel = 0.15             # m/s (baja velocidad para prueba)
    clearance_height = 0.04
    penetration_depth = 0.005
    dt = 0.02              # ciclo de control

    # === Inicializar ===
    robot_model = RobotModel()
    gait_gen = BezierGait(Tswing=Tswing, dt=dt)
    servo_controller = ServoController(config_dir="~/robot/src/config")

    # ✅ Postura NEUTRAL FIJA (¡no se modifica nunca!)
    T_bf_base = copy.deepcopy(robot_model.WorldToFoot)
    leg_order = ["FL", "FR", "RL", "RR"]
    contacts = [1, 1, 1, 1]  # Simulación de contacto (en hardware real, mejora esto)

    print("Postura neutral cargada. Iniciando marcha en 2 segundos...")
    time.sleep(2)

    try:
        start_time = time.time()
        cycle = 0
        while True:
            # ✅ SIEMPRE generar desde la postura base (¡nunca desde T_bf anterior!)
            T_bf = gait_gen.GenerateTrajectory(
                L=L,
                LateralFraction=LateralFraction,
                YawRate=YawRate,
                vel=vel,
                T_bf_=T_bf_base,      # ← ¡clave! postura neutral fija
                T_bf_curr=T_bf_base,  # ← igual
                clearance_height=clearance_height,
                penetration_depth=penetration_depth,
                contacts=contacts,
                dt=dt
            )

            # Extraer coordenadas para diagnóstico (opcional)
            if cycle % 50 == 0:  # cada ~1 segundo
                p_fl = T_bf["FL"][:3, 3]
                print(f"t={time.time()-start_time:.2f}s | FL: x={p_fl[0]:.3f}, y={p_fl[1]:.3f}, z={p_fl[2]:.3f} m")

            # Calcular ángulos IK
            joint_angles_rad = robot_model.IK(
                orn=np.array([0.0, 0.0, 0.0]),
                pos=np.array([0.0, 0.0, 0.0]),
                T_bf=T_bf
            )
            joint_angles_deg = np.degrees(joint_angles_rad)

            # Enviar a servos
            for i, leg in enumerate(leg_order):
                servo_controller.set_leg_angles(leg, joint_angles_deg[i])

            # Control de tiempo
            elapsed = time.time() - start_time
            sleep_time = dt - (elapsed % dt)
            if sleep_time > 0:
                time.sleep(sleep_time)

            cycle += 1

    except KeyboardInterrupt:
        print("\nInterrupción recibida. Regresando a postura neutral...")
    finally:
        # Regresar a postura neutral segura
        neutral_angles_rad = robot_model.IK(
            orn=np.array([0, 0, 0]),
            pos=np.array([0, 0, 0]),
            T_bf=T_bf_base
        )
        neutral_deg = np.degrees(neutral_angles_rad)
        for i, leg in enumerate(leg_order):
            servo_controller.set_leg_angles(leg, neutral_deg[i])
        servo_controller.deinit()
        print("Robot en postura neutral. ¡Marcha finalizada!")


if __name__ == "__main__":
    main()