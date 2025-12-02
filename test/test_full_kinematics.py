#!/usr/bin/env python3
"""
Test de cinemática completa para robot cuadrúpedo.
Valida la cadena: RobotModel → IK → ángulos lógicos → (simulación de) ServoController.
"""

import numpy as np
import os
import json
from src.kinematics.RobotModel import RobotModel

# -----------------------------
# Configuración
# -----------------------------
CONFIG_DIR = os.path.expanduser("~/robot/src/config")
USE_HARDWARE = True  # Cambia a True si quieres mover servos reales

# -----------------------------
# Cargar offsets y calibración (solo para diagnóstico)
# -----------------------------
def load_calibration(config_dir):
    with open(os.path.join(config_dir, "servo_calibration.json"), "r") as f:
        calib = json.load(f)
    with open(os.path.join(config_dir, "offsets.json"), "r") as f:
        offsets = json.load(f)
    return calib, offsets

# -----------------------------
# Simulación de ServoController (sin hardware)
# -----------------------------
def simulate_set_servo_angle(name, logical_angle_deg, calib, offsets):
    """Simula lo que haría ServoController.set_servo_angle."""
    cal = calib[name]
    offset = offsets.get(name, 0.0)

    # Aplicar inversión lógica (cambio de signo)
    if cal["invert_direction"]:
        logical_angle_deg = -logical_angle_deg

    # Ángulo físico absoluto
    physical = cal["zero_angle"] + logical_angle_deg + offset
    physical = np.clip(physical, 0, 180)

    print(f"  {name:15} | lógico={logical_angle_deg:6.1f}° | físico={physical:6.1f}°")

# -----------------------------
# Función principal
# -----------------------------
def main():
    print("=== Test de Cinemática Completa ===\n")

    # 1. Crear modelo del robot
    robot = RobotModel()

    # 2. Definir postura deseada del cuerpo
    orn = np.array([0.0, 0.0, 0.0])   # roll, pitch, yaw (rad)
    pos = np.array([0.0, 0.0, 0.0])   # x, y, z (m)

    # Usar la postura de los pies definida en RobotModel como objetivo
    T_bf = robot.WorldToFoot  # dict con transformaciones cuerpo -> pie

    # 3. Calcular ángulos IK (en radianes)
    print("Calculando ángulos IK...")
    joint_angles_rad = robot.IK(orn, pos, T_bf)  # shape (4, 3)
    joint_angles_deg = np.degrees(joint_angles_rad)

    leg_ids = ["FL", "FR", "RL", "RR"]
    joint_names = ["hip_roll", "hip_pitch", "knee"]

    # 4. Cargar calibración para simulación
    calib, offsets = load_calibration(CONFIG_DIR)

    # 5. Mostrar y simular envío a servos
    print("\nResultados por pata:")
    print("--------------------------------------------------------")
    for i, leg in enumerate(leg_ids):
        print(f"\n{leg} (ángulos lógicos en grados):")
        angles = joint_angles_deg[i]
        for j, angle in enumerate(angles):
            print(f"  {joint_names[j]:10} = {angle:7.2f}°")

        # Simular envío a servos
        servo_names = [f"{leg}_hip_roll", f"{leg}_hip_pitch", f"{leg}_knee"]
        print("  -> Ángulos físicos en servos:")
        for name, angle in zip(servo_names, angles):
            simulate_set_servo_angle(name, angle, calib, offsets)

    # 6. Opcional: Enviar a hardware real
    if USE_HARDWARE:
        print("\nEnviando comandos a servos reales...")
        from src.hardware.ServoController import ServoController
        controller = ServoController(config_dir=CONFIG_DIR)
        try:
            for i, leg in enumerate(leg_ids):
                controller.set_leg_angles(leg, joint_angles_deg[i])
        finally:
            controller.deinit()

    print("\n=== Test completado ===")

if __name__ == "__main__":
    main()