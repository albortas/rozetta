# main.py
#!/usr/bin/env python3
import time
import numpy as np
import sys
import os

sys.path.append(os.path.expanduser("~/robot"))

from src.controller.PS4Controller import PS4Controller
from src.control.robot_interface import RobotInterface
from src.control.gait_controller import GaitController

# Estados
NEUTRAL = 0
STATIC_POSTURE = 1
GAIT_MODE = 2

def activate_emergency(robot):
    """Activa emergencia: deshabilita servos (OE = HIGH)"""
    print("\n🚨 ¡EMERGENCIA ACTIVADA! Servos desactivados.")
    robot.servo_controller.disable_servos()
    # No intentar mover más → esperar reinicio o cierre manual

def main():
    print("Iniciando control con toggle (L1 = postura, R1 = marcha)...")

    # Inicializar módulos
    ps4 = PS4Controller()
    robot = RobotInterface(config_dir="~/robot/src/config")
    gait_ctrl = GaitController(Tswing=0.25, dt=0.02)

    # Límites
    MAX_ROLL = 0.3
    MAX_PITCH = 0.25
    MAX_YAW_STATIC = 0.5

    # Estado inicial
    current_mode = NEUTRAL
    robot.go_to_neutral()
    print("Modo: NEUTRAL")

    # Para detectar flanco ascendente (presión única)
    last_buttons = [False] * 14 
    
    try:
        while True:
            state = ps4.get_joystick_state()
            axes = state["axes"]
            buttons = state["buttons"]
            
            # 🔴 ¡EMERGENCIA! (botón OPTIONS = 9)
            if buttons[9] and not last_buttons[9]:
                activate_emergency(robot)
                print("Presiona Ctrl+C para salir.")
                time.sleep(0.5)  # anti-rebote
                continue  # no procesar más hasta reiniciar

            # 🔄 Toggle: L1 (4) → postura estática
            if buttons[4] and not last_buttons[4]:
                if current_mode == STATIC_POSTURE:
                    current_mode = NEUTRAL
                    robot.go_to_neutral()
                    print("⏹️  Modo: NEUTRAL")
                else:
                    current_mode = STATIC_POSTURE
                    print("🟢 Modo: POSTURA ESTÁTICA (L1 para salir)")

            # 🚶 Toggle: R1 (5) → marcha
            elif buttons[5] and not last_buttons[5]:
                if current_mode == GAIT_MODE:
                    current_mode = NEUTRAL
                    robot.go_to_neutral()
                    print("⏹️  Modo: NEUTRAL")
                else:
                    current_mode = GAIT_MODE
                    print("🟢 Modo: MARCHA (R1 para salir)")
            
            # === Ejecutar según modo actual ===

            if current_mode == NEUTRAL:
                angles = robot.get_neutral_angles()
                robot.send_joint_angles(angles)

            elif current_mode == STATIC_POSTURE:
                roll = -axes[0] * MAX_ROLL
                pitch = axes[1] * MAX_PITCH
                yaw = axes[3] * MAX_YAW_STATIC
                angles = robot.get_posture_angles(roll, pitch, yaw)
                robot.send_joint_angles(angles)
                if int(time.time() * 2) % 2 == 0:
                    print(f"[Postura] Roll: {roll:.2f}, Pitch: {pitch:.2f}, Yaw: {yaw:.2f}")

            elif current_mode == GAIT_MODE:  
                vel_x = -axes[1] * gait_ctrl.max_vel_xy
                vel_y = -axes[0] * gait_ctrl.max_vel_xy
                yaw_rate = axes[3] * gait_ctrl.max_yaw_rate
                T_bf = gait_ctrl.compute_gait(vel_x, vel_y, yaw_rate)
                angles = robot.get_gait_angles(T_bf)
                robot.send_joint_angles(angles)
                if int(time.time() * 2) % 2 == 0:
                    print(f"[Marcha] vx: {vel_x:.2f}, vy: {vel_y:.2f}, yaw_rate: {yaw_rate:.2f}")

            last_buttons = buttons[:]
            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\nApagando robot...")
    finally:
        print("Volviendo a Neutral...")
        robot.shutdown()
        print("¡Sistema detenido!")

if __name__ == "__main__":
    main()