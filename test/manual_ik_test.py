#!/home/pi/spotmicroai/venv/bin/python3 -u

import numpy as np
import sys
import signal
import os

# --- Cargar tu IK ---
from src.kinematics.LegKinematics import LegIK

# --- Controlador simple de servos ---
import busio
from board import SCL, SDA
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
import json

class SimpleServoController:
    def __init__(self, config_path="~/robot/src/config/robot.json"):
        config_path = os.path.expanduser(config_path)
        with open(config_path, 'r') as f:
            config = json.load(f)

        # PCA9685
        board_cfg = config["motion_controller"][0]["boards"][0]["pca9685_1"][0]
        i2c = busio.I2C(SCL, SDA)
        self.pca = PCA9685(
            i2c,
            address=int(board_cfg["address"], 0),
            reference_clock_speed=board_cfg["reference_clock_speed"]
        )
        self.pca.frequency = board_cfg["frequency"]

        # Servos
        self.servos = {}
        servos_cfg = config["motion_controller"][0]["servos"][0]
        for name, data in servos_cfg.items():
            d = data[0]
            if d["pca9685"] == 1:
                s = servo.Servo(self.pca.channels[d["channel"]])
                s.set_pulse_width_range(min_pulse=d["min_pulse"], max_pulse=d["max_pulse"])
                self.servos[name] = s

        signal.signal(signal.SIGINT, self._exit)
        signal.signal(signal.SIGTERM, self._exit)

    def _exit(self, signum, frame):
        print("\n🛑 Apagando servos...")
        self.pca.deinit()
        sys.exit(0)

    def move_servo(self, name, angle_deg):
        if name in self.servos:
            angle_deg = float(np.clip(angle_deg, 0, 180))
            self.servos[name].angle = angle_deg
            print(f"   → {name}: {angle_deg:.1f}°")
        else:
            print(f"   ❌ Servo '{name}' no encontrado.")

# --- Configuración de patas ---
LEG_SETUP = {
    "FL": {
        "ik": LegIK(legtype="LEFT"),
        "servos": ["front_shoulder_left", "front_leg_left", "front_feet_left"],
        "invert": False
    },
    "FR": {
        "ik": LegIK(legtype="RIGHT"),
        "servos": ["front_shoulder_right", "front_leg_right", "front_feet_right"],
        "invert": True
    },
    "RL": {
        "ik": LegIK(legtype="LEFT"),
        "servos": ["rear_shoulder_left", "rear_leg_left", "rear_feet_left"],
        "invert": False
    },
    "RR": {
        "ik": LegIK(legtype="RIGHT"),
        "servos": ["rear_shoulder_right", "rear_leg_right", "rear_feet_right"],
        "invert": True
    }
}

# --- ✅ SOLO ESTOS DOS SERVOS GIRAN EN DIRECCIÓN ERRÓNEA ---
INVERTED_SHOULDERS = {
    "front_shoulder_right",
    "rear_shoulder_left"
}

def main():
    print("🦵 Prueba manual de cinemática inversa (IK → servo)")
    print("   - Postura neutral IK = 0° → servo = 90°")
    print("   - Rango IK: -90° a +90° → convertido a 0°–180°")
    print("   - Sistema de coordenadas (relativo a la cadera):")
    print("        x: adelante (+)")
    print("        y: afuera de la pata (+)")
    print("        z: abajo (-) si tu IK usa z negativo\n")

    leg_id = input("Elige pata (FL, FR, RL, RR): ").strip().upper()
    if leg_id not in LEG_SETUP:
        print("❌ Pata no válida.")
        return

    config = LEG_SETUP[leg_id]
    ik_solver = config["ik"]
    servo_names = config["servos"]
    invert_ik = config["invert"]
    controller = SimpleServoController()

    print(f"\n✅ Probando pata: {leg_id} {'(invertida)' if invert_ik else ''}")
    print("Escribe 'q' en x para salir.\n")

    try:
        while True:
            try:
                x_str = input("x (m): ").strip()
                if x_str.lower() == 'q':
                    break
                x = float(x_str)
                y = float(input("y (m): "))
                z = float(input("z (m): "))
            except ValueError:
                print("   ❌ Ingresa números válidos.")
                continue

            print(f"\n➡️  IK para [x={x:.3f}, y={y:.3f}, z={z:.3f}]...")
            try:
                angles_rad = ik_solver.solve(np.array([x, y, z]))
                angles_deg = np.degrees(angles_rad)
                print("   Ángulos IK (grados):", [f"{a:.1f}" for a in angles_deg])

                # --- 🔑 CONVERSIÓN CORREGIDA ---
                servo_angles = []
                for angle_ik, name in zip(angles_deg, servo_names):
                    # Paso 1: Aplicar inversión general de pata derecha (para simetría)
                    if invert_ik:
                        angle_ik = -angle_ik

                    # Paso 2: Corrección individual solo para los dos servos problemáticos
                    if name in INVERTED_SHOULDERS:
                        angle_servo = 90.0 - angle_ik  # Inversión específica de hombro
                    else:
                        angle_servo = angle_ik + 90.0  # Conversión normal

                    angle_servo = np.clip(angle_servo, 0, 180)
                    servo_angles.append(angle_servo)

                print("   Ángulos servo (grados):", [f"{a:.1f}" for a in servo_angles])

                # Mover servos
                for name, angle in zip(servo_names, servo_angles):
                    controller.move_servo(name, angle)

                print("   ✔️  Movimiento aplicado.\n")

            except Exception as e:
                print(f"   ❌ Error en IK: {e}\n")

    except KeyboardInterrupt:
        print("\n👋 Saliendo...")
    finally:
        controller.pca.deinit()

if __name__ == "__main__":
    main()