#!/home/pi/spotmicroai/venv/bin/python3 -u

import json
import os
import time
import sys

import busio
from board import SCL, SDA
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

# --- Rutas ---
CONFIG_PATH = os.path.expanduser("~/robot/src/config/robot.json")
CALIB_PATH = os.path.expanduser("~/robot/src/config/servo_calibration.json")
OFFSET_PATH = os.path.expanduser("~/robot/src/config/offsets.json")

# --- Orden cinemático ---
SERVO_ORDER = [
    "FL_hip_roll", "FL_hip_pitch", "FL_knee",
    "FR_hip_roll", "FR_hip_pitch", "FR_knee",
    "RL_hip_roll", "RL_hip_pitch", "RL_knee",
    "RR_hip_roll", "RR_hip_pitch", "RR_knee"
]

class OffsetCalibrator:
    def __init__(self):
        # Cargar configuraciones
        with open(CONFIG_PATH, 'r') as f:
            self.robot_config = json.load(f)
        with open(CALIB_PATH, 'r') as f:
            self.calib_config = json.load(f)

        # Inicializar PCA9685 #1
        board_cfg = self.robot_config["motion_controller"][0]["boards"][0]["pca9685_1"][0]
        i2c = busio.I2C(SCL, SDA)
        self.pca = PCA9685(i2c, address=int(board_cfg["address"], 0))
        self.pca.frequency = board_cfg["frequency"]

        # Crear servos
        self.servos = {}
        servos_cfg = self.robot_config["motion_controller"][0]["servos"][0]
        for name, data in servos_cfg.items():
            d = data[0]
            if d["pca9685"] == 1:
                s = servo.Servo(self.pca.channels[d["channel"]])
                s.set_pulse_width_range(min_pulse=d["min_pulse"], max_pulse=d["max_pulse"])
                self.servos[name] = s

        # Cargar offsets existentes o iniciar en 0
        self.offsets = {}
        if os.path.exists(OFFSET_PATH):
            with open(OFFSET_PATH, 'r') as f:
                self.offsets = json.load(f)
        else:
            self.offsets = {name: 0.0 for name in self.servos.keys()}

    def apply_zero_position(self, name):
        """Mueve el servo a su zero_angle + offset, respetando inversión."""
        cal = self.calib_config[name]
        base = cal["zero_angle"] + self.offsets[name]
        if cal["invert_direction"]:
            physical = 180.0 - base
        else:
            physical = base
        physical = max(0, min(180, physical))
        self.servos[name].angle = physical

    def calibrate_servo(self, name):
        print(f"\n🔧 Ajustando offset para: {name}")
        self.apply_zero_position(name)
        current_offset = self.offsets[name]
        print(f"   Offset actual: {current_offset:.1f}°")

        while True:
            cmd = input("   '+' / '-' para ajustar, 's' para guardar: ").strip()
            if cmd == 's':
                break
            elif cmd == '+':
                current_offset += 0.5
            elif cmd == '-':
                current_offset -= 0.5
            else:
                print("     ⚠️  Usa '+', '-' o 's'")
                continue

            self.offsets[name] = current_offset
            self.apply_zero_position(name)
            print(f"     → Offset: {current_offset:.1f}°")

    def run(self):
        print("📈 Calibración de offsets usando servo_calibration.json")
        print("   Moviendo cada servo a su posición de referencia (zero_angle + offset)...\n")

        for name in SERVO_ORDER:
            if name not in self.servos:
                continue
            input(f"\n➡️  Presiona Enter para calibrar: {name}")
            self.calibrate_servo(name)

        # Guardar
        os.makedirs(os.path.dirname(OFFSET_PATH), exist_ok=True)
        with open(OFFSET_PATH, 'w') as f:
            json.dump(self.offsets, f, indent=2)
        print(f"\n✅ Offsets guardados en: {OFFSET_PATH}")
        self.pca.deinit()

if __name__ == "__main__":
    try:
        calibrator = OffsetCalibrator()
        calibrator.run()
    except KeyboardInterrupt:
        print("\n🛑 Cancelado.")
        calibrator.pca.deinit()