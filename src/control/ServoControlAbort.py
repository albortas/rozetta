#!/home/pi/spotmicroai/venv/bin/python3 -u

import busio
from board import SCL, SDA
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
import json
import os
import time
import RPi.GPIO as GPIO
import sys

# --- Configuración de rutas y parámetros ---
CONFIG_PATH = os.path.expanduser("~/robot/src/config/robot.json")
SERVO_POWER_GPIO = 17  # Debe coincidir con tu abort_controller.gpio_port

class SafeServoController:
    def __init__(self, config_path=CONFIG_PATH):
        # Cargar configuración
        with open(config_path, 'r') as f:
            config = json.load(f)

        # Configurar GPIO para alimentación de servos
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(SERVO_POWER_GPIO, GPIO.OUT)
        self.disable_servo_power()  # Apagado inicial por seguridad

        # Leer PCA9685 #1
        board_cfg = config["motion_controller"][0]["boards"][0]["pca9685_1"][0]
        self.pca_address = int(board_cfg["address"], 0)
        self.ref_clock = board_cfg["reference_clock_speed"]
        self.frequency = board_cfg["frequency"]

        # Mapear servos del PCA9685 #1
        self.servo_details = {}
        servos = config["motion_controller"][0]["servos"][0]
        for name, data in servos.items():
            d = data[0]
            if d["pca9685"] == 1:
                self.servo_details[name] = {
                    "channel": d["channel"],
                    "min_pulse": d["min_pulse"],
                    "max_pulse": d["max_pulse"]
                }

        # Inicializar PCA9685
        i2c = busio.I2C(SCL, SDA)
        self.pca = PCA9685(i2c, address=self.pca_address, reference_clock_speed=self.ref_clock)
        self.pca.frequency = self.frequency

        # Crear objetos servo
        self.servos = {}
        for name, cfg in self.servo_details.items():
            s = servo.Servo(self.pca.channels[cfg["channel"]])
            s.set_pulse_width_range(min_pulse=cfg["min_pulse"], max_pulse=cfg["max_pulse"])
            self.servos[name] = s

        # Orden: [FL, FR, RL, RR]
        self.matrix_order = [
            ["front_shoulder_left",   "front_shoulder_right",   "rear_shoulder_left",   "rear_shoulder_right"],
            ["front_leg_left",        "front_leg_right",        "rear_leg_left",        "rear_leg_right"],
            ["front_feet_left",       "front_feet_right",       "rear_feet_left",       "rear_feet_right"]
        ]

    def enable_servo_power(self):
        """Activa la alimentación de los servos (GPIO.LOW)."""
        GPIO.output(SERVO_POWER_GPIO, GPIO.LOW)
        time.sleep(0.5)  # Tiempo para que la fuente se estabilice

    def disable_servo_power(self):
        """Corta la alimentación de los servos (GPIO.HIGH)."""
        GPIO.output(SERVO_POWER_GPIO, GPIO.HIGH)

    def set_legs_from_matrix(self, angles_3x4):
        """Aplica ángulos a los servos."""
        for i in range(3):
            for j in range(4):
                name = self.matrix_order[i][j]
                if name in self.servos:
                    self.servos[name].angle = angles_3x4[i][j]

    def deinit(self):
        """Limpieza segura."""
        self.disable_servo_power()  # ¡Primero cortar energía!
        self.pca.deinit()
        GPIO.cleanup()  # Opcional: limpia solo el GPIO usado

# === PRUEBA SEGURA ===
if __name__ == "__main__":
    # Matriz de prueba: [hombro, pierna, pie] x [FL, FR, RL, RR]
    test_angles = [
        [90, 90, 90, 90],      # Hombros centrados
        [25, 165, 25, 155],    # Piernas: traseras extendidas
        [145, 25, 155, 25]     # Pies nivelados
    ]

    controller = None
    try:
        print("🔌 Inicializando controlador seguro...")
        controller = SafeServoController()

        print("⚡ Activando alimentación de servos...")
        controller.enable_servo_power()

        print("🦿 Aplicando ángulos de prueba...")
        controller.set_legs_from_matrix(test_angles)
        time.sleep(3)

        print("✅ Prueba completada. Robot en posición.")

    except KeyboardInterrupt:
        print("\n⏹️  Interrupción por usuario.")
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        print("🔌 Cortando alimentación de servos...")
        if controller:
            controller.deinit()
        print("👋 ¡Listo! Servos apagados y recursos liberados.")