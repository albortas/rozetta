# ~/robot/src/hardware/servo_controller.py

import json
import os
import busio
from board import SCL, SDA
import digitalio
import board
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

class ServoController:
    def __init__(self, config_dir="~/robot/src/config"):
        config_dir = os.path.expanduser(config_dir)
        self._load_configs(config_dir)
        self._init_abort_gpio()
        self._init_pca9685()
        self._init_servos()
        self.enable_servos() #Habilitar al iniciar

    def _load_configs(self, config_dir):
        def load_json(name):
            with open(os.path.join(config_dir, name), 'r') as f:
                return json.load(f)
        self.robot_cfg = load_json("robot.json")
        self.calib_cfg = load_json("servo_calibration.json")
        self.offsets = load_json("offsets.json")
    
    def _init_abort_gpio(self):
        # Leer GPIO de abort del config
        abort_cfg = self.robot_cfg.get("abort_controller", [{}])[0]
        gpio_num = abort_cfg.get("gpio_port", 17)
        pin_name = f"D{gpio_num}"
        self.abort_pin = digitalio.DigitalInOut(getattr(board, pin_name))
        self.abort_pin.direction = digitalio.Direction.OUTPUT
        self.abort_pin.value = False  # OE = LOW → habilitado

    def _init_pca9685(self):
        board = self.robot_cfg["motion_controller"][0]["boards"][0]["pca9685_1"][0]
        i2c = busio.I2C(SCL, SDA)
        self.pca = PCA9685(i2c, address=int(board["address"], 0))
        self.pca.frequency = board["frequency"]

    def _init_servos(self):
        self.servos = {}
        servos = self.robot_cfg["motion_controller"][0]["servos"][0]
        for name, data in servos.items():
            d = data[0]
            if d["pca9685"] == 1:
                s = servo.Servo(self.pca.channels[d["channel"]])
                s.set_pulse_width_range(min_pulse=d["min_pulse"], max_pulse=d["max_pulse"])
                self.servos[name] = s

    def enable_servos(self):
        """Habilita los servos (OE = LOW)"""
        self.abort_pin.value = False
        print("[Servos] HABILITADOS")

    def disable_servos(self):
        """¡EMERGENCIA! Deshabilita los servos (OE = HIGH)"""
        self.abort_pin.value = True
        print("[EMERGENCIA] Servos DESHABILITADOS")

    def set_servo_angle(self, name, logical_angle_deg):
        """Envía un ángulo lógico (grados) al servo físico."""
        # Logica existente (solo si están habilitados)
        if self.abort_pin.value:
            return  # ignorar si deshabilitado
        cal = self.calib_cfg[name]
        offset = self.offsets.get(name, 0.0)
        base = cal["zero_angle"] + logical_angle_deg + offset
        physical = 180.0 - base if cal["invert_direction"] else base
        physical = max(0, min(180, physical))
        self.servos[name].angle = physical

    def set_leg_angles(self, leg_id, angles_deg):
        """Envía [roll, pitch, knee] a una pata. Usa nombres: FL, FR, RL, RR."""
        LEG_MAP = {
            "FL": ["FL_hip_roll", "FL_hip_pitch", "FL_knee"],
            "FR": ["FR_hip_roll", "FR_hip_pitch", "FR_knee"],
            "RL": ["RL_hip_roll", "RL_hip_pitch", "RL_knee"],
            "RR": ["RR_hip_roll", "RR_hip_pitch", "RR_knee"]
        }
        for name, angle in zip(LEG_MAP[leg_id], angles_deg):
            self.set_servo_angle(name, angle)

    def deinit(self):
        self.disable_servos() # ¡Seguridad!
        self.pca.deinit()
