import time
import board
import busio
from adafruit_pca9685 import PCA9685

# Inicializar I2C y PCA9685
i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 50  # 50 Hz para servos estándar

# Función para mapear ángulo (0–180°) a pulso (150–600 aprox.)
def set_servo_angle(channel, angle):
    min_pulse = 500   # 0.5 ms → 0°
    max_pulse = 2500  # 2.5 ms → 180°
    pulse_range = max_pulse - min_pulse
    pulse = min_pulse + (pulse_range * angle / 180.0)
    pca.channels[channel].duty_cycle = int(pulse * 0xFFFF / 20000)  # 20000 µs = período a 50 Hz

# Ejemplo: mover servo en canal 0 a 90°
shoulder = 90
femur = 15
foot = 25
# Leg FL
set_servo_angle(12, shoulder + 5)
set_servo_angle(13, femur)
set_servo_angle(14, 170 - foot)

# Leg FR
set_servo_angle(0, shoulder - 5)
set_servo_angle(1, 180 - femur)
set_servo_angle(2, foot)

# Leg RL
set_servo_angle(8, 170 - shoulder)
set_servo_angle(9, femur + 10)
set_servo_angle(10, 180 - foot)

# Leg RR
set_servo_angle(4, 180 - shoulder)
set_servo_angle(5, 170 - femur)
set_servo_angle(6, foot)
# # time.sleep(1)
# set_servo_angle(0, 0)