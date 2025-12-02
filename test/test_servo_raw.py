# ~/robot/src/tests/test_servos_raw.py

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), ".."))

from src.hardware.ServoController import ServoController

def main():
    controller = ServoController()
    servo_names = sorted(controller.servos.keys())
    
    try:
        while True:
            print("\nServos:")
            for i, name in enumerate(servo_names):
                print(f"{i+1:2}. {name}")
            choice = input("Elige servo (número) o 'q': ").strip()
            if choice == "q": break
            try:
                name = servo_names[int(choice)-1]
                angle = float(input(f"Ángulo para {name} (0-180): "))
                controller.set_servo_angle(name, angle)  # si usas IK centrada en 0
                print(f"✅ {name} = {angle}°")
            except (ValueError, IndexError):
                print("❌ Entrada inválida.")
    finally:
        controller.deinit()

if __name__ == "__main__":
    main()