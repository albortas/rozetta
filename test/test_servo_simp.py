import busio
from board import SCL, SDA
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

# Configuración inicial
i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c)
pca.frequency = 50

# Configurar el servo en el canal 0
mi_servo = servo.Servo(pca.channels[8], min_pulse=500, max_pulse=2500)

# Diccionario de ángulos fijos vinculados a una tecla
posiciones = {
    "1": 0,
    "2": 90,
    "3": 180
}

print("--- Control por Teclas Rápidas ---")
print("Presiona: [1] para 0° | [2] para 90° | [3] para 180°")
print("Escribe 'salir' para terminar.")

try:
    while True:
        tecla = input("\nSelecciona posición (1, 2, 3): ").strip().lower()

        if tecla == 'salir':
            break

        if tecla in posiciones:
            angulo = posiciones[tecla]
            print(f"-> Moviendo a {angulo} grados")
            mi_servo.angle = angulo
        else:
            print("⚠️ Tecla no válida. Usa 1, 2 o 3.")

except KeyboardInterrupt:
    pass

finally:
    mi_servo.angle = None # Relajar el servo
    pca.deinit()
    print("\nPrograma finalizado.")

