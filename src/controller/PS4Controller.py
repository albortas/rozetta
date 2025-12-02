import pygame
import time
from math import fabs


class PS4Controller:
    def __init__(self):
        pygame.init()
        # Verificar si hay un controlador conectado
        if pygame.joystick.get_count() == 0:
            raise Exception("No se detectó ningún controlador PS4.")
        # Inicialar si hay un controlador
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        print(f"Controlador PS4 instancia: {self.joystick.get_instance_id()}")
        print(f"Controlador PS4 conectado: {self.joystick.get_name()}")

        # Estado inicial
        self.num_axes = self.joystick.get_numaxes()
        self.num_buttons = self.joystick.get_numbuttons()
        self.target_joyaxes = [0.0] * self.num_axes
        self.last_joyaxes = [0.0] * self.num_axes
        self.joybut = [False] * self.num_buttons
        self.last_send_time = time.time()
        # Configuración de velocidad
        self.speed_index = 2
        self.available_speeds = [0.5, 1.0, 3.0, 4.0]

    def ramped_vel(self, v_prev, v_target, t_prev, t_now):
        step = t_now - t_prev
        sign = (
            self.available_speeds[self.speed_index]
            if (v_target > v_prev)
            else -self.available_speeds[self.speed_index]
        )
        error = fabs(v_target - v_prev)

        if error < self.available_speeds[self.speed_index] * step:
            return v_target
        else:
            return v_prev + sign * step

    def get_joystick_state(self):
        pygame.event.pump()
        self.joybut = [self.joystick.get_button(i) for i in range(self.num_buttons)]
        now = time.time()
        processed_axes = []
        # Aplicar rampa a los ejesx
        for i in range(self.num_axes):
            raw = self.joystick.get_axis(i)
            target = raw if abs(raw) > 0.1 else 0.0  # Zona muerta
            if target == self.last_joyaxes[i]:
                processed_axes.append(self.last_joyaxes[i])
            else:
                processed_axes.append(
                    self.ramped_vel(
                        self.last_joyaxes[i], target, self.last_send_time, now
                    )
                )
        # Actualizar el estado
        self.last_joyaxes = processed_axes
        self.last_send_time = now
        return {"axes": self.last_joyaxes, "buttons": self.joybut}


if __name__ == "__main__":
    try:
        ps4 = PS4Controller()
        while True:
            pygame.event.get()
            f = ps4.get_joystick_state()
            print(f)
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("\nPrograma detenido por el usuario.")
    except Exception as e:
        print(f"Programa detenido: {e}, {type(e)}")
    finally:
        pygame.quit()
