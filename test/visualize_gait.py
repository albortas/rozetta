#!/usr/bin/env python3
"""
Visualizador 3D de la marcha de Bézier para tu robot cuadrúpedo.
Compatible con tu BezierGait (FL, FR, RL, RR) y RobotModel.
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
import copy
import sys
import os

# Asegurar que el módulo src está accesible
sys.path.append(os.path.expanduser("~/robot"))

from src.kinematics.RobotModel import RobotModel

# Importar BezierGait: asegúrate de que esté en PYTHONPATH o en el mismo directorio
try:
    from src.gait.bezier_gait import BezierGait
except ImportError:
    # Si está en src/gait/, descomenta la siguiente línea
    # sys.path.append(os.path.expanduser("~/robot/src/gait"))
    from src.gait.bezier_gait import BezierGait


class GaitVisualizer:
    def __init__(self,
                 Tswing=0.25,
                 L=0.08,              # mitad de la longitud del paso (adelante/atrás)
                 LateralFraction=0.0, # 0.0 = puro X, >0 = deriva lateral
                 YawRate=0.0,         # rad/s (positivo = giro antihorario)
                 vel=0.3,             # m/s
                 clearance_height=0.05,
                 penetration_depth=0.01,
                 dt=0.02):
        
        self.robot = RobotModel()
        self.gait = BezierGait(Tswing=Tswing, dt=dt)
        self.dt = dt

        # Parámetros de marcha
        self.L = L
        self.LateralFraction = LateralFraction
        self.YawRate = YawRate
        self.vel = vel
        self.clearance_height = clearance_height
        self.penetration_depth = penetration_depth

        # Estado inicial: usar postura neutral de RobotModel
        self.T_bf = copy.deepcopy(self.robot.WorldToFoot)
        self.time = 0.0

        # Orden fijo de patas (¡crucial para fases correctas!)
        self.leg_order = ["FL", "FR", "RL", "RR"]
        self.colors = {
            "FL": "red",
            "FR": "blue",
            "RL": "green",
            "RR": "orange"
        }
        self.swing_colors = {
            "FL": "salmon",
            "FR": "skyblue",
            "RL": "lightgreen",
            "RR": "gold"
        }

        # Historial de trayectorias
        self.trajectories = {leg: [] for leg in self.leg_order}
        self.swing_flags = {leg: [] for leg in self.leg_order}

        # Figura
        self.fig = plt.figure(figsize=(12, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_xlim([-0.3, 0.3])
        self.ax.set_ylim([-0.3, 0.3])
        self.ax.set_zlim([-0.3, 0.2])
        self.ax.set_xlabel('X (adelante)')
        self.ax.set_ylabel('Y (izquierda)')
        self.ax.set_zlabel('Z (arriba)')

    def step(self):
        """Genera un paso de la marcha y actualiza las posiciones de los pies."""
        # Simulamos contacto constante (puedes extender con sensores reales)
        contacts = [1, 1, 1, 1]

        # Aseguramos que T_bf_ tenga el orden correcto (aunque sea dict)
        T_bf_ordered = {leg: self.T_bf[leg] for leg in self.leg_order}

        # Generar nueva trayectoria
        T_bf_next = self.gait.GenerateTrajectory(
            L=self.L,
            LateralFraction=self.LateralFraction,
            YawRate=self.YawRate,
            vel=self.vel,
            T_bf_=T_bf_ordered,
            T_bf_curr=T_bf_ordered,
            clearance_height=self.clearance_height,
            penetration_depth=self.penetration_depth,
            contacts=contacts,
            dt=self.dt
        )

        # Actualizar estado
        self.T_bf = T_bf_next
        self.time += self.dt

        # Guardar historial
        for leg in self.leg_order:
            p = self.T_bf[leg][:3, 3]
            self.trajectories[leg].append(p.copy())

            # Determinar modo actual (swing o stance)
            Tstance = 2.0 * abs(self.L) / (abs(self.vel) + 1e-6) if self.vel != 0 else 0.0
            _, mode = self.gait.GetPhase(
                index=self.leg_order.index(leg),
                Tstance=Tstance,
                Tswing=self.gait.Tswing
            )
            self.swing_flags[leg].append(mode == self.gait.SWING)

    def animate(self, frame):
        self.ax.clear()
        self.ax.set_xlim([-0.3, 0.3])
        self.ax.set_ylim([-0.3, 0.3])
        self.ax.set_zlim([-0.3, 0.2])
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.set_title(f'Marcha de Bézier - t = {self.time:.2f}s')

        # Avanzar simulación
        self.step()

        # Dibujar cuerpo (origen)
        self.ax.scatter([0], [0], [0], color='black', s=100, label='Cuerpo')

        # Dibujar trayectorias y pies actuales
        for leg in self.leg_order:
            traj = np.array(self.trajectories[leg])
            if len(traj) > 1:
                # Dibujar segmentos con color según modo
                for i in range(1, len(traj)):
                    color = self.swing_colors[leg] if self.swing_flags[leg][i] else self.colors[leg]
                    self.ax.plot(
                        traj[i-1:i+1, 0],
                        traj[i-1:i+1, 1],
                        traj[i-1:i+1, 2],
                        color=color,
                        linewidth=1.5
                    )
            # Pie actual
            p = self.T_bf[leg][:3, 3]
            current_mode = 'swing' if self.swing_flags[leg][-1] else 'stance'
            color = self.swing_colors[leg] if current_mode == 'swing' else self.colors[leg]
            self.ax.scatter([p[0]], [p[1]], [p[2]], color=color, s=50)

        # Leyenda manual
        self.ax.scatter([], [], color='red', s=50, label='FL (stance)')
        self.ax.scatter([], [], color='salmon', s=50, label='FL (swing)')
        self.ax.scatter([], [], color='blue', s=50, label='FR (stance)')
        self.ax.scatter([], [], color='skyblue', s=50, label='FR (swing)')
        self.ax.legend(loc='upper left', fontsize=7)

    def run(self, duration=6.0):
        frames = int(duration / self.dt)
        anim = FuncAnimation(self.fig, self.animate, frames=frames, interval=self.dt * 1000, repeat=False)
        plt.show()


if __name__ == "__main__":
    visualizer = GaitVisualizer(
        Tswing=0.25,
        L=0.08,              # longitud de paso (total = 2*L)
        LateralFraction=0.0, # 0.0 = recto, 0.3 = ligera deriva
        YawRate=0.5,         # giro antihorario
        vel=0.4,             # m/s
        clearance_height=0.05,
        penetration_depth=0.005,
        dt=0.02              # más rápido para visualización
    )
    visualizer.run(duration=8.0)