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

# Importar BezierGait
try:
    from src.gait.bezier_gait import BezierGait
except ImportError:
    # Si BezierGait está en otra carpeta, ajusta la ruta aquí
    # sys.path.append(os.path.expanduser("~/robot/src/gait"))
    from src.gait.bezier_gait import BezierGait


class GaitVisualizer:
    def __init__(self,
                 Tswing=0.25,
                 L=0.06,              # mitad de la longitud del paso
                 LateralFraction=0.0,
                 YawRate=0.0,
                 vel=0.2,
                 clearance_height=0.04,
                 penetration_depth=0.005,
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

        # Estado inicial
        self.T_bf = copy.deepcopy(self.robot.WorldToFoot)
        self.time = 0.0

        # Orden fijo de patas
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

        # Historial
        self.trajectories = {leg: [] for leg in self.leg_order}
        self.swing_flags = {leg: [] for leg in self.leg_order}

        # Figura
        self.fig = plt.figure(figsize=(12, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        max_range = 2
        self.ax.set_xlim([-max_range, max_range])
        self.ax.set_ylim([-max_range, max_range])
        self.ax.set_zlim([-max_range, max_range])
        self.ax.set_xlabel('X (adelante)')
        self.ax.set_ylabel('Y (izquierda)')
        self.ax.set_zlabel('Z (arriba)')

    def step(self):
        contacts = [1, 1, 1, 1]  # Simulación: siempre en contacto

        # Garantizar orden explícito
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

        self.T_bf = T_bf_next
        self.time += self.dt

        # Diagnóstico: imprimir cada 1 segundo
        if int(self.time / self.dt) % 50 == 0:
            p_fl = self.T_bf["FL"][:3, 3]
            print(f"t={self.time:.2f}s | FL pie: [{p_fl[0]:.3f}, {p_fl[1]:.3f}, {p_fl[2]:.3f}]")

        # Guardar historial
        for leg in self.leg_order:
            p = self.T_bf[leg][:3, 3]
            self.trajectories[leg].append(p.copy())

            # Determinar modo
            Tstance = 2.0 * abs(self.L) / (abs(self.vel) + 1e-6) if self.vel != 0 else 0.0
            _, mode = self.gait.GetPhase(
                index=self.leg_order.index(leg),
                Tstance=Tstance,
                Tswing=self.gait.Tswing
            )
            self.swing_flags[leg].append(mode == self.gait.SWING)

    def animate(self, frame):
        self.ax.clear()
        max_range = 2
        self.ax.set_xlim([-max_range, max_range])
        self.ax.set_ylim([-max_range, max_range])
        self.ax.set_zlim([-max_range, max_range])
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.set_title(f'Marcha de Bézier - t = {self.time:.2f}s')

        self.step()

        # Cuerpo
        self.ax.scatter([0], [0], [0], color='black', s=100, label='Cuerpo')

        # Pies y trayectorias
        for leg in self.leg_order:
            traj = np.array(self.trajectories[leg])
            if len(traj) > 1:
                for i in range(1, len(traj)):
                    color = self.swing_colors[leg] if self.swing_flags[leg][i] else self.colors[leg]
                    self.ax.plot(
                        traj[i-1:i+1, 0],
                        traj[i-1:i+1, 1],
                        traj[i-1:i+1, 2],
                        color=color,
                        linewidth=1.5
                    )
            p = self.T_bf[leg][:3, 3]
            color = self.swing_colors[leg] if self.swing_flags[leg][-1] else self.colors[leg]
            self.ax.scatter([p[0]], [p[1]], [p[2]], color=color, s=50)

        # Leyenda simplificada
        legend_elements = [
            plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='salmon', markersize=8, label='FL/RL Swing'),
            plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='red', markersize=8, label='FL/RL Stance'),
            plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='skyblue', markersize=8, label='FR/RR Swing'),
            plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='blue', markersize=8, label='FR/RR Stance'),
        ]
        self.ax.legend(handles=legend_elements, loc='upper left', fontsize=8)

    def run(self, duration=6.0):
        frames = int(duration / self.dt)
        anim = FuncAnimation(self.fig, self.animate, frames=frames, interval=self.dt * 1000, repeat=False)
        plt.show()


if __name__ == "__main__":
    visualizer = GaitVisualizer(
        Tswing=0.25,
        L=0.06,              # Paso total = 12 cm
        LateralFraction=0.0, # Recto
        YawRate=0.0,         # Sin giro
        vel=0.2,             # 0.2 m/s
        clearance_height=0.04,
        penetration_depth=0.005,
        dt=0.02
    )
    visualizer.run(duration=8.0)