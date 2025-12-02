import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.widgets import Slider

from src.kinematics.Forward import Forward
from src.kinematics.Inverse import Inverse

ik = Inverse(legtype="R")
fk = Forward(legtype="R")

# === CONFIGURACIÓN INICIAL ===
x0, y0, z0 = 0.0, 0.0685, -0.24655

# Crear figura
fig = plt.figure(figsize=(10, 6))
ax3d = fig.add_subplot(121, projection='3d')
ax_err = fig.add_subplot(122)
plt.subplots_adjust(left=0.25, bottom=0.25)

# Sliders
ax_x = plt.axes([0.25, 0.15, 0.5, 0.03])
ax_y = plt.axes([0.25, 0.10, 0.5, 0.03])
ax_z = plt.axes([0.25, 0.05, 0.5, 0.03])

s_x = Slider(ax_x, 'X', -0.24, 0.24, valinit=x0)
s_y = Slider(ax_y, 'Y', -0.2, 0.2, valinit=y0)
s_z = Slider(ax_z, 'Z', -0.246, -0.09, valinit=z0)

# Función para actualizar la gráfica
def update(val):
    x = s_x.val
    y = s_y.val
    z = s_z.val
    position = [x, y, z]

    try:
        theta = ik.solve(position)
        p1, p2, p3 = fk.solve(theta)
        x_fk, y_fk, z_fk = p3
    except Exception as e:
        ax3d.set_title(f"Error: {str(e)}")
        fig.canvas.draw_idle()
        return

    # Limpiar ejes
    ax3d.clear()
    
    # Dibujar la pata
    xs = [0, p1[0], p2[0], p3[0]]
    ys = [0, p1[1], p2[1], p3[1]]
    zs = [0, p1[2], p2[2], p3[2]]
    ax3d.plot(xs, ys, zs, 'o-', linewidth=2, markersize=6)
    ax3d.scatter([x], [y], [z], color='red', s=50, label='Objetivo IK')
    ax3d.scatter([x_fk], [y_fk], [z_fk], color='green', s=50, label='Resultado FK')
    ax3d.set_xlim([-0.25, 0.25])
    ax3d.set_ylim([-0.25, 0.25])
    ax3d.set_zlim([-0.25, 0.25])
    ax3d.legend()
    ax3d.set_title('Cinemática 3D')

    # Error
    ax_err.clear()
    err = [x_fk - x, y_fk - y, z_fk - z]
    ax_err.bar(['X', 'Y', 'Z'], err, color=['r', 'g', 'b'])
    ax_err.set_title('Error FK - IK (m)')
    ax_err.axhline(0, color='k', linewidth=0.8)
    ax_err.set_ylim(-1, 1)

    fig.canvas.draw_idle()

s_x.on_changed(update)
s_y.on_changed(update)
s_z.on_changed(update)

update(None)
plt.show()