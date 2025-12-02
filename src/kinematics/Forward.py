import numpy as np


class Forward:
    def __init__(self, legtype="LEFT", L1=0.0685, L2=0.10805, L3=0.1385):
        self.legtype = legtype
        self.L1 = L1
        self.L2 = L2
        self.L3 = L3

    def solve(self, theta):
        if self.legtype == "LEFT":
            return self.ForwardKinematic(theta, side=1)
        else:
            return self.ForwardKinematic(theta, side=-1)

    def ForwardKinematic(self, theta, side):
        """Posicion del los sistemas de coordenadas cinemática directa"""

        # Sistema de coordenadas S_1
        p1 = np.array(
            [
                0,
                side * self.L1 * np.cos(theta[0]),
                side * self.L1 * np.sin(theta[0]),
            ]
        )

        # Sistema de coordenadas S_2
        p2 = np.array(
            [
                -self.L2 * np.sin(theta[1]),
                side * self.L1 * np.cos(theta[0])
                + self.L2 * np.sin(theta[0]) * np.cos(theta[1]),
                side * self.L1 * np.sin(theta[0])
                - self.L2 * np.cos(theta[0]) * np.cos(theta[1]),
            ]
        )

        # Sistema de coordenadas S_2
        p3 = np.array(
            [
                (-self.L2 * np.sin(theta[1]) - self.L3 * np.sin(theta[1] + theta[2])),
                (
                    side * self.L1 * np.cos(theta[0])
                    + self.L2 * np.sin(theta[0]) * np.cos(theta[1])
                    + self.L3 * np.sin(theta[0]) * np.cos(theta[1] + theta[2])
                ),
                (
                    side * self.L1 * np.sin(theta[0])
                    - self.L2 * np.cos(theta[0]) * np.cos(theta[1])
                    - self.L3 * np.cos(theta[0]) * np.cos(theta[1] + theta[2])
                ),
            ]
        )

        return p1, p2, p3


if __name__ == "__main__":
    import matplotlib.pyplot as plt
    from matplotlib.widgets import Slider

    fk = Forward(legtype="RIGHT")

    # Valores iniciales
    theta0, theta1, theta2 = 0, 0, 0

    # Crear figura y eje 3D
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")
    plt.subplots_adjust(bottom=0.3)

    # Graficar robot iniciales
    theta = [theta0, theta1, theta2]
    p0 = np.array([0, 0, 0])
    p1, p2, p3 = fk.solve(theta)
    puntos = np.array([p0, p1, p2, p3])
    articulaciones = np.array([p0, p1, p2, p3])

    (line,) = ax.plot(puntos[:, 0], puntos[:, 1], puntos[:, 2], "-b", linewidth=2)
    points = ax.scatter(
        articulaciones[:, 0],
        articulaciones[:, 1],
        articulaciones[:, 2],
        color="red",
        s=80,
    )

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("Robot - usa los sliders")

    # Ajustar límites iniciales
    max_range = 0.25
    ax.set_xlim(-max_range, max_range)
    ax.set_ylim(-max_range, max_range)
    ax.set_zlim(-max_range, max_range)

    # Crear ejes para sliders
    ax_theta0 = plt.axes([0.2, 0.2, 0.5, 0.03])
    ax_theta1 = plt.axes([0.2, 0.15, 0.5, 0.03])
    ax_theta2 = plt.axes([0.2, 0.1, 0.5, 0.03])

    s_theta0 = Slider(ax_theta0, "θ0 (rad)", -np.pi / 2, np.pi / 2, valinit=theta0)
    s_theta1 = Slider(ax_theta1, "θ1 (rad)", -np.pi / 2, np.pi / 2, valinit=theta1)
    s_theta2 = Slider(ax_theta2, "θ2 (rad)", -np.pi / 2, np.pi / 2, valinit=theta2)

    def update(val):
        th0 = s_theta0.val
        th1 = s_theta1.val
        th2 = s_theta2.val
        theta = [th0, th1, th2]
        p1, p2, p3 = fk.solve(theta)
        print(p3)
        puntos = np.array([p0, p1, p2, p3])
        articulaciones = np.array([p0, p1, p2, p3])

        line.set_data_3d(puntos[:, 0], puntos[:, 1], puntos[:, 2])
        points._offsets3d = (
            articulaciones[:, 0],
            articulaciones[:, 1],
            articulaciones[:, 2],
        )
        fig.canvas.draw_idle()

    s_theta0.on_changed(update)
    s_theta1.on_changed(update)
    s_theta2.on_changed(update)

    plt.show()
