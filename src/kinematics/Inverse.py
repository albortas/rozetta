import numpy as np


class Inverse:
    def __init__(self, legtype="LEFT", L1=0.058, L2=0.10805, L3=0.1385):
        self.legtype = legtype
        self.L1 = L1
        self.L2 = L2
        self.L3 = L3

    def get_domain(self, x, y, z):
        """Calcula el dominio de la pierna y lo limita en caso de incumplimiento

        :param x, y, z: Distancia de cadera a pie en cada dimension
        :return: Dominio de la pierna D.
        """
        D = (y**2 + z**2 - self.L1**2 + x**2 - self.L2**2 - self.L3**2) / (
            2 * self.L2 * self.L3
        )
        if D > 1 or D < -1:
            # Dominio Invalido
            print("----- DOMINIO INVALIDO -----")
            D = np.clip(D, -1.0, 1.0)
            return D
        else:
            return D

    def solve(self, xyz_coord):
        """Solucionador generico de cinematica inversa de piernas

        Args:
            xyz_coord (m): Distancia de cadera a pie de cada dimension
            return: Angulos articulares necesarios para la posición deseada
        """
        x = xyz_coord[0]
        y = xyz_coord[1]
        z = xyz_coord[2]
        D = self.get_domain(x, y, z)
        if self.legtype == "LEFT":
            return self.InverseKinematic(x, y, z, D, side=1)
        else:
            return self.InverseKinematic(x, y, z, D, side=-1)

    def InverseKinematic(self, x, y, z, D, side):
        """Solucionador de cinematica inversa de pierna izquierda

        Args:
            x, y, z (m): Distancia de cadera a pie en cada dimension
            D : Dominio de la pierna
        Return: Angulos articulares necesarios para la posicion deseada
        """
        AG_cuadrado = y**2 + z**2 - self.L1**2
        if AG_cuadrado < 0.0:
            print("SQRT NEGATIVO")
            AG_cuadrado = 0.0
        AG = np.sqrt(AG_cuadrado)
        theta1 = - np.arctan2(z, y) - np.arctan2(AG, side * self.L1)
        theta3 = np.arccos(D)
        theta2 = np.arctan2(x,AG) - np.arctan2(
            self.L3 * np.sin(theta3), self.L2 + self.L3 * np.cos(theta3)
        )
        return np.array([-theta1, -theta2, -theta3])
    
if __name__ == "__main__":
    np.set_printoptions(precision=2, suppress=True)
    ik = Inverse(legtype="R")
    position = [0, -0.0685, -0.24655]
    theta = np.degrees(ik.solve(position))
    print(theta)