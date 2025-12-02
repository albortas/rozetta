import numpy as np

# NOTE: Code snippets from Modern Robotics at Northwestern University:
# See https://github.com/NxRLab/ModernRobotics


def RpToTrans(R, p):
    """
    Convierte una matriz de rotación y un vector de posición
    en una matriz de transformación homogénea.

    :param R: Matriz de rotación 3x3
    :param p: Un 3-vector
    :return: Matriz de transformación homogénea correspondiente a la entradas.

    Ejemplo de entrada:
        R = np.array([[1, 0,  0],
                      [0, 0, -1],
                      [0, 1,  0]])
        p = np.array([1, 2, 5])

    Salida:
        np.array([[1, 0,  0, 1],
                  [0, 0, -1, 2],
                  [0, 1,  0, 5],
                  [0, 0,  0, 1]])
    """
    return np.r_[np.c_[R, p], [[0, 0, 0, 1]]]


def TransToRp(T):
    """
    Convierte una matriz de transformación homogénea
    en una matriz de rotación y un vector de posición.

    :param T: Matriz de transformación homogénea.
    :return R: La matriz de rotación correspondiente.
    :return p: El vector de posición correspondiente.

    Ejemplo de entrada:
        T = np.array([[1, 0,  0, 0],
                      [0, 0, -1, 0],
                      [0, 1,  0, 3],
                      [0, 0,  0, 1]])

    Salida:
        (np.array([[1, 0,  0],
                   [0, 0, -1],
                   [0, 1,  0]]),
         np.array([0, 0, 3]))
    """
    T = np.array(T)
    return T[0:3, 0:3], T[0:3, 3]


def TransInv(T):
    """
    Invierte una matriz de transformación homogénea.

    :param T: Una matriz de transformación homogénea.
    :return: La inversa de T.
    Utiliza la estructura de las matrices de transformación para evitar
    tomar la inversa de una matriz, por eficiencia.

    Ejemplo de entrada:
        T = np.array([[1, 0,  0, 0],
                      [0, 0, -1, 0],
                      [0, 1,  0, 3],
                      [0, 0,  0, 1]])
    Salida:
        np.array([[1,  0, 0,  0],
                  [0,  0, 1, -3],
                  [0, -1, 0,  0],
                  [0,  0, 0,  1]])
    """
    R, p = TransToRp(T)
    Rt = np.array(R).T
    return np.r_[np.c_[Rt, -np.dot(Rt, p)], [[0, 0, 0, 1]]]


def Adjoint(T):
    """
    Calcula la representación adjunta de una matriz de transformación
    homogénea.

    :param T: Una matriz de transformación homogénea.
    :return: La representación adjunta 6x6 [AdT] de T.

    Ejemplo de entrada:
        T = np.array([[1, 0,  0, 0],
                      [0, 0, -1, 0],
                      [0, 1,  0, 3],
                      [0, 0,  0, 1]])
    Salida:
        np.array([[1, 0,  0, 0, 0,  0],
                  [0, 0, -1, 0, 0,  0],
                  [0, 1,  0, 0, 0,  0],
                  [0, 0,  3, 1, 0,  0],
                  [3, 0,  0, 0, 0, -1],
                  [0, 0,  0, 0, 1,  0]])
    """
    R, p = TransToRp(T)
    return np.r_[np.c_[R, np.zeros((3, 3))], np.c_[np.dot(VecToso3(p), R), R]]


def VecToso3(omg):
    """
    Convierte un 3-vector en una representación so(3)
    :param omg: Un 3-vector
    :return: La representación simétrica sesgada de omg

    Ejemplo de entrada:
        omg = np.array([1, 2, 3])
    Salida:
        np.array([[ 0, -3,  2],
                  [ 3,  0, -1],
                  [-2,  1,  0]])
    """
    return np.array([[0, -omg[2], omg[1]], [omg[2], 0, -omg[0]], [-omg[1], omg[0], 0]])


def RPY(roll, pitch, yaw):
    """
    Crea una matriz de transformación de roll, pitch, yaw.

    :param roll: Componente de balanceo(roll) de la matriz.
    :param pitch: Componente de cabeceo(pitch) de la matriz.
    :param yaw: Componente de guiñada(yaw) de la matriz.
    :return: Matriz de transformación.

    Ejemplo de entrada:
        roll = 0.0
        pitch = 0.0
        yaw = 0.0
    Salida:
        np.array([[1, 0, 0, 0],
                  [0, 1, 0, 0],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]])
    """
    Roll = np.array(
        [
            [1, 0, 0, 0],
            [0, np.cos(roll), -np.sin(roll), 0],
            [0, np.sin(roll), np.cos(roll), 0],
            [0, 0, 0, 1],
        ]
    )
    Pitch = np.array(
        [
            [np.cos(pitch), 0, np.sin(pitch), 0],
            [0, 1, 0, 0],
            [-np.sin(pitch), 0, np.cos(pitch), 0],
            [0, 0, 0, 1],
        ]
    )
    Yaw = np.array(
        [
            [np.cos(yaw), -np.sin(yaw), 0, 0],
            [np.sin(yaw), np.cos(yaw), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ]
    )
    return np.matmul(np.matmul(Roll, Pitch), Yaw)


def RotateTranslate(rotation, position):
    """
    Crea una matriz de transformación a partir de una rotación y,
    luego, una traslación.

    :param rotation: Matriz de rotación pura.
    :param translation: Matriz de traslación pura.
    :return: Matriz de transfomación.
    """
    trans = np.eye(4)
    trans[0, 3] = position[0]
    trans[1, 3] = position[1]
    trans[2, 3] = position[2]

    return np.dot(rotation, trans)


def TransformVector(xyz_coord, rotation, translation):
    """
    Transforma un vector mediante una rotación especificada
    y luego una matriz de traslación.

    :param xyz_coord: El vector a transformar.
    :param rotation: Matriz de rotación pura.
    :param translation: Matriz de traslación pura.
    :return: El vector transformado.
    """
    xyz_vec = np.append(xyz_coord, 1.0)

    Transformed = np.dot(RotateTranslate(rotation, translation), xyz_vec)
    return Transformed[:3]
