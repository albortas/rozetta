import numpy as np
from src.kinematics.Inverse import Inverse
from src.kinematics.LieAlgebra import (
    RpToTrans,
    TransToRp,
    TransInv,
    RPY,
    TransformVector,
)
from collections import OrderedDict


class RobotModel:
    def __init__(
        self,
        L1=0.058,
        L2=0.10805,
        L3=0.1385,
        hip_x=0.192,
        hip_y=0.078,
        foot_x=0.192,
        foot_y=0.194,
        height=0.135,
    ):
        """
        Cinematica
        """

        # Parametros pierna
        self.L1 = L1
        self.L2 = L2
        self.L3 = L3

        # Vector de piernas posiciones_deseadas

        # Distancia entre caderas
        # Longitud
        self.hip_x = hip_x
        # Ancho
        self.hip_y = hip_y

        # Distancia entre pies
        # Longitud
        self.foot_x = foot_x
        # Ancho
        self.foot_y = foot_y

        # Altura del cuerpo
        self.height = height

        # Diccionario para almacenar solucionadores de Cinematica Inversa
        self.Legs = OrderedDict()
        self.Legs["FL"] = Inverse("LEFT", self.L1, self.L2, self.L3)
        self.Legs["FR"] = Inverse("RIGHT", self.L1, self.L2, self.L3)
        self.Legs["RL"] = Inverse("LEFT", self.L1, self.L2, self.L3)
        self.Legs["RR"] = Inverse("RIGHT", self.L1, self.L2, self.L3)

        # Diccionario para almacenar transformaciones de cadera y pie

        # Transformación de la cadera en relación con el marco mundial
        # Con el centroide del cuerpo en el marco mundial
        Rwb = np.eye(3)  # Matriz de identidad
        self.WorldToHip = OrderedDict()

        self.ph_FL = np.array([self.hip_x / 2.0, self.hip_y / 2.0, 0])
        self.WorldToHip["FL"] = RpToTrans(Rwb, self.ph_FL)

        self.ph_FR = np.array([self.hip_x / 2.0, -self.hip_y / 2.0, 0])
        self.WorldToHip["FR"] = RpToTrans(Rwb, self.ph_FR)

        self.ph_RL = np.array([-self.hip_x / 2.0, self.hip_y / 2.0, 0])
        self.WorldToHip["RL"] = RpToTrans(Rwb, self.ph_RL)

        self.ph_RR = np.array([-self.hip_x / 2.0, -self.hip_y / 2.0, 0])
        self.WorldToHip["RR"] = RpToTrans(Rwb, self.ph_RR)

        # Transformación del pie en relación con el marco mundial
        # Con el centroide del cuerpo en el marco mundial
        self.WorldToFoot = OrderedDict()

        self.pf_FL = np.array([self.foot_x / 2.0, self.foot_y / 2.0, -self.height])
        self.WorldToFoot["FL"] = RpToTrans(Rwb, self.pf_FL)

        self.pf_FR = np.array([self.foot_x / 2.0, -self.foot_y / 2.0, -self.height])
        self.WorldToFoot["FR"] = RpToTrans(Rwb, self.pf_FR)

        self.pf_RL = np.array([-self.foot_x / 2.0, self.foot_y / 2.0, -self.height])
        self.WorldToFoot["RL"] = RpToTrans(Rwb, self.pf_RL)

        self.pf_RR = np.array([-self.foot_x / 2.0, -self.foot_y / 2.0, -self.height])
        self.WorldToFoot["RR"] = RpToTrans(Rwb, self.pf_RR)

    def HipToFoot(self, orn, pos, T_bf):
        """
        Convierte la posición y orientación deseadas respecto a la posición
        inicial de Robot, con la transformación deseada de cuerpo a pie,
        en una transformación de cuerpo a cadera, que se utiliza para extraer
        y devolver el vector de cadera a pie

        :param orn: Un 3x1 np.array([]) con ángulos Roll, Pitch, Yaw del Robot
        :param pos: Un 3x1 np.array([]) con las coordenada X, Y, Z del Robot
        :param T_bf: Diccionario de las transformaciones deseadas de cuerpo a pie
        :return: Vector de cadera a pie para cada pierna
        """

        # Solo obtener el componete Rot
        Rb, _ = TransToRp(RPY(orn[0], orn[1], orn[2]))
        pb = pos
        T_wb = RpToTrans(Rb, pb)

        # Diccionario para almacenar vectores
        HipToFoot_List = OrderedDict()

        for i, (key, T_wh) in enumerate(self.WorldToHip.items()):
            # ORDER: FL, FR, RL, RR

            # Extraer un componente vectorial
            _, p_bf = TransToRp(T_bf[key])

            # Paso 1, obtener T_bh para cada pierna
            T_bh = np.dot(TransInv(T_wb), T_wh)

            # Paso 2, obtener T_hf para cada pierna

            # MÉTODO DE ADICIÓN DE VECTORES
            _, p_bh = TransToRp(T_bh)
            p_hf0 = p_bf - p_bh

            # MÉTODO DE TRANSFORMACIÓN
            T_hf = np.dot(TransInv(T_bh), T_bf[key])
            _, p_hf1 = TransToRp(T_hf)

            # Deberián producir el mismo resultado
            if p_hf1.all() != p_hf0.all():
                print("NOT EQUAL")

            p_hf = p_hf1

            HipToFoot_List[key] = p_hf

        return HipToFoot_List
    
    def IK(self, orn, pos, T_bf):
        """
        Utiliza HipToFoot() para convertir la posión y
        orientación deseadas respecto a la posión inicial del Robot
        en un vector de Cadera a Pie, que se introduce en el solucionador
        de la Cinematica Inversa.

        Finalmente, el solucionador de Cinematica Inversa devuelve los
        ángulos de la articulación resultante para cada pierna.

        :param orn: Un 3x1 np.array([]) con los ángulos de Roll, Pitch, Yaw del Robot.
        :param pos: Un 3x1 np.array([]) con las coordenadas X, Y, Z del Robot.
        :param T_bf: Diccionario de las transformaciones deseadas del Cuerpo a Pie.
        :return: Ángulos de articulación para cada articulación del Robot.
        """

        # 4 piernas, 3 articulaciones por pierna
        joint_angles = np.zeros((4, 3))

        HipToFoot = self.HipToFoot(orn, pos, T_bf)

        for i, (key, p_hf) in enumerate(HipToFoot.items()):
            # ORDER: FL, FR, RL, RR

            # print("LEG: {} \t HipToFoot: {}".format(key, p_hf))

            # Step 3, compute joint angles from T_hf for each leg
            joint_angles[i, :] = self.Legs[key].solve(p_hf)

        # print("-----------------------------")

        return joint_angles
