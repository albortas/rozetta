import numpy as np
from scipy.special import factorial
from src.kinematics.LieAlgebra import TransToRp
import copy

# Curvas de Bézier basadas en: https://dspace.mit.edu/handle/1721.1/98270  
# Lógica de rotación basada en: http://www.inase.org/library/2014/santorini/bypaper/ROBCIRC/ROBCIRC-54.pdf


class BezierGait():
    STANCE = 0
    SWING = 1
    def __init__(self, dSref=[0.0, 0.0, 0.5, 0.5], dt=0.01, Tswing=0.2):
        # Desfase de fase por pata: FL, FR, RL, RR
        # La pata de referencia es FL, siempre 0
        self.dSref = dSref
        self.Prev_fxyz = [0.0, 0.0, 0.0, 0.0]
        # Número de puntos de control es n + 1 = 11 + 1 = 12
        self.NumControlPoints = 11
        # Paso de tiempo
        self.dt = dt

        # Tiempo total transcurrido
        self.time = 0.0
        # Tiempo de contacto con el suelo (Touchdown)
        self.TD_time = 0.0
        # Tiempo transcurrido desde el último touchdown
        self.time_since_last_TD = 0.0
        # Modo de la trayectoria
        self.StanceSwing = self.STANCE
        # Valor de fase en la fase de swing [0, 1] de la pata de referencia
        self.SwRef = 0.0
        self.Stref = 0.0
        # Indica si la pata de referencia ha tocado el suelo
        self.TD = False

        # Tiempo de swing
        self.Tswing = Tswing

        # Pata de referencia
        self.ref_idx = 0

        # Almacena las fases de todas las patas
        self.Phases = self.dSref

    def reset(self):
        """Reinicia los parámetros del generador de marcha de Bézier
        """
        self.Prev_fxyz = [0.0, 0.0, 0.0, 0.0]

        # Tiempo total transcurrido
        self.time = 0.0
        # Tiempo de touchdown
        self.TD_time = 0.0
        # Tiempo transcurrido desde el último touchdown
        self.time_since_last_TD = 0.0
        # Modo de la trayectoria
        self.StanceSwing = self.STANCE
        # Valor de fase en la fase de swing [0, 1] de la pata de referencia
        self.SwRef = 0.0
        self.Stref = 0.0
        # Indica si la pata de referencia ha tocado el suelo
        self.TD = False

    def GetPhase(self, index, Tstance, Tswing):
        """Obtiene la fase de una pata individual.

        NOTA: modificación respecto al artículo original:

        if ti < -Tswing:
           ti += Tstride

        Esto evita una discontinuidad de fase si el usuario elige
        una combinación de longitud de paso y velocidad que cause Tstance > Tswing.

        :param index: Índice de la pata, usado para identificar el desfase requerido
        :param Tstance: Periodo de stance actual especificado por el usuario
        :param Tswing: Periodo de swing (constante, miembro de la clase)
        :return: Fase de la pata y StanceSwing (bool) que indica si la pata
                 está en modo stance o swing
        """
        StanceSwing = self.STANCE
        Sw_phase = 0.0
        Tstride = Tstance + Tswing
        ti = self.Get_ti(index, Tstride)

        # NOTA: ¡¡EL ARTÍCULO ORIGINAL OMITIÓ ESTA LÓGICA!!
        if ti < -Tswing:
            ti += Tstride

        # STANCE
        if ti >= 0.0 and ti <= Tstance:
            StanceSwing = self.STANCE
            if Tstance == 0.0:
                Stnphase = 0.0
            else:
                Stnphase = ti / float(Tstance)
            if index == self.ref_idx:
                # print("STANCE REF: {}".format(Stnphase))
                self.StanceSwing = StanceSwing
            return Stnphase, StanceSwing
        # SWING
        elif ti >= -Tswing and ti < 0.0:
            StanceSwing = self.SWING
            Sw_phase = (ti + Tswing) / Tswing
        elif ti > Tstance and ti <= Tstride:
            StanceSwing = self.SWING
            Sw_phase = (ti - Tstance) / Tswing
        # Contacto con el suelo al final del swing
        if Sw_phase >= 1.0:
            Sw_phase = 1.0
        if index == self.ref_idx:
            # print("SWING REF: {}".format(Sw_phase))
            self.StanceSwing = StanceSwing
            self.SwRef = Sw_phase
            # Contacto con el suelo de la pata de referencia al final del swing
            if self.SwRef >= 0.999:
                self.TD = True
            # else:
            #     self.TD = False
        return Sw_phase, StanceSwing

    def Get_ti(self, index, Tstride):
        """Obtiene el índice temporal para una pata individual

        :param index: Índice de la pata, usado para identificar el desfase requerido
        :param Tstride: Periodo total del movimiento de la pata (Tstance + Tswing)
        :return: Índice temporal de la pata
        """
        # NOTA: Por alguna razón, Python tiene problemas numéricos aquí
        # Forzamos a 0 para la pata de referencia
        if index == self.ref_idx:
            self.dSref[index] = 0.0
        return self.time_since_last_TD - self.dSref[index] * Tstride

    def Increment(self, dt, Tstride):
        """Incrementa el reloj interno del generador de marcha de Bézier (self.time)

        :param dt: Paso de tiempo
        :param Tstride: Periodo total del movimiento de la pata (Tstance + Tswing)
        :return: Índice temporal de la pata
        """
        self.CheckTouchDown()
        self.time_since_last_TD = self.time - self.TD_time
        if self.time_since_last_TD > Tstride:
            self.time_since_last_TD = Tstride
        elif self.time_since_last_TD < 0.0:
            self.time_since_last_TD = 0.0
        # print("T STRIDE: {}".format(Tstride))
        # Incrementamos el tiempo al final, en caso de que acabe de ocurrir un touchdown
        # Así obtenemos time_since_last_TD = 0.0
        self.time += dt

        # Si Tstride = Tswing, entonces Tstance = 0
        # REINICIAMOS TODO
        if Tstride < self.Tswing + dt:
            self.time = 0.0
            self.time_since_last_TD = 0.0
            self.TD_time = 0.0
            self.SwRef = 0.0

    def CheckTouchDown(self):
        """Verifica si ha ocurrido un touchdown de la pata de referencia,
           y si esto justifica reiniciar el tiempo de touchdown
        """
        if self.SwRef >= 0.9 and self.TD:
            self.TD_time = self.time
            self.TD = False
            self.SwRef = 0.0

    def BernSteinPoly(self, t, k, point):
        """Calcula el punto en el polinomio de Bernstein
           basado en la fase (0->1), el número del punto (0-11),
           y el valor del punto de control

           :param t: fase
           :param k: número del punto
           :param point: valor del punto
           :return: Valor en la curva de Bézier
        """
        return point * self.Binomial(k) * np.power(t, k) * np.power(
            1 - t, self.NumControlPoints - k)

    def Binomial(self, k):
        """Resuelve el teorema del binomio dado el número de un punto de Bézier
           en relación con el número total de puntos.

           :param k: Número del punto de Bézier
           :returns: Solución binomial
        """
        return factorial(self.NumControlPoints) / (
            factorial(k) *
            factorial(self.NumControlPoints - k))

    def BezierSwing(self, phase, L, LateralFraction, clearance_height=0.04):
        """Calcula las coordenadas del paso para el período de Bézier (swing)

           :param phase: Fase actual de la trayectoria
           :param L: Longitud del paso
           :param LateralFraction: Determina cuán lateral es el movimiento
           :param clearance_height: Altura de despeje del pie durante la fase de swing

           :returns: Coordenadas X, Y, Z del pie relativas al cuerpo sin modificar
        """

        # Coordenadas polares de la pata
        X_POLAR = np.cos(LateralFraction)
        Y_POLAR = np.sin(LateralFraction)

        # Puntos de la curva de Bézier (12 pts)
        # NOTA: L es la MITAD de la longitud del paso
        # Componente hacia adelante
        STEP = np.array([
            -L,  # Punto de control 0, mitad de la longitud de la zancada
            -L * 1.4,  # Punto de control 1, diferencia entre 1 y 0 = velocidad de elevación en X
            -L * 1.5,  # Puntos de control 2, 3, 4 superpuestos para
            -L * 1.5,  # cambio de dirección después de
            -L * 1.5,  # seguimiento
            0.0,  # Cambio de aceleración durante la protracción
            0.0,  # Por eso incluimos tres
            0.0,  # puntos de control superpuestos: 5, 6, 7
            L * 1.5,  # Cambio de dirección para la retracción de la pata en swing
            L * 1.5,  # requiere dos puntos de control superpuestos: 8, 9
            L * 1.4,  # Velocidad de retracción de la pata en swing = Ctrl 11 - 10
            L
        ])
        # Consideramos movimientos laterales multiplicando por la coordenada polar.
        # LateralFraction cambia el movimiento de la pata desde X hacia Y+ o Y-
        # a medida que se aleja de cero
        X = STEP * X_POLAR

        # Consideramos movimientos laterales multiplicando por la coordenada polar.
        # LateralFraction cambia el movimiento de la pata desde X hacia Y+ o Y-
        # a medida que se aleja de cero
        Y = STEP * Y_POLAR

        # Componente vertical
        Z = np.array([
            0.0,  # Puntos de control doblemente superpuestos para velocidad cero de elevación
            0.0,  # Velocidad respecto a la cadera (Pts 0 y 1)
            clearance_height * 0.9,  # Tres puntos superpuestos para cambio de
            clearance_height * 0.9,  # dirección de fuerza durante la transición de
            clearance_height * 0.9,  # seguimiento a protracción (2, 3, 4)
            clearance_height * 0.9,  # Puntos doblemente superpuestos para cambio de
            clearance_height * 0.9,  # dirección de trayectoria durante la protracción (5, 6)
            clearance_height * 1.1,  # Altura máxima de despeje en el centro de la trayectoria, Pt 7
            clearance_height * 1.1,  # Transición suave de protracción
            clearance_height * 1.1,  # a retracción, dos puntos de control (8, 9)
            0.0,  # Puntos doblemente superpuestos para velocidad cero
            0.0,  # en el contacto con el suelo respecto a la cadera (Pts 10 y 11)
        ])

        stepX = 0.
        stepY = 0.
        stepZ = 0.
        # Suma del polinomio de Bernstein sobre los puntos de control
        for i in range(len(X)):
            stepX += self.BernSteinPoly(phase, i, X[i])
            stepY += self.BernSteinPoly(phase, i, Y[i])
            stepZ += self.BernSteinPoly(phase, i, Z[i])

        return stepX, stepY, stepZ

    def SineStance(self, phase, L, LateralFraction, penetration_depth=0.00):
        """Calcula las coordenadas del paso para el período sinusoidal de stance

           :param phase: Fase actual de la trayectoria
           :param L: Longitud del paso
           :param LateralFraction: Determina cuán lateral es el movimiento
           :param penetration_depth: Profundidad de penetración del pie durante la fase de stance

           :returns: Coordenadas X, Y, Z del pie relativas al cuerpo sin modificar
        """
        X_POLAR = np.cos(LateralFraction)
        Y_POLAR = np.sin(LateralFraction)
        # Se mueve desde +L a -L
        step = L * (1.0 - 2.0 * phase)
        stepX = step * X_POLAR
        stepY = step * Y_POLAR
        if L != 0.0:
            stepZ = -penetration_depth * np.cos(
                (np.pi * (stepX + stepY)) / (2.0 * L))
        else:
            stepZ = 0.0
        return stepX, stepY, stepZ

    def YawCircle(self, T_bf, index):
        """ Calcula la rotación requerida del plano de la trayectoria
            para el movimiento de guiñada (yaw)

           :param T_bf: Vector predeterminado de cuerpo a pie
           :param index: Índice del pie en el contenedor
           :returns: phi_arc, ángulo de rotación del plano necesario para el movimiento de yaw
        """

        # Magnitud del pie según el tipo de pata
        DefaultBodyToFoot_Magnitude = np.sqrt(T_bf[0]**2 + T_bf[1]**2)

        # Ángulo de rotación según el tipo de pata
        DefaultBodyToFoot_Direction = np.arctan2(T_bf[1], T_bf[0])

        # Coordenadas anteriores de la pata relativas a las coordenadas predeterminadas
        g_xyz = self.Prev_fxyz[index] - np.array([T_bf[0], T_bf[1], T_bf[2]])

        # Modulamos la magnitud para mantener el trazado de un círculo
        g_mag = np.sqrt((g_xyz[0])**2 + (g_xyz[1])**2)
        th_mod = np.arctan2(g_mag, DefaultBodyToFoot_Magnitude)

        # Ángulo recorrido por el pie para la rotación
        # FR y RL
        if index == 1 or index == 2:
            phi_arc = np.pi / 2.0 + DefaultBodyToFoot_Direction + th_mod
        # FL y RR
        else:
            phi_arc = np.pi / 2.0 - DefaultBodyToFoot_Direction + th_mod

        # print("INDEX {}: \t Angle: {}".format(
        #     index, np.degrees(DefaultBodyToFoot_Direction)))

        return phi_arc

    def SwingStep(self, phase, L, LateralFraction, YawRate, clearance_height,
                  T_bf, key, index):
        """Calcula las coordenadas del paso para el período de Bézier (swing)
           usando una combinación de coordenadas de paso lineal y rotacional
           inicialmente descompuestas de la entrada del usuario:
           L, LateralFraction y YawRate

           :param phase: Fase actual de la trayectoria
           :param L: Longitud del paso
           :param LateralFraction: Determina cuán lateral es el movimiento
           :param YawRate: Tasa de guiñada (yaw) deseada del cuerpo
           :param clearance_height: Altura de despeje del pie durante la fase de swing
           :param T_bf: Vector predeterminado de cuerpo a pie
           :param key: Indica qué pie se está procesando
           :param index: Índice del pie en el contenedor

           :returns: Coordenadas del pie relativas al cuerpo sin modificar
        """

        # Ángulo del pie para movimiento tangente al círculo en yaw
        phi_arc = self.YawCircle(T_bf, index)

        # Obtiene coordenadas del pie para movimiento lineal
        X_delta_lin, Y_delta_lin, Z_delta_lin = self.BezierSwing(
            phase, L, LateralFraction, clearance_height)

        X_delta_rot, Y_delta_rot, Z_delta_rot = self.BezierSwing(
            phase, YawRate, phi_arc, clearance_height)

        coord = np.array([
            X_delta_lin + X_delta_rot, Y_delta_lin + Y_delta_rot,
            Z_delta_lin + Z_delta_rot
        ])

        self.Prev_fxyz[index] = coord

        return coord

    def StanceStep(self, phase, L, LateralFraction, YawRate, penetration_depth,
                   T_bf, key, index):
        """Calcula las coordenadas del paso para el período sinusoidal (stance)
           usando una combinación de coordenadas de paso lineal y rotacional
           inicialmente descompuestas de la entrada del usuario:
           L, LateralFraction y YawRate

           :param phase: Fase actual de la trayectoria
           :param L: Longitud del paso
           :param LateralFraction: Determina cuán lateral es el movimiento
           :param YawRate: Tasa de guiñada (yaw) deseada del cuerpo
           :param penetration_depth: Profundidad de penetración del pie durante la fase de stance
           :param T_bf: Vector predeterminado de cuerpo a pie
           :param key: Indica qué pie se está procesando
           :param index: Índice del pie en el contenedor

           :returns: Coordenadas del pie relativas al cuerpo sin modificar
        """

        # Ángulo del pie para movimiento tangente al círculo en yaw
        phi_arc = self.YawCircle(T_bf, index)

        # Obtiene coordenadas del pie para movimiento lineal
        X_delta_lin, Y_delta_lin, Z_delta_lin = self.SineStance(
            phase, L, LateralFraction, penetration_depth)

        X_delta_rot, Y_delta_rot, Z_delta_rot = self.SineStance(
            phase, YawRate, phi_arc, penetration_depth)

        coord = np.array([
            X_delta_lin + X_delta_rot, Y_delta_lin + Y_delta_rot,
            Z_delta_lin + Z_delta_rot
        ])

        self.Prev_fxyz[index] = coord

        return coord

    def GetFootStep(self, L, LateralFraction, YawRate, clearance_height,
                    penetration_depth, Tstance, T_bf, index, key):
        """Calcula las coordenadas del paso ya sea en la porción de Bézier o
           sinusoidal de la trayectoria, según la fase obtenida

           :param L: Longitud del paso
           :param LateralFraction: Determina cuán lateral es el movimiento
           :param YawRate: Tasa de guiñada (yaw) deseada del cuerpo
           :param clearance_height: Altura de despeje del pie durante la fase de swing
           :param penetration_depth: Profundidad de penetración del pie durante la fase de stance
           :param Tstance: Periodo de stance actual especificado por el usuario
           :param T_bf: Vector predeterminado de cuerpo a pie
           :param index: Índice del pie en el contenedor
           :param key: Indica qué pie se está procesando

           :returns: Coordenadas del pie relativas al cuerpo sin modificar
        """
        phase, StanceSwing = self.GetPhase(index, Tstance, self.Tswing)
        if StanceSwing == self.SWING:
            stored_phase = phase + 1.0
        else:
            stored_phase = phase
        # Solo para seguimiento
        self.Phases[index] = stored_phase
        # print("LEG: {} \t PHASE: {}".format(index, stored_phase))
        if StanceSwing == self.STANCE:
            return self.StanceStep(phase, L, LateralFraction, YawRate,
                                   penetration_depth, T_bf, key, index)
        elif StanceSwing == self.SWING:
            return self.SwingStep(phase, L, LateralFraction, YawRate,
                                  clearance_height, T_bf, key, index)

    def GenerateTrajectory(self,
                           L,
                           LateralFraction,
                           YawRate,
                           vel,
                           T_bf_,
                           T_bf_curr,
                           clearance_height=0.06,
                           penetration_depth=0.01,
                           contacts=[0, 0, 0, 0],
                           dt=None):
        """Calcula las coordenadas del paso para cada pata

           :param L: Longitud del paso
           :param LateralFraction: Determina cuán lateral es el movimiento
           :param YawRate: Tasa de guiñada (yaw) deseada del cuerpo
           :param vel: Velocidad deseada del paso
           :param clearance_height: Altura de despeje del pie durante la fase de swing
           :param penetration_depth: Profundidad de penetración del pie durante la fase de stance
           :param contacts: Arreglo que contiene 1 si hay contacto y 0 en caso contrario
           :param dt: Paso de tiempo

           :returns: Coordenadas del pie relativas al cuerpo sin modificar
        """
        # Primero, obtenemos Tstance a partir de la velocidad deseada y la longitud de la zancada
        # NOTA: L es la MITAD de la longitud de la zancada
        if vel != 0.0:
            Tstance = 2.0 * abs(L) / abs(vel)
        else:
            Tstance = 0.0
            L = 0.0
            self.TD = False
            self.time = 0.0
            self.time_since_last_TD = 0.0

        # Luego, obtenemos el tiempo desde el último touchdown e incrementamos el contador de tiempo
        if dt is None:
            dt = self.dt

        YawRate *= dt

        # Detectamos pasos de tiempo inviables
        if Tstance < dt:
            Tstance = 0.0
            L = 0.0
            self.TD = False
            self.time = 0.0
            self.time_since_last_TD = 0.0
            YawRate = 0.0
        # NOTA: ES MUCHO MÁS ESTABLE CON ESTO
        elif Tstance > 1.3 * self.Tswing:
            Tstance = 1.3 * self.Tswing

        # Verificamos contactos
        if contacts[0] == 1 and Tstance > dt:
            self.TD = True

        self.Increment(dt, Tstance + self.Tswing)

        T_bf = copy.deepcopy(T_bf_)
        for i, (key, Tbf_in) in enumerate(T_bf_.items()):
            # TODO: HACER ESTO MÁS ELEGANTE
            if key == "FL":
                self.ref_idx = i
                self.dSref[i] = 0.0
            if key == "FR":
                self.dSref[i] = 0.5
            if key == "RL":
                self.dSref[i] = 0.5
            if key == "RR":
                self.dSref[i] = 0.0
            _, p_bf = TransToRp(Tbf_in)
            if Tstance > 0.0:
                step_coord = self.GetFootStep(L, LateralFraction, YawRate,
                                              clearance_height,
                                              penetration_depth, Tstance, p_bf,
                                              i, key)
            else:
                step_coord = np.array([0.0, 0.0, 0.0])
            T_bf[key][0, 3] = Tbf_in[0, 3] + step_coord[0]
            T_bf[key][1, 3] = Tbf_in[1, 3] + step_coord[1]
            T_bf[key][2, 3] = Tbf_in[2, 3] + step_coord[2]
        return T_bf