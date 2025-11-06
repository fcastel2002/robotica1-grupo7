# Contenido de: robotKine.py

from roboticstoolbox import DHRobot, RevoluteDH
import numpy as np
from numpy import pi
from toppra.constraint import SecondOrderConstraint
class IRB120Kinematics:
    """
    Clase contenedora para la cinemática del IRB120.
    toppra necesita un objeto que tenga un método .jacob(q)
    """
    def __init__(self):
        L1 = RevoluteDH(d=0.290, a=0,     alpha=-pi/2, offset=0)
        L2 = RevoluteDH(d=0,     a=0.270, alpha=0,    offset=-pi/2) # El offset puede variar
        L3 = RevoluteDH(d=0,     a=0.070, alpha=-pi/2, offset=0)
        L4 = RevoluteDH(d=0.302, a=0,     alpha=pi/2,offset=0)
        L5 = RevoluteDH(d=0,     a=0,     alpha=-pi/2, offset=0)
        L6 = RevoluteDH(d=0.072, a=0,     alpha=0,    offset=0) # d puede ser negativo

        # 2. Crear la lista de eslabones
        lista_eslabones = [L1, L2, L3, L4, L5, L6]

        try:
            # 3. Construir el robot
            self.robot = DHRobot(
                lista_eslabones, 
                name="IRB120_Custom"
            )
            
            # --- Configuración del robot ---
            self.dof = self.robot.n       # Nro de juntas (6)
            self.vel_dof = 6              # Nro de vel. cartesianas [x,y,z,wx,wy,wz]
            
            print(f"Modelo personalizado '{self.robot.name}' cargado, Juntas: {self.dof}")
        except Exception as e:
                print(f"Error al construir el robot desde DH: {e}")
                print("Verifica los parámetros de los eslabones.")
                exit()

    def jacob(self, q):
        """
        Calcula el Jacobiano del efector final para una posición
        de junta 'q' dada.
        """
        # Usamos el Jacobiano en el frame base (world frame)
        q_flat = q.flatten()
        return self.robot.jacob0(q_flat)
    def fk_vel(self, q, dq):
        """
        Calcula la velocidad cartesiana [lineal, angular]
        dada una posición (q) y velocidad (dq) de juntas.
        """
        q_flat = q.flatten()
        dq_flat = dq.flatten()
        
        # J es (6 x 6)
        J = self.robot.jacob0(q_flat)
        
        # v_cart = J * dq
        # v_cart es (6,) [vx, vy, vz, wx, wy, wz]
        v_cart = J @ dq_flat
        
        # Separa lineal y angular
        v_lineal = v_cart[:3]
        v_angular = v_cart[3:]
        
        # Devuelve las NORMAS (magnitudes) de velocidad
        return np.linalg.norm(v_lineal), np.linalg.norm(v_angular)

class CartesianVelocityConstraint(SecondOrderConstraint):

    def __init__(self, kinematics_model, max_linear_vel, max_angular_vel):
        """
        :param kinematics_model: Una instancia de tu clase IRB120Kinematics.
        :param max_linear_vel: Límite MÁXIMO de velocidad lineal (norma) [m/s]
        :param max_angular_vel: Límite MÁXIMO de velocidad angular (norma) [rad/s]
        """
        super(CartesianVelocityConstraint, self).__init__()
        
        self.kin = kinematics_model
        self.max_lin_vel = max_linear_vel
        self.max_ang_vel = max_angular_vel
        self.dof = self.kin.dof
        
        # Función de "forward kinematics" que esta clase usará
        self.fk = self.kin.fk_vel

    def invdyn(self, q, dq, ddq):
        # Esta restricción no depende de la aceleración, solo de q y dq
        # Toppra trabaja con variables u = s_doble_punto, x = s_punto_cuadrado
        # Aquí calculamos las velocidades cartesianas (normas)
        linear_vel_norm, angular_vel_norm = self.fk(q, dq)
        
        # Retornamos el CUADRADO de las normas
        return np.array([linear_vel_norm**2, angular_vel_norm**2])

    def constraintf(self, q):
        # Esta función define cómo la restricción se relaciona con s_punto_cuadrado (x)
        # f(q) * x <= g(q)
        # (v_lin_norm^2) * x <= v_max_lin^2
        # (v_ang_norm^2) * x <= v_max_ang^2
        #
        # Esto es f(q). No podemos calcularlo aquí, se define en invdyn.
        # Esta implementación usa una forma diferente.
        pass

    def constraintg(self, q):
        # Esta es la parte g(q)
        pass

    # --- Esta es la implementación mágica ---
    # Le dice a toppra que esta es una restricción canónica del tipo:
    #     x_min <= x <= x_max
    # donde x = s_punto_cuadrado (velocidad al cuadrado)
    
    # Sobrescribimos el método que toppra usa
    def compute_constraint_params(self, path, gridpoints):
        
        N = len(gridpoints)
        # q(s) y q'(s) [derivada respecto a s, NO al tiempo]
        q_grid = path(gridpoints)
        qs_grid = path(gridpoints, 1) # q'(s)
        
        # Pre-alocamos los límites
        # xbound = [x_min, x_max]
        xbound = np.zeros((N, 2))
        xbound[:, 1] = np.inf # x_max es infinito por defecto
        
        # Para cada punto en la trayectoria...
        for i in range(N):
            q = q_grid[i]
            qs = qs_grid[i] # q'(s)
            
            # Calculamos las "velocidades" cartesianas para q'(s)
            # v_lin_norm = || J_lin(q) * q'(s) ||
            # v_ang_norm = || J_ang(q) * q'(s) ||
            v_lin_norm_s, v_ang_norm_s = self.kin.fk_vel(q, qs)
            
            # La velocidad real es v(t) = q'(s) * s_punto(t)
            # La velocidad cartesiana real es V(t) = J(q) * q'(s) * s_punto(t)
            # La norma es ||V(t)|| = ||J(q) * q'(s)|| * s_punto(t)
            # ||V(t)|| = v_norm_s * s_punto
            
            # Queremos ||V(t)|| <= V_max
            # v_norm_s * s_punto <= V_max
            # s_punto <= V_max / v_norm_s
            
            # toppra usa x = s_punto_cuadrado, así que...
            # x <= (V_max / v_norm_s)^2
            
            x_max_lin = (self.max_lin_vel / v_lin_norm_s)**2 if v_lin_norm_s > 1e-6 else np.inf
            x_max_ang = (self.max_ang_vel / v_ang_norm_s)**2 if v_ang_norm_s > 1e-6 else np.inf
            
            # El límite es el más restrictivo de los dos
            xbound[i, 1] = min(x_max_lin, x_max_ang)

        # Devolvemos None para todos los otros tipos de restricción
        # y solo xbound para la restricción de velocidad al cuadrado
        return None, None, None, None, None, None, xbound