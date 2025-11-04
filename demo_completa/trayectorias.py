import scipy.io
import numpy as np
import toppra as ta
import toppra.constraint as constraint
import toppra.algorithm as algo
import matplotlib.pyplot as plt

print("Cargando datos desde MATLAB...")

# 1. Cargar el archivo .mat
try:
    mat_data = scipy.io.loadmat('trayectoria_para_toppra.mat')
except FileNotFoundError:
    print("Error: No se encontró el archivo 'trayectoria_para_toppra.mat'.")
    print("Asegúrate de ejecutar primero el script de MATLAB.")
    exit()

# 2. Extraer los datos
#    waypoints_q -> (N_muestras, N_juntas)
#    path_pos_s  -> (N_muestras, 1)
waypoints = mat_data['waypoints_q']
path_pos = mat_data['path_pos_s'].flatten() # .flatten() para convertirlo en (N,)

print(f"Datos cargados: {waypoints.shape[0]} waypoints para {waypoints.shape[1]} juntas.")

# 3. Crear el objeto de trayectoria (Path) para toppra
#    Usamos un interpolador Spline Cúbico.
#    Le decimos que en la posición 's' (path_pos) debe estar en 'q' (waypoints)
path = ta.SplineInterpolator(path_pos, waypoints)

# 4. Definir las restricciones del robot (LÍMITES)
N_JOINTS = waypoints.shape[1]

# Límites de velocidad (rad/s) - (Ejemplo)
vlim_abs = np.array([2.0, 2.0, 2.0, 2.5, 2.5, 3.0]) 
vlim = np.vstack([-vlim_abs, vlim_abs]).T # Formato (N_juntas, 2) [min, max]

# Límites de aceleración (rad/s^2) - (Ejemplo)
alim_abs = np.array([1, 1.5, 1.5, 1.0, 1.0, 1.0])
alim = np.vstack([-alim_abs, alim_abs]).T # Formato (N_juntas, 2) [min, max]

# 5. Crear las restricciones para Toppra
#    Toppra v1.0+ usa un formato de lista [min, max]
pc_vel = constraint.JointVelocityConstraint(vlim)
pc_acc = constraint.JointAccelerationConstraint(alim)

# 6. Instanciar el problema de parametrización
#    (Usando el algoritmo TOPPRA más nuevo y robusto: )
instance = algo.TOPPRA([pc_vel, pc_acc], path,
                       gridpoints=np.linspace(path_pos[0], path_pos[-1], 1001))

print("Calculando parametrización de tiempo óptima...")
# 7. Resolver la trayectoria
jnt_traj = instance.compute_trajectory(0, 0)

if jnt_traj is None:
    print("Error: No se pudo encontrar una solución para la parametrización.")
else:
    print(f"¡Trayectoria calculada! Duración total: {jnt_traj.duration:.3f} segundos")

    # 8. Muestrear y graficar la nueva trayectoria optimizada
    #    Muestreamos a 100 Hz (dt = 0.01)
    ts_sample = np.arange(0, jnt_traj.duration, 0.01)
    
    # Interpolar en la nueva escala de tiempo
    qs_new = jnt_traj(ts_sample)    # Posiciones
    qds_new = jnt_traj(ts_sample, 1) # Velocidades
    qdds_new = jnt_traj(ts_sample, 2) # Aceleraciones

    # --- Gráficos ---
    fig, axs = plt.subplots(3, 1, figsize=(10, 10), sharex=True)
    
    # Títulos
    axs[0].set_title(f"Posición Articular (q) - Duración: {jnt_traj.duration:.3f} s")
    axs[1].set_title("Velocidad Articular (qd)")
    axs[2].set_title("Aceleración Articular (qdd)")

    # Labels
    axs[2].set_xlabel("Tiempo (s)")
    axs[0].set_ylabel("Posición (rad)")
    axs[1].set_ylabel("Velocidad (rad/s)")
    axs[2].set_ylabel("Aceleración (rad/s^2)")

    # Colores
    colors = plt.get_cmap('tab10', N_JOINTS)

    for i in range(N_JOINTS):
        # Posición
        axs[0].plot(ts_sample, qs_new[:, i], label=f'q{i+1}', color=colors(i))
        
        # Velocidad
        axs[1].plot(ts_sample, qds_new[:, i], label=f'qd{i+1}', color=colors(i))
        # Límites de velocidad
        axs[1].axhline(vlim[i, 1], color=colors(i), linestyle='--', lw=1)
        axs[1].axhline(vlim[i, 0], color=colors(i), linestyle='--', lw=1)
        
        # Aceleración
        axs[2].plot(ts_sample, qdds_new[:, i], label=f'qdd{i+1}', color=colors(i))
        # Límites de aceleración
        axs[2].axhline(alim[i, 1], color=colors(i), linestyle='--', lw=1)
        axs[2].axhline(alim[i, 0], color=colors(i), linestyle='--', lw=1)

    axs[0].legend(loc='best')
    axs[0].grid(True)
    axs[1].grid(True)
    axs[2].grid(True)
    
    plt.tight_layout()
    plt.show()
    if jnt_traj is not None:
        print("Guardando trayectoria optimizada para MATLAB...")
        
        # Preparamos los datos en un diccionario
        # Es importante usar nombres de variables claros
        datos_para_matlab = {
            'ts_sample': ts_sample,  # Vector de tiempo (N,)
            'q_toppra': qs_new,      # Posiciones (N, 6)
            'qd_toppra': qds_new,     # Velocidades (N, 6)
            'qdd_toppra': qdds_new    # Aceleraciones (N, 6)
        }
        
        # Nombre del archivo de salida
        nombre_archivo_salida = 'trayectoria_optimizada.mat'
        
        # Guardar en .mat
        # 'do_compression=True' ahorra espacio
        # 'oned_as='row'' asegura que los vectores (como ts_sample) se guarden como 1xN
        scipy.io.savemat(
            nombre_archivo_salida, 
            datos_para_matlab, 
            do_compression=True, 
            oned_as='row'
        )
        
        print(f"Trayectoria optimizada guardada en: {nombre_archivo_salida}")