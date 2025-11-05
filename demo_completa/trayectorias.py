import scipy.io
import numpy as np
import toppra as ta
import toppra.constraint as constraint
import toppra.algorithm as algo
import os
import glob

# --- 1. Configuración de Directorios ---
INPUT_DIR = 'raw_trajectories'
OUTPUT_DIR = 'toppra_trajectories'

# --- 2. Crear Directorio de Salida (si no existe) ---
os.makedirs(OUTPUT_DIR, exist_ok=True)
print(f"Directorio de salida asegurado: {OUTPUT_DIR}")

# --- 3. Definir 
# Restricciones del Robot (una sola vez) ---
# Estas son las mismas para todas las trayectorias
# Límites de velocidad (rad/s) - (¡Ajusta tus valores reales!)
vlim_abs = np.array([2.0, 2.0, 2.0, 2.5, 2.5, 3.0]) 
vlim = np.vstack([-vlim_abs, vlim_abs]).T # Formato (N_juntas, 2) [min, max]

# Límites de aceleración (rad/s^2) - (¡Ajusta tus valores reales!)
alim_abs = np.array([1.0, 1.5, 1.5, 1.0, 1.0, 1.0])
alim = np.vstack([-alim_abs, alim_abs]).T # Formato (N_juntas, 2) [min, max]

# --- 4. Encontrar todos los archivos .mat de entrada ---
# Busca todos los archivos que terminen en .mat dentro de INPUT_DIR
search_pattern = os.path.join(INPUT_DIR, '*.mat')
input_files = glob.glob(search_pattern)

if not input_files:
    print(f"Error: No se encontraron archivos .mat en el directorio '{INPUT_DIR}'.")
    print("Asegúrate de ejecutar primero el script de MATLAB.")
    exit()

print(f"Encontrados {len(input_files)} archivos para procesar. Comenzando...")

# --- 5. Bucle de Procesamiento por Lotes ---
for input_filepath in input_files:
    # Extraer el nombre base del archivo (ej: R1_rawtraj_0-1.mat)
    input_filename = os.path.basename(input_filepath)
    print(f"\n--- Procesando: {input_filename} ---")

    try:
        # 5.1. Cargar el archivo .mat individual
        mat_data = scipy.io.loadmat(input_filepath)
        
        # 5.2. Extraer los datos
        waypoints = mat_data['waypoints_q']
        path_pos = mat_data['path_pos_s'].flatten() # .flatten() para (N,)

        if waypoints.shape[0] <= 1:
            print("Error: La trayectoria tiene 1 o menos puntos. Saltando.")
            continue

        print(f"Datos cargados: {waypoints.shape[0]} waypoints para {waypoints.shape[1]} juntas.")

        # 5.3. Crear el objeto de trayectoria (Path)
        path = ta.SplineInterpolator(path_pos, waypoints)

        # 5.4. Crear las restricciones
        pc_vel = constraint.JointVelocityConstraint(vlim)
        pc_acc = constraint.JointAccelerationConstraint(alim)

        # 5.5. Instanciar y resolver el problema TOPPRA
        # (Aumentamos gridpoints por si las trayectorias son cortas)
        gpts = max(101, waypoints.shape[0] * 2) 
        instance = algo.TOPPRA([pc_vel, pc_acc], path,
                               gridpoints=np.linspace(path_pos[0], path_pos[-1], gpts))
        
        print("Calculando parametrización de tiempo óptima...")
        jnt_traj = instance.compute_trajectory(0, 0)

        # 5.6. Comprobar si TOPPRA falló
        if jnt_traj is None:
            print("Error: TOPPRA no pudo encontrar una solución. Saltando este archivo.")
            continue
            
        print(f"¡Trayectoria calculada! Duración total: {jnt_traj.duration:.3f} segundos")

        # 5.7. Muestrear la nueva trayectoria optimizada (sin graficar)
        ts_sample = np.arange(0, jnt_traj.duration, 0.01) # Muestreo a 100 Hz
        qs_new = jnt_traj(ts_sample)    # Posiciones
        qds_new = jnt_traj(ts_sample, 1) # Velocidades
        qdds_new = jnt_traj(ts_sample, 2) # Aceleraciones

        # 5.8. Preparar datos para guardar
        datos_para_matlab = {
            'ts_sample': ts_sample,
            'q_toppra': qs_new,
            'qd_toppra': qds_new,
            'qdd_toppra': qdds_new
        }
        
        # 5.9. Crear nombre de archivo de salida
        # Reemplaza "_rawtraj_" con "_toppratraj_"
        output_filename = input_filename.replace('_rawtraj_', '_toppratraj_')
        
        # Crear la ruta completa de salida
        output_filepath = os.path.join(OUTPUT_DIR, output_filename)
        
        # 5.10. Guardar el nuevo archivo .mat
        scipy.io.savemat(
            output_filepath, 
            datos_para_matlab, 
            do_compression=True, 
            oned_as='row'
        )
        
        print(f"Trayectoria optimizada guardada en: {output_filepath}")

    except Exception as e:
        print(f"Error fatal procesando el archivo {input_filename}: {e}")
        print("Saltando al siguiente archivo.")
        continue

print("\n--- Proceso por lotes completado. ---")