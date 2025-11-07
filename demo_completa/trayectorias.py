import scipy.io
import numpy as np
import toppra as ta
import toppra.constraint as constraint
import toppra.algorithm as algo
import os
import glob
from scipy import signal # <-- NUEVO: Importar para el filtro

# --- 1. Configuración de Directorios ---
INPUT_DIR = 'raw_trajectories'
OUTPUT_DIR = 'toppra_trajectories'

# --- 2. Crear Directorio de Salida (si no existe) ---
os.makedirs(OUTPUT_DIR, exist_ok=True)
print(f"Directorio de salida asegurado: {OUTPUT_DIR}")

# --- 3. Definir Restricciones del Robot (una sola vez) ---
vlim_abs = np.array([2.0, 2.0, 2.0, 1.0, 1.0, 1.0]) 
vlim = np.vstack([-vlim_abs, vlim_abs]).T
alim_abs = np.array([1.0, 1.5, 1.5, 1.0, 1.0, 1.0])
alim = np.vstack([-alim_abs, alim_abs]).T

# --- 4. Encontrar todos los archivos .mat de entrada ---
search_pattern = os.path.join(INPUT_DIR, '*.mat')
input_files = glob.glob(search_pattern)

if not input_files:
    print(f"Error: No se encontraron archivos .mat en el directorio '{INPUT_DIR}'.")
    exit()

print(f"Encontrados {len(input_files)} archivos para procesar. Comenzando...")

# --- 5. Bucle de Procesamiento por Lotes ---
for input_filepath in input_files:
    input_filename = os.path.basename(input_filepath)
    print(f"\n--- Procesando: {input_filename} ---")
    
    # Identificar la trayectoria especial
    is_special_traj = input_filename.endswith('_rawtraj_10-11.mat')
    target_duration = 10.0
    
    try:
        # 5.1. Cargar el archivo .mat individual
        mat_data = scipy.io.loadmat(input_filepath)
        
        # 5.2. Extraer los datos
        waypoints = mat_data['waypoints_q']
        path_pos = mat_data['path_pos_s'].flatten()

        if waypoints.shape[0] <= 1:
            print("Error: La trayectoria tiene 1 o menos puntos. Saltando.")
            continue

        print(f"Datos cargados: {waypoints.shape[0]} waypoints para {waypoints.shape[1]} juntas.")

        # --- 5.3. NUEVO BLOQUE: Suavizado condicional ---
        if is_special_traj:
            print("*** Aplicando filtro de suavizado (Savitzky-Golay) a '10-11' ***")
            
            # Parámetros (ajustables):
            # window_length: Más grande = más suave. Debe ser IMPAR.
            # polyorder: Orden del polinomio. Debe ser < window_length.
            window_len = 51  # Prueba con 51 (aprox 0.5s si dt=0.01)
            poly_order = 3   # Cúbico es bueno para trayectorias
            
            # Asegurarse de que la ventana no sea más grande que los datos
            if waypoints.shape[0] > window_len:
                try:
                    waypoints_suavizados = signal.savgol_filter(
                        waypoints, 
                        window_length=window_len, 
                        polyorder=poly_order, 
                        axis=0 # Aplicar a lo largo de las muestras (filas)
                    )
                    # Reemplazar los waypoints originales por los suavizados
                    waypoints = waypoints_suavizados
                    print(f"Suavizado completado con ventana={window_len}, orden={poly_order}")
                except Exception as e_filter:
                    print(f"ADVERTENCIA: Falló el filtro de suavizado: {e_filter}. Usando datos originales.")
            else:
                print("ADVERTENCIA: No se puede suavizar, la trayectoria es demasiado corta. Usando datos originales.")
        # ---------------------------------------------------

        # 5.4. Crear el objeto de trayectoria (Path)
        # ¡Ahora 'path' usará los 'waypoints' suavizados si es la traj 10-11!
        path = ta.SplineInterpolator(path_pos, waypoints)

        # 5.5. Crear las restricciones
        pc_vel = constraint.JointVelocityConstraint(vlim)
        pc_acc = constraint.JointAccelerationConstraint(alim)

        # 5.6. Instanciar y resolver el problema TOPPRA
        gpts = max(500, waypoints.shape[0] * 2) 
        instance = algo.TOPPRA([pc_vel, pc_acc], path,
                               gridpoints=np.linspace(path_pos[0], path_pos[-1], gpts))
        
        print("Calculando parametrización de tiempo óptima...")
        jnt_traj = instance.compute_trajectory(0, 0)

        # 5.7. Comprobar si TOPPRA falló
        if jnt_traj is None:
            print("Error: TOPPRA no pudo encontrar una solución. Saltando este archivo.")
            continue
        
        optimal_duration = jnt_traj.duration
        print(f"¡Trayectoria calculada! Duración ÓPTIMA (post-suavizado): {optimal_duration:.3f} segundos")

        # --- 5.8. Re-escalar para trayectoria 10-11 (esta lógica sigue siendo válida) ---
        final_duration = optimal_duration
        scale_factor = 1.0

        if is_special_traj:
            print(f"*** Trayectoria especial '10-11' detectada. Objetivo: {target_duration}s ***")
            if optimal_duration > target_duration:
                # El mensaje de "6s" estaba hardcodeado, lo corrijo a la variable
                print(f"ADVERTENCIA: El tiempo óptimo ({optimal_duration:.3f}s) es MAYOR que el objetivo ({target_duration}s).")
                print("             Es IMPOSIBLE cumplir el objetivo sin violar los límites vlim/alim.")
                print("             Se guardará la trayectoria con el tiempo óptimo.")
                final_duration = optimal_duration
            else:
                print(f"Tiempo óptimo ({optimal_duration:.3f}s) es MENOR/IGUAL al objetivo.")
                print(f"             Re-escalando la trayectoria para que dure exactamente {target_duration}s.")
                final_duration = target_duration
                if final_duration > 0: 
                    scale_factor = optimal_duration / final_duration
        
        print(f"Duración FINAL para guardar: {final_duration:.3f} segundos")

        # 5.9. Muestrear la nueva trayectoria (optimizada o re-escalada)
        num_samples = int(final_duration / 0.01) + 1
        ts_sample = np.linspace(0, final_duration, num_samples)

        if scale_factor == 1.0:
            qs_new = jnt_traj(ts_sample)
            qds_new = jnt_traj(ts_sample, 1)
            qdds_new = jnt_traj(ts_sample, 2)
        else:
            t_scaled = ts_sample * scale_factor
            qs_new = jnt_traj(t_scaled)
            qds_new = jnt_traj(t_scaled, 1) * scale_factor
            qdds_new = jnt_traj(t_scaled, 2) * (scale_factor ** 2)

        # 5.10. Preparar datos para guardar
        datos_para_matlab = {
            'ts_sample': ts_sample,
            'q_toppra': qs_new,
            'qd_toppra': qds_new,
            'qdd_toppra': qdds_new,
            'original_optimal_duration': optimal_duration,
            'final_duration': final_duration
        }
        
        # 5.11. Crear nombre de archivo de salida
        output_filename = input_filename.replace('_rawtraj_', '_toppratraj_')
        output_filepath = os.path.join(OUTPUT_DIR, output_filename)
        
        # 5.12. Guardar el nuevo archivo .mat
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