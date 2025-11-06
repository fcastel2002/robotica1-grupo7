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

# --- 3. Definir Restricciones del Robot (una sola vez) ---
# Límites de velocidad (rad/s)
vlim_abs = np.array([2.0, 2.0, 2.0, 1.0, 1.0, 1.0]) 
vlim = np.vstack([-vlim_abs, vlim_abs]).T

# Límites de aceleración (rad/s^2)
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
    # Extraer el nombre base del archivo (ej: R1_rawtraj_0-1.mat)
    input_filename = os.path.basename(input_filepath)
    print(f"\n--- Procesando: {input_filename} ---")
    
    # *** NUEVO: Identificar la trayectoria especial ***
    # Asumimos que el archivo se llama algo como "..._rawtraj_10-11.mat"
    is_special_traj = input_filename.endswith('_rawtraj_10-11.mat')
    ##
    target_duration = 10.0
    ##
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

        # 5.3. Crear el objeto de trayectoria (Path)
        path = ta.SplineInterpolator(path_pos, waypoints)

        # 5.4. Crear las restricciones
        pc_vel = constraint.JointVelocityConstraint(vlim)
        pc_acc = constraint.JointAccelerationConstraint(alim)

        # 5.5. Instanciar y resolver el problema TOPPRA
        gpts = max(500, waypoints.shape[0] * 2) 
        instance = algo.TOPPRA([pc_vel, pc_acc], path,
                               gridpoints=np.linspace(path_pos[0], path_pos[-1], gpts))
        
        print("Calculando parametrización de tiempo óptima...")
        jnt_traj = instance.compute_trajectory(0, 0)

        # 5.6. Comprobar si TOPPRA falló
        if jnt_traj is None:
            print("Error: TOPPRA no pudo encontrar una solución. Saltando este archivo.")
            continue
        
        optimal_duration = jnt_traj.duration
        print(f"¡Trayectoria calculada! Duración ÓPTIMA: {optimal_duration:.3f} segundos")

        # --- 5.7. NUEVO BLOQUE: Comprobar y Re-escalar para trayectoria 10-11 ---
        final_duration = optimal_duration
        scale_factor = 1.0 # Factor de escalado de tiempo

        if is_special_traj:
            print(f"*** Trayectoria especial '10-11' detectada. Objetivo: {target_duration}s ***")
            if optimal_duration > target_duration:
                print(f"ADVERTENCIA: El tiempo óptimo ({optimal_duration:.3f}s) es MAYOR que el objetivo.")
                print("             Es IMPOSIBLE cumplir 6s sin violar los límites vlim/alim.")
                print("             Se guardará la trayectoria con el tiempo óptimo.")
                final_duration = optimal_duration
            else:
                print(f"Tiempo óptimo ({optimal_duration:.3f}s) es MENOR/IGUAL al objetivo.")
                print(f"             Re-escalando la trayectoria para que dure exactamente {target_duration}s.")
                final_duration = target_duration
                if final_duration > 0: # Evitar división por cero si la duración es 0
                    scale_factor = optimal_duration / final_duration
                # Si optimal_duration es 0, scale_factor será 0, lo cual está bien.
        
        print(f"Duración FINAL para guardar: {final_duration:.3f} segundos")

        # 5.8. Muestrear la nueva trayectoria (optimizada o re-escalada)
        # Usamos linspace para asegurar que el punto final esté incluido
        num_samples = int(final_duration / 0.01) + 1
        ts_sample = np.linspace(0, final_duration, num_samples)

        if scale_factor == 1.0:
            # Caso normal: usamos la trayectoria óptima tal cual
            qs_new = jnt_traj(ts_sample)
            qds_new = jnt_traj(ts_sample, 1)
            qdds_new = jnt_traj(ts_sample, 2)
        else:
            # Caso re-escalado: mapeamos el nuevo tiempo [0, T_final] al tiempo óptimo [0, T_opt]
            # y aplicamos la regla de la cadena para las derivadas
            
            # t_scaled = t_sample * (T_opt / T_final) = t_sample * scale_factor
            t_scaled = ts_sample * scale_factor
            
            # q(t_sample) = jnt_traj(t_scaled)
            qs_new = jnt_traj(t_scaled)
            
            # qd(t_sample) = d(jnt_traj(t_scaled)) / dt_sample
            #              = jnt_traj_deriv(t_scaled) * (dt_scaled / dt_sample)
            #              = jnt_traj_deriv(t_scaled) * scale_factor
            qds_new = jnt_traj(t_scaled, 1) * scale_factor
            
            # qdd(t_sample) = d(q_dot(t_sample)) / dt_sample
            #               = jnt_traj_deriv2(t_scaled) * (dt_scaled / dt_sample) * scale_factor
            #               = jnt_traj_deriv2(t_scaled) * (scale_factor^2)
            qdds_new = jnt_traj(t_scaled, 2) * (scale_factor ** 2)

        # 5.9. Preparar datos para guardar
        datos_para_matlab = {
            'ts_sample': ts_sample,
            'q_toppra': qs_new,
            'qd_toppra': qds_new,
            'qdd_toppra': qdds_new,
            'original_optimal_duration': optimal_duration,
            'final_duration': final_duration
        }
        
        # 5.10. Crear nombre de archivo de salida
        output_filename = input_filename.replace('_rawtraj_', '_toppratraj_')
        output_filepath = os.path.join(OUTPUT_DIR, output_filename)
        
        # 5.11. Guardar el nuevo archivo .mat
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