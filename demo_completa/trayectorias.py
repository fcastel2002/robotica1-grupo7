import scipy.io
import numpy as np
import toppra as ta
import toppra.constraint as constraint
import toppra.algorithm as algo
import os
import glob
from scipy import signal

# --- 1. Configuración de Directorios ---
INPUT_DIR = 'raw_trajectories'
OUTPUT_DIR = 'toppra_trajectories'

# --- 2. Crear Directorio de Salida (si no existe) ---
os.makedirs(OUTPUT_DIR, exist_ok=True)
print(f"Directorio de salida asegurado: {OUTPUT_DIR}")

# ==============================================================================
# --- 3. Definir PERFILES de Restricciones ---
#    (¡Aquí defines TUS perfiles de robot!)
#
#    Añade aquí todos los conjuntos de límites que necesites.
#    Usa una clave (string) fácil de recordar para cada uno.
# ==============================================================================

# Perfil 1: Límites por defecto (Robot 1, ej: los que ya tenías)
perfil_robot1_default = {
    'vlim_abs': np.array([1.8, 1.8, 1.8, 1.0, 0.8, 0.8]),
    'alim_abs': np.array([1.0, 1.5, 1.5, 1.0, 1.0, 1.0])
}

# Perfil 2: Límites para "Cafe" (Robot 1, más lentos)
perfil_robot1_cafe_special = {
    'vlim_abs': np.array([1.0, 1.0, 1.0, 0.5, 2, 2]), # Más lento
    'alim_abs': np.array([1, 1, 1, 0.5, 0.5, 0.5])  # Más suave
}

# Perfil 3: Límites para Robot 2 (ejemplo)
perfil_robot2_articularq5 = {
    'vlim_abs': np.array([1.8, 1.8, 1.8, 1.0, 0.3, 0.8]),
    'alim_abs': np.array([1.0, 1.5, 1.5, 1.0, 1.0, 1.0])
}

# --- Contenedor de todos los perfiles ---
#   (Este diccionario agrupa todos los perfiles de arriba)
LIMIT_PROFILES = {
    'robot1_default': perfil_robot1_default,
    'robot1_cafe': perfil_robot1_cafe_special,
    'robot2_articularq5': perfil_robot2_articularq5
    # ... añade más perfiles aquí si los creas
}

# ==============================================================================
# --- 4. Mapeo de Archivos a Perfiles de Límites ---
#
#    El script comprobará si el nombre del archivo CONTIENE el texto de la
#    primera columna. Si es así, usará el perfil de la segunda columna.
#    Se comprueba en orden, el primero que coincida gana.
# ==============================================================================

PROFILE_MAP = [
    # ('texto_a_buscar_en_filename', 'profile_key_de_LIMIT_PROFILES')
    
    # Ejemplo: Asignar las dos primeras de "Cafe" al perfil especial
    ('Cafe_rawtraj_1-2', 'robot1_cafe'), # Si el archivo contiene "Cafe_rawtraj_1-2"
    ('Cafe_rawtraj_2-3', 'robot1_cafe'), # Si el archivo contiene "Cafe_rawtraj_2"

    ('Leche_rawtraj_7-8', 'robot2_default'),
    
    # Ejemplo: Asignar archivos de "Cafe" (que no sean 1 o 2) al default
    ('cafe_traj', 'robot1_default'), # Esta regla debe ir DESPUÉS de las específicas
]

# Perfil a usar si un archivo no coincide con NADA en PROFILE_MAP
DEFAULT_PROFILE_NAME = 'robot1_default' 

# --- 5. Encontrar todos los archivos .mat de entrada ---
search_pattern = os.path.join(INPUT_DIR, '*.mat')
input_files = glob.glob(search_pattern)

if not input_files:
    print(f"Error: No se encontraron archivos .mat en el directorio '{INPUT_DIR}'.")
    exit()

print(f"Encontrados {len(input_files)} archivos para procesar. Comenzando...")

# --- 6. Bucle de Procesamiento por Lotes ---
for input_filepath in input_files:
    input_filename = os.path.basename(input_filepath)
    print(f"\n--- Procesando: {input_filename} ---")
    
    # Identificar la trayectoria especial (para suavizado/escalado)
    is_special_traj = input_filename.endswith('_rawtraj_10-11.mat')
    target_duration = 10.0
    
    try:
        # 6.1. Cargar el archivo .mat individual
        mat_data = scipy.io.loadmat(input_filepath)
        
        # 6.2. Extraer los datos
        waypoints = mat_data['waypoints_q']
        path_pos = mat_data['path_pos_s'].flatten()

        if waypoints.shape[0] <= 1:
            print("Error: La trayectoria tiene 1 o menos puntos. Saltando.")
            continue

        print(f"Datos cargados: {waypoints.shape[0]} waypoints para {waypoints.shape[1]} juntas.")

        # --- 6.3. NUEVO: Seleccionar Perfil de Límites ---
        selected_profile_name = DEFAULT_PROFILE_NAME # Empezar con default
        
        for (filename_substring, profile_name) in PROFILE_MAP:
            if filename_substring in input_filename:
                selected_profile_name = profile_name
                break # Encontrado, salir del bucle de búsqueda

        print(f"Usando perfil de límites: '{selected_profile_name}'")

        # Obtener los límites para este perfil
        selected_limits = LIMIT_PROFILES[selected_profile_name]
        vlim_abs = selected_limits['vlim_abs']
        alim_abs = selected_limits['alim_abs']

        # Construir matrices de límites (como antes, pero localmente)
        vlim = np.vstack([-vlim_abs, vlim_abs]).T
        alim = np.vstack([-alim_abs, alim_abs]).T
        # ---------------------------------------------------

        # --- 6.4. BLOQUE: Suavizado condicional (sin cambios) ---
        if is_special_traj:
            print("*** Aplicando filtro de suavizado (Savitzky-Golay) a '10-11' ***")
            window_len = 51
            poly_order = 3
            
            if waypoints.shape[0] > window_len:
                try:
                    waypoints_suavizados = signal.savgol_filter(
                        waypoints, 
                        window_length=window_len, 
                        polyorder=poly_order, 
                        axis=0
                    )
                    waypoints = waypoints_suavizados
                    print(f"Suavizado completado con ventana={window_len}, orden={poly_order}")
                except Exception as e_filter:
                    print(f"ADVERTENCIA: Falló el filtro de suavizado: {e_filter}. Usando datos originales.")
            else:
                print("ADVERTENCIA: No se puede suavizar, la trayectoria es demasiado corta. Usando datos originales.")
        # ---------------------------------------------------

        # 6.5. Crear el objeto de trayectoria (Path)
        path = ta.SplineInterpolator(path_pos, waypoints)

        # 6.6. Crear las restricciones
        #    (¡Usa las variables vlim/alim locales seleccionadas arriba!)
        pc_vel = constraint.JointVelocityConstraint(vlim)
        pc_acc = constraint.JointAccelerationConstraint(alim)

        # 6.7. Instanciar y resolver el problema TOPPRA
        gpts = max(500, waypoints.shape[0] * 2) 
        instance = algo.TOPPRA([pc_vel, pc_acc], path,
                               gridpoints=np.linspace(path_pos[0], path_pos[-1], gpts))
        
        print("Calculando parametrización de tiempo óptima...")
        jnt_traj = instance.compute_trajectory(0, 0)

        # 6.8. Comprobar si TOPPRA falló
        if jnt_traj is None:
            print("Error: TOPPRA no pudo encontrar una solución. Saltando este archivo.")
            continue
        
        optimal_duration = jnt_traj.duration
        print(f"¡Trayectoria calculada! Duración ÓPTIMA (post-suavizado): {optimal_duration:.3f} segundos")

        # 6.9. Re-escalar para trayectoria 10-11 (sin cambios)
        final_duration = optimal_duration
        scale_factor = 1.0

        if is_special_traj:
            print(f"*** Trayectoria especial '10-11' detectada. Objetivo: {target_duration}s ***")
            if optimal_duration > target_duration:
                print(f"ADVERTENCIA: El tiempo óptimo ({optimal_duration:.3f}s) es MAYOR que el objetivo ({target_duration}s).")
                print("             Se guardará la trayectoria con el tiempo óptimo.")
                final_duration = optimal_duration
            else:
                print(f"Tiempo óptimo ({optimal_duration:.3f}s) es MENOR/IGUAL al objetivo.")
                print(f"             Re-escalando la trayectoria para que dure exactamente {target_duration}s.")
                final_duration = target_duration
                if final_duration > 0: 
                    scale_factor = optimal_duration / final_duration
        
        print(f"Duración FINAL para guardar: {final_duration:.3f} segundos")

        # 6.10. Muestrear la nueva trayectoria (sin cambios)
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

        # 6.11. Preparar datos para guardar (sin cambios)
        datos_para_matlab = {
            'ts_sample': ts_sample,
            'q_toppra': qs_new,
            'qd_toppra': qds_new,
            'qdd_toppra': qdds_new,
            'original_optimal_duration': optimal_duration,
            'final_duration': final_duration
        }
        
        # 6.12. Crear nombre de archivo de salida (sin cambios)
        output_filename = input_filename.replace('_rawtraj_', '_toppratraj_')
        output_filepath = os.path.join(OUTPUT_DIR, output_filename)
        
        # 6.13. Guardar el nuevo archivo .mat (sin cambios)
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