import websocket
import json
import random
import os
import numpy as np
import csv

# --- Parámetros ---
URL_SERVIDOR = "wss://ws.davinsony.com/city_2"
ACTOR_ID = "actor_semaforo_1"
BRAIN_ID = "Brain_RL"
CANAL_WEBSOCKET = "city_2"

# --- Parámetros de Q-learning ---
alpha = 0.1      # tasa de aprendizaje
gamma = 0.9      # factor de descuento
epsilon = 0.1    # política ε-greedy exploración
epsilon_min = 0.01
epsilon_decay = 0.995 # Multiplicamos en cada iteración
Q_TABLE_FILE = "q_table.json"
REPORT_CSV_FILE = "reporte_aprendizaje.csv"
recompensas = []  # Lista global para acumular recompensas

# --- Estados y Acciones ---
# El estado ahora es: (estadoMEF, peticionPeaton, hayTraficoEsperando)
states = [(s, p, t) for s in range(6) for p in [0, 1] for t in [0, 1]]
n_actions = 3  # 0: SEGUIR_CICLO, 1: ATENDER_PEATON, 2: FORZAR_CAMBIO
state_to_index = {s: i for i, s in enumerate(states)}
n_states = len(states)
Q = np.zeros((n_states, n_actions))
last_state = None
last_action = None
passage_time = 0
modo_lluvia = False # por defecto, no está lloviendo
ws_actor = None # Guardaremos una referencia al websocket del actor

nombres_estados_mef = {0:"S1 Verde", 1:"S1 Amarillo", 2:"S2 Verde", 3:"S2 Amarillo", 4:"Peatones S1", 5:"Peatones S2"}
nombres_acciones = {0:"SEGUIR_CICLO", 1:"ATENDER_PEATON", 2:"FORZAR_CAMBIO"}

def get_status(estado_actor):
    estado_mef = estado_actor.get("estadoMEF", 0)
    peticion_peaton = 1 if estado_actor.get("peticionPeatonS1") or estado_actor.get("peticionPeatonS2") else 0
    
    # Verificamos si hay tráfico esperando en la luz roja
    trafico_s1 = estado_actor.get("traficoS1_peso", 0)
    trafico_s2 = estado_actor.get("traficoS2_peso", 0)
    hay_trafico_esperando = 0
    
    if estado_mef in [0, 1] and trafico_s2 > 0:
        hay_trafico_esperando = 1
    elif estado_mef in [2, 3] and trafico_s1 > 0:
        hay_trafico_esperando = 1
        
    return (estado_mef, peticion_peaton, hay_trafico_esperando)

# --- Funciones de Q-Learning ---
def cargar_q_table():
    global Q
    if os.path.exists(Q_TABLE_FILE):
        Q_cargada = np.loadtxt(Q_TABLE_FILE)
        if Q_cargada.shape == (n_states, n_actions):
            Q[:] = Q_cargada
            print("Q-table cargada con forma:", Q.shape)
        else:
            print(f"Q-table inválida (esperado {n_states}x{n_actions}, obtenido {Q_cargada.shape}). Se reinicia.")
    else:
        print("No se encontró Q-table. Se usará una nueva tabla vacía.")

def guardar_q_table():
    np.savetxt(Q_TABLE_FILE, Q)
    print("Tabla-Q guardada.")

def calcular_recompensa(estado_ant, accion_idx):
    _, peticion_peaton_ant, trafico_esperando_ant = estado_ant

    # >> PRIORIDAD 1: Gestionar el tráfico es lo más importante.
    if trafico_esperando_ant:
        if accion_idx == 2: # FORZAR_CAMBIO
            return 25.0  # Recompensa máxima por aliviar el trafico.
        if accion_idx == 0: # SEGUIR_CICLO
            return -20.0 # Castigo muysevero por ignorar la congestion.
        if accion_idx == 1: # ATENDER_PEATON
            return 5.0   # Recompensa pequeña. No es lo ideal, pero ejecutó algo.

    # >> PRIORIDAD 2: Si no hay tráfico, atender al peatón es lo siguiente más importante.
    elif peticion_peaton_ant:
        if accion_idx == 1: # ATENDER_PEATON
            return 20.0  # Gran recompensa por atender al peatón correctamente.
        if accion_idx == 0: # SEGUIR_CICLO
            return -15.0 # Castigo severo por ignorar al peatón.
        if accion_idx == 2: # FORZAR_CAMBIO
            return -5.0  # Castigo por una acción innecesaria (no había tráfico).

    # >> PRIORIDAD 3: Estrategia especial para el MODO LLUVIA (si no hay emergencias)
    elif modo_lluvia:
        print("[ESTRATEGIA] Aplicando recompensas de MODO LLUVIA.")
        if accion_idx == 0: # SEGUIR_CICLO
            return 10.0 # En lluvia, se premia mantener un ciclo estable y predecible.
        else: # ATENDER_PEATON o FORZAR_CAMBIO
            return -15.0 # En lluvia, se castigan las interrupciones bruscas.

    # >> PRIORIDAD 4: Si no hay nadie esperando, el ciclo normal es lo correcto.
    else: # No hay ni tráfico ni peatones
        if accion_idx == 0: # SEGUIR_CICLO
            return 5.0   # Recompensa por mantener el ciclo eficientemente.
        else: # ATENDER_PEATON o FORZAR_CAMBIO
            return -10.0 # Castigo por interrupciones innecesarias.
        

def report_init_csv():
    # Creamos el archivo en modo escritura ('w') para empezar de cero cada vez
    with open(REPORT_CSV_FILE, mode='w', newline='') as file:
        writer = csv.writer(file)
        # Escribimos la fila de encabezados
        writer.writerow([
            "Paso de Tiempo", 
            "Estado Anterior", 
            "Accion Tomada", 
            "Recompensa"
        ])
    print(f"Reporte '{REPORT_CSV_FILE}' inicializado.")

def write_report(paso, estado_ant, accion_idx, recompensa):
    # Abrimos el archivo en modo 'append' ('a') para añadir datos sin borrar lo anterior
    with open(REPORT_CSV_FILE, mode='a', newline='') as file:
        writer = csv.writer(file)
        # Traducimos la acción numérica a texto para que sea más claro en el reporte
        accion_str = nombres_acciones.get(accion_idx, "DESCONOCIDA")
        
        # Escribimos la nueva fila de datos
        writer.writerow([
            paso, 
            str(estado_ant), 
            accion_str, 
            f"{recompensa:.1f}"
        ])

def on_message(ws, message):
    global last_state, last_action, Q, epsilon, passage_time, modo_lluvia
    ws_actor = ws # Guardamos la conexión para poder enviar comandos estratégicos

    try:
        datos_recibidos = json.loads(message)
        
        # ESCUCHAR AL SUPERVISOR
        if datos_recibidos.get("type") == "STRATEGIC_COMMAND":
            comando = datos_recibidos.get("comando")
            print(f"\n[!!!] COMANDO ESTRATÉGICO RECIBIDO: {comando} [!!!]\n")
            
            nuevo_modo_lluvia = (comando == "MODO_LLUVIA_ON")
            
            # Si el modo cambió, se lo informamos al actor
            if nuevo_modo_lluvia != modo_lluvia:
                modo_lluvia = nuevo_modo_lluvia
                # Reenviamos el comando estratégico al actor físico
                # if ws_actor and ws_actor.sock and ws_actor.sock.connected:
                #     ws_actor.send(json.dumps(datos_recibidos))
                #     print(f"--> Cerebro Táctico reenviando comando '{comando}' al actor.")
            return
        
        # ESTADOS DEL ACTOR
        if datos_recibidos.get("type") == "ACTOR_STATE":
            
            # 1. OBSERVAR
            estado_actual = get_status(datos_recibidos)
            # 2. APRENDER (basado en la transición)
            if last_state is not None:
                s0_idx = state_to_index[last_state]
                s1_idx = state_to_index[estado_actual]
                recompensa = calcular_recompensa(last_state, last_action)
                valor_antiguo = Q[s0_idx][last_action]
                proximo_mejor_valor = np.max(Q[s1_idx])
                nuevo_valor = valor_antiguo + alpha * (recompensa + gamma * proximo_mejor_valor - valor_antiguo)
                Q[s0_idx][last_action] = nuevo_valor
                
                passage_time += 1
                write_report(passage_time, last_state, last_action, recompensa)
                # --- BRAIN LOGS---
                print(f"Estado Ant: {last_state} -> Nuevo: {estado_actual} | Acción: {last_action}, Recompensa: {recompensa:.1f}")
                recompensas.append(recompensa)
            # 3. DECAY de epsilon (reducir la exploración poco a poco)
            if epsilon > epsilon_min:
                epsilon *= epsilon_decay
                print(f"[ε] Epsilon actualizado: {epsilon:.4f}")
            if len(recompensas) >= 100 and len(recompensas) % 10 == 0:
                promedio = np.mean(recompensas[-100:])
                print(f"Promedio últimas 100 recompensas: {promedio:.2f}")
            # 4. DECIDIR
            idx_actual = state_to_index[estado_actual]
            if random.random() < epsilon:
                action = random.choice([0, 1, 2])
            else:
                action = int(np.argmax(Q[idx_actual]))
            if action == 1:
                accion_str = "ATENDER_PEATON"
            elif action == 2:
                accion_str = "FORZAR_CAMBIO"
            else:
                accion_str = "SEGUIR_CICLO"
            print(f"Decisión del {BRAIN_ID} : {accion_str}")
            # 5. ACTUAR
            if accion_str != "SEGUIR_CICLO":
                mensaje_respuesta = {
                    "from": BRAIN_ID,
                    "to": CANAL_WEBSOCKET,
                    "type": "BRAIN_COMMAND",
                    "msg": accion_str
                }
                ws.send(json.dumps(mensaje_respuesta))
                print(f"--> {BRAIN_ID} Enviando Comando: {accion_str}\n")
            # 6. MEMORIZAR ESTADO Y ACCIÓN
            last_state = estado_actual
            last_action = action
            
                
    except Exception as e:
        print(f"Ocurrió un error en on_message: {e}")

def on_error(ws, error): print(f"Error de WebSocket: {error}")
def on_close(ws, _, __): print("\n### Conexión cerrada ###"); guardar_q_table()
def on_open(ws): print("Cerebro conectado y aprendiendo.")

if __name__ == "__main__":
    cargar_q_table()
    report_init_csv()
    ws = websocket.WebSocketApp(URL_SERVIDOR,
                              on_open=on_open,
                              on_message=on_message,
                              on_error=on_error,
                              on_close=on_close)
    ws.run_forever(ping_interval=20, ping_timeout=5)