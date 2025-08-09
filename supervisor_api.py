import requests
import json
import time
import websocket
import threading
import sys

# --- CONFIGURACIÓN ---
API_KEY = "889b43e71ff5b89453df286bd00e2a7d"  # API KEY OPENWEATHERMAP
# Medellin,CO, Cairo,EG, London,UK, Moscow,RU, Cherrapunji,IN . . .
CIUDAD = "Medellin,CO"
URL_API = f"http://api.openweathermap.org/data/2.5/weather?q={CIUDAD}&APPID={API_KEY}&units=metric"

# Usamos el MISMO servidor WebSocket, pero un CANAL DIFERENTE para la comunicación estratégica
URL_WEBSOCKET_CEREBRO = "wss://ws.davinsony.com/city_2"
SUPERVISOR_ID = "Supervisor_Clima_GrupoX"
INTERVALO_CONSULTA = 60  # Consulta el clima cada (n segundos)


def obtener_clima():
    """Consulta la API de OpenWeatherMap y devuelve la condición principal del clima."""
    try:
        response = requests.get(URL_API)
        response.raise_for_status()  # Lanza un error si la petición HTTP falla
        datos_clima = response.json()
        condicion_principal = datos_clima['weather'][0]['main']
        temperatura = datos_clima['main']['temp']
        print(f"[{time.strftime('%H:%M:%S')}] Clima actual en {CIUDAD}: {condicion_principal}, {temperatura}°C")
        return condicion_principal
    except Exception as e:
        print(f"Error consultando API: {e}")
        return None


def enviar_comando_estrategico(ws, comando, clima_actual):
    """Envía un comando al cerebro táctico y al actor."""
    mensaje = {
        "from": SUPERVISOR_ID,
        "to": "city_2",  # Envía a todos en el canal
        "type": "STRATEGIC_COMMAND",
        "comando": comando,
        "clima_info": clima_actual  # Añadimos info extra para logs
    }
    # Verificamos que el websocket esté vivo antes de enviar
    if ws and ws.sock and ws.sock.connected:
        ws.send(json.dumps(mensaje))
        print(
            f"--> Supervisor enviando comando: {comando} (debido a clima: {clima_actual})\n")
    else:
        print("Supervisor no conectado, no se puede enviar comando.")


def on_open_supervisor(ws):
    print(
        f"Supervisor conectado al canal {URL_WEBSOCKET_CEREBRO}. Iniciando monitoreo climático...")


def on_error(ws, error):
    print(f"Error de supervisor: {error}")


def on_close(ws, _, __):
    print("Conexión de supervisor cerrada.")


def iniciar_supervisor(ws):
    """Bucle principal que consulta el clima y envía comandos."""
    while True:
        if not ws.sock or not ws.sock.connected:
            print("Supervisor esperando conexión...")
            time.sleep(5)
            continue

        clima = obtener_clima()
        if clima:
            if clima.lower() in ["rain", "drizzle", "thunderstorm", "snow"]:
                enviar_comando_estrategico(ws, "MODO_LLUVIA_ON", clima)
            else:
                enviar_comando_estrategico(ws, "MODO_LLUVIA_OFF", clima)

        time.sleep(INTERVALO_CONSULTA)


if __name__ == "__main__":
    ws = websocket.WebSocketApp(
        URL_WEBSOCKET_CEREBRO, on_open=on_open_supervisor, on_error=on_error, on_close=on_close)
    ws_thread = threading.Thread(target=ws.run_forever)
    ws_thread.daemon = True
    ws_thread.start()

    time.sleep(3)

    try:
        iniciar_supervisor(ws)
    except KeyboardInterrupt:
        print("\nCerrando supervisor...")
        ws.close()
        sys.exit(0)
