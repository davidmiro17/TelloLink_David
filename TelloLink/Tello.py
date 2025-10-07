class TelloDron(object):
    def __init__(self, id=None):
        print("TelloDron inicializado")


        #A continuación, se van a escribir los atributos del objeto TelloDron. Muchos de estos no se van a utilizar por ahora, pues segun mis primeras investigaciones, no serían compatibles con Tello
        #pero los dejo para seguir el estilo de la librería DroneLink
        self.id = id
        self.state = "disconnected" #Los otros estados posibles en TelloDron: connected, takingOff, flying y landing
        self.lat = None  # No disponible en Tello
        self.lon = None  # No disponible en Tello
        self.alt = 0     # altura inicial = 0 m (sí existe)
        self.groundSpeed = 0  # velocidad inicial = 0 cm/s
        self.frequency = None #Se usa para la telemetría, pero usaremos directamente período

        self.going = False
        self.navSpeed = 20       #Estos tres atributos son de NAVLink, pero los dejamos para modificar lo menos posible el estilo
        self.direction = 'Stop'  #de la librería inicial DroneLink

        self.sendTelemetryInfo = False

        self.step = 50           # cm por paso
        self.speeds = [0, 0, 0]  # velocidades vx, vy, vz (cm/s)     #Estos tres atributos, inicialmente, no se van a usar dada su incompatibilidad
        self.lastDirection = None  # última dirección usada

        self.message_handler = None  # No aplica en Tello (no hay MAVLink)

        self.distance = None     # No disponible en Tello (no hay GPS)


        self._tello = None
        self._log_thread = None
        self._log_run = False

        #Valores de Tello que vamos a usar en diferentes nódulos
        self.battery = 0        # nivel de batería (%)
        self.flight_time = 0    # tiempo de vuelo acumulado (s)
        self.temperature = 0    # temperatura interna (°C)

        #Nuevos atributos del módulo Telemetría y otros, son atributos que se van actualizando y el dron envía al usuario
        self.height_cm = 0             # altura en cm
        self.battery_pct = None        # batería en %
        self.temp_c = None             # temperatura °C (float)
        self.wifi = None               # calidad enlace de conexión (0..100 aprox)
        self.flight_time_s = 0         # tiempo de vuelo en s
        self.telemetry_ts = None       # marca temporal última actualización
        self.FRAME_FORMAT = "RGB"  # Formato por defecto de los frames de cámara

    # --- Métodos "colgados" desde los módulos ---
    from TelloLink.modules.tello_camera import stream_on, stream_off, get_frame, snapshot
    from TelloLink.modules.tello_connect import connect, _connect, disconnect, _send, _require_connected
    from TelloLink.modules.tello_takeOff import takeOff, _takeOff, _checkAltitudeReached, _ascend_to_target
    from TelloLink.modules.tello_land import Land, _land
    from TelloLink.modules.tello_telemetry import startTelemetry, stopTelemetry
    from TelloLink.modules.tello_move import _move, up, down, set_speed
    from TelloLink.modules.tello_move import _move, forward, back, left, right
    from TelloLink.modules.tello_heading import rotate, cw, ccw
    from TelloLink.modules.tello_video import start_video, stop_video, show_video_blocking
