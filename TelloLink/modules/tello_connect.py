import threading
import time
from djitellopy import Tello


def _connect(self, freq=5, callback=None, params=None):
    try:
        # Crea el objeto Tello y conecta
        self._tello = Tello()
        self._tello.connect()

        # A veces el stream de video se queda colgado de sesiones anteriores (bug común en Tello) ; lo apagamos preventivamente
        try:
            self._tello.streamoff()
        except Exception:
            pass

        # Estado del Tello pasa a ser conectado
        self.state = "connected"
        self.frequency = freq

        # POSE: asegurar objeto y sincronizar Z con barómetro (si ya hubiera altura)
        try:
            if not hasattr(self, "pose") or self.pose is None:

                self.pose = self.PoseVirtual()
            # Sincroniza Z con la altura disponible (si aún no hay telemetría, simplemente quedará en 0.0)
            self.pose.set_from_telemetry(height_cm=getattr(self, "height_cm", None))
        except Exception:
            pass

        return True

    except Exception as Err: #Si ocurre cualquier error en el try anterior, lo capturamos en la variable 'Err'
        print("Error conectando a Tello:", Err) # Se muestra un mensaje de error con el detalle (texto del error real)
        self._tello = None # No hay ningún objeto tello
        self.state = "disconnected" # El estado del Tello es Desconectado
        return False  # Se devuelve False para avisar de que no se ha podido conectar



#Esta función llama a la función anterior _connect. Con blocking True, la conexión se hace inmediatamente en el mismno hilo
def connect(self, freq=5, blocking=True, callback=None, params=None):

    if self.state != "disconnected":
        # Ya estaba conectado o en otro estado; no se va a reconectar
        return False

    self.frequency = freq  # guardamos la frecuencia

    if blocking: #Si blocking=True, se llama directamente a la función previa para poder conectarse
        return _connect(self, freq=freq, callback=callback, params=params)
    else: #Si blocking=False, se crea un hilo un segundo plano, cuyo trabajo es ejecutar _connect
        t = threading.Thread(
            target=_connect, args=(self,), kwargs=dict(freq=freq, callback=callback, params=params), daemon=True
        )
        t.start()
        return True

def disconnect(self):
    #Primero intentamos parar telemetría. Si la telemetría devuelve True, se va a parar, si no existe, en vez de error, devuelve False
    try:
        if getattr(self, "sendTelemetryInfo", False):
            self.stop_sending_telemetry_info()
    except Exception:
        pass

    # Cierra conexión con el dron. Primero intenta apagar el stream, y si falla, lo ignora y cierra la sesión.
    try:
        if self._tello:
            try:
                self._tello.streamoff()
            except Exception:
                pass
            self._tello.end()
    finally:
        self._tello = None
        self.state = "disconnected"

    # Pequeña pausa de seguridad
    time.sleep(0.2)
    return True



#Con esta función, antes de mandar cualquier comando al dron nos aseguramos que se creó el objeto Tello()
def _require_connected(self):
    if not hasattr(self, "_tello") or self._tello is None:
        raise RuntimeError("No hay backend '_tello' inicializado. ¿Llamaste connect()?")


#Función en la que se mendan los comandos que se tengan que mandar al dron, y se responde al ususario
def _send(self, cmd: str) -> str:
    _require_connected(self)

    # djitellopy expone distintos nombres según versión; probamos varios:
    if hasattr(self._tello, "send_read_command"):
        resp = self._tello.send_read_command(cmd)
        return str(resp)

    if hasattr(self._tello, "send_command_with_return"):
        resp = self._tello.send_command_with_return(cmd)
        return str(resp)

    if hasattr(self._tello, "send_control_command"):
        res = self._tello.send_control_command(cmd)
        if isinstance(res, bool):
            return "ok" if res else "error"
        return str(res)

    raise RuntimeError("El backend Tello no soporta envío textual en esta versión.")