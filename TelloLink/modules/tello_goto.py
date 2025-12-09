# tello_goto.py - Actualizado 2025-12-09
# Movimiento fluido con comando 'go' + flecha de velocidad en tiempo real
from __future__ import annotations
import math
import threading
import time
from typing import Optional, Callable, Any
from TelloLink.modules.tello_move import MIN_STEP

#Parámetros ajustables
_MIN_BAT_PCT   = 20        #Batería mínima para realizar la operación
_STEP_XY_CM    = 25.0      #Paso horizontal
_STEP_Z_CM     = 20.0      #Paso vertical
_TOL_XY_CM     = 8         #Tolerancia horizontal
_TOL_Z_CM      = 8         #Tolerancia vertical
_SLEEP_S       = 0.10      #Pausa entre comandos
_MAX_RETRY_CMD = 2         #Reintentos de cada paso si falla

#Función que decide el tamaño de cada paso a realizar según lo que le queda por recorrer.
def _adaptive_step(rest: float, base: float, min_step: float = MIN_STEP) -> float:
    rest = abs(rest)  # Distancia restante en valor absoluto
    if rest <= min_step: #Si la distancia restante es mas pequeña que el mínimo paso permitido en Tello
        return rest #Devuelve lo que falta
    #Escala el paso según la distancia que queda: lejos = paso base, cerca = paso reducido. Si rest / (3.0 * base) es mayor que 1, aún le falta mucho y coge 1 que es el paso base
    scale = max(0.35, min(1.0, rest / (3.0 * base)))
    # Paso final, siempre >= min_step. Si el paso que hemos escalado es menor que el mínimo de Tello, daría error, por eso eligiriamos el mínimo de tello
    return max(min_step, base * scale)

#Función que hace girar al dron hasta el ángulo deseado
def _rotate_to_yaw(self, target_yaw_deg: float) -> bool:
    # Leer yaw actual desde pose
    if hasattr(self, "pose") and self.pose:
        curr = float(getattr(self.pose, "yaw_deg", 0.0) or 0.0)
    else:
        curr = float(getattr(self, "yaw_deg", 0.0) or 0.0)

    delta = (float(target_yaw_deg) - curr) % 360.0
    print(f"[rotate] curr={curr:.1f}°, target={target_yaw_deg:.1f}°, delta={delta:.1f}°")

    if delta == 0:
        print("[rotate] Ya en ángulo, no rota")
        return True

    if delta > 180.0:
        amt = int(round(360.0 - delta))
        print(f"[rotate] CCW {amt}°")
        resp = self.ccw(amt)
    else:
        amt = int(round(delta))
        print(f"[rotate] CW {amt}°")
        resp = self.cw(amt)

    ok = bool(str(resp).lower() == "ok" or resp is True)
    print(f"[rotate] resp={resp}, ok={ok}")

    # Actualizar pose.yaw_deg después de rotar
    if ok and hasattr(self, "pose") and self.pose:
        self.pose.yaw_deg = target_yaw_deg % 360.0

    return ok



#Esta función envía un mmovimiento al dron, espera la confirmación y actualiza la pose virtual si tuvo éxito
def _send_and_update(self, cmd: str, dist_cm: float) -> bool:
    dist_i = int(round(dist_cm))
    if dist_i <= 0: #Si la distancia del paso a realizar es 0, devuelve True
        return True
    for _ in range(_MAX_RETRY_CMD + 1): #Hacemos un bucle con el numero de "vueltas" (intentos) que son el inicial + el número de reintentos
        resp = getattr(self, cmd)(dist_i) #Ejecuta el movimiento
        ok = bool(str(resp).lower() == "ok" or resp is True) #Comprueba si el dron confirmó el movimiento
        if ok: #Si se ejecutó correctamente
            try:
                if hasattr(self, "pose") and self.pose: #Comprueba que haya el atributo pose exista
                    self.pose.update_move(cmd, dist_i) #Si existe, actualiza las coordenadas
            except Exception:
                pass
            return True
        time.sleep(0.05)
    return False


def _goto_rel_worker(self,
                     dx_cm: float, dy_cm: float, dz_cm: float = 0.0,
                     yaw_deg: Optional[float] = None,
                     speed_cm_s: Optional[float] = None,
                     face_destination: bool = False,
                     callback: Optional[Callable[..., Any]] = None,
                     params: Any = None) -> None:
    """
    Worker que usa el comando 'go' para movimiento fluido y directo.
    El comando 'go x y z speed' mueve el dron en línea recta al punto relativo.
    Actualiza la pose en tiempo real para visualización suave en el mapa.
    """
    #Chequeos básicos previos
    if not hasattr(self, "pose") or self.pose is None:
        print("[goto] No hay PoseVirtual; abortando.")
        return
    if getattr(self, "state", "") == "disconnected":
        print("[goto] Dron desconectado; abortando.")
        return
    bat = getattr(self, "battery_pct", None)
    if isinstance(bat, int) and bat < _MIN_BAT_PCT:
        print(f"[goto] Batería baja ({bat}%), abortando.")
        return

    #Si el dron no está volando, hace un despegue seguro
    if getattr(self, "state", "") != "flying":
        ok = self.takeOff(0.5, blocking=True)
        if not ok:
            print("[goto] No se pudo despegar.")
            return
        time.sleep(0.4)

    # Velocidad por defecto 50 cm/s (rango Tello: 10-100)
    speed = int(speed_cm_s) if speed_cm_s else 50
    speed = max(10, min(100, speed))

    # Si face_destination está activado, rotar para mirar al destino
    if face_destination and (dx_cm != 0 or dy_cm != 0):
        target_angle = math.degrees(math.atan2(dy_cm, dx_cm))
        print(f"[goto] face_destination: rotando a {target_angle:.1f}° (yaw actual: {self.pose.yaw_deg:.1f}°)")
        if not _rotate_to_yaw(self, target_angle):
            print("[goto] Error al rotar hacia destino.")
            return
        print(f"[goto] Rotación completada. Nuevo yaw: {self.pose.yaw_deg:.1f}°")
        time.sleep(0.3)

    # Convertir desplazamiento mundo a coordenadas relativas al dron
    # El comando 'go' usa: x=forward, y=left, z=up (relativo al heading del dron)
    yaw_rad = math.radians(getattr(self.pose, "yaw_deg", 0.0) or 0.0)
    cos_yaw = math.cos(yaw_rad)
    sin_yaw = math.sin(yaw_rad)

    # Rotar del sistema mundo al sistema del dron
    # forward = componente en dirección del heading
    # left = componente perpendicular (positivo = izquierda)
    forward_cm = dx_cm * cos_yaw + dy_cm * sin_yaw
    left_cm = -dx_cm * sin_yaw + dy_cm * cos_yaw
    up_cm = dz_cm

    # El comando go tiene límites: 20-500 cm por eje (valores menores de 20 se ignoran)
    # Si alguna distancia es muy pequeña, usar comandos individuales
    dist_xy = math.hypot(forward_cm, left_cm)
    dist_total = math.hypot(dist_xy, up_cm)

    if dist_total < 20:
        print("[goto] Distancia muy pequeña, ya en destino.")
        if callback:
            try: callback(params)
            except: pass
        return

    # Dividir en segmentos si la distancia es > 500cm
    MAX_GO_DIST = 450  # Un poco menos del límite para seguridad
    num_segments = max(1, int(math.ceil(dist_total / MAX_GO_DIST)))

    seg_forward = forward_cm / num_segments
    seg_left = left_cm / num_segments
    seg_up = up_cm / num_segments

    for seg in range(num_segments):
        if getattr(self, "_goto_abort", False):
            print("[goto] Abortado por solicitud externa.")
            self.pose.vx = 0.0
            self.pose.vy = 0.0
            self.pose.vz = 0.0
            return

        # Verificar batería
        bat = getattr(self, "battery_pct", None)
        if isinstance(bat, int) and bat < _MIN_BAT_PCT:
            print(f"[goto] Abortado por batería ({bat}%).")
            self.pose.vx = 0.0
            self.pose.vy = 0.0
            self.pose.vz = 0.0
            return

        # Preparar valores para el comando go (deben ser enteros, mínimo 20 o 0)
        x_cmd = int(round(seg_forward))
        y_cmd = int(round(seg_left))
        z_cmd = int(round(seg_up))

        # El Tello ignora valores < 20, así que si son pequeños, poner 0
        if abs(x_cmd) < 20: x_cmd = 0
        if abs(y_cmd) < 20: y_cmd = 0
        if abs(z_cmd) < 20: z_cmd = 0

        # Si todo es 0, saltar
        if x_cmd == 0 and y_cmd == 0 and z_cmd == 0:
            continue

        print(f"[goto] Segmento {seg+1}/{num_segments}: go {x_cmd} {y_cmd} {z_cmd} {speed}")

        # Calcular desplazamiento en coordenadas mundo para este segmento
        dx_world = seg_forward * cos_yaw - seg_left * sin_yaw
        dy_world = seg_forward * sin_yaw + seg_left * cos_yaw
        dz_world = seg_up

        # Calcular tiempo estimado del movimiento
        seg_dist = math.hypot(math.hypot(abs(x_cmd), abs(y_cmd)), abs(z_cmd))
        estimated_time = seg_dist / speed if speed > 0 else 1.0

        # Guardar posición inicial
        start_x = self.pose.x_cm
        start_y = self.pose.y_cm
        start_z = self.pose.z_cm

        # Flag para controlar el hilo de actualización
        movement_done = threading.Event()

        # Calcular velocidad en coordenadas mundo (para flecha de dirección)
        if estimated_time > 0:
            vel_x_world = dx_world / estimated_time  # cm/s en dirección X mundo
            vel_y_world = dy_world / estimated_time  # cm/s en dirección Y mundo
            vel_z_world = dz_world / estimated_time  # cm/s en dirección Z mundo
        else:
            vel_x_world = vel_y_world = vel_z_world = 0.0

        # Función para actualizar pose en tiempo real
        def update_pose_realtime():
            UPDATE_INTERVAL = 0.1  # Actualizar cada 100ms
            elapsed = 0.0
            while not movement_done.is_set() and elapsed < estimated_time + 1.0:
                progress = min(1.0, elapsed / estimated_time) if estimated_time > 0 else 1.0
                # Actualizar pose interpolada
                self.pose.x_cm = start_x + dx_world * progress
                self.pose.y_cm = start_y + dy_world * progress
                self.pose.z_cm = start_z + dz_world * progress
                # Actualizar velocidad para flecha de dirección de movimiento
                self.pose.vx = vel_x_world
                self.pose.vy = vel_y_world
                self.pose.vz = vel_z_world
                time.sleep(UPDATE_INTERVAL)
                elapsed += UPDATE_INTERVAL

        # Iniciar hilo de actualización en tiempo real
        update_thread = threading.Thread(target=update_pose_realtime, daemon=True)
        update_thread.start()

        # Enviar comando go
        try:
            # Intentar usar el método go_xyz_speed si existe
            if hasattr(self, 'go_xyz_speed'):
                resp = self.go_xyz_speed(x_cmd, y_cmd, z_cmd, speed)
            elif hasattr(self, '_send'):
                # Usar método interno _send de TelloDron
                resp = self._send(f"go {x_cmd} {y_cmd} {z_cmd} {speed}")
            elif hasattr(self, '_tello') and hasattr(self._tello, 'send_control_command'):
                # Fallback: usar djitellopy directamente
                resp = self._tello.send_control_command(f"go {x_cmd} {y_cmd} {z_cmd} {speed}")
            else:
                raise RuntimeError("No se encontró método para enviar comando go")

            # Señalar que el movimiento terminó
            movement_done.set()
            update_thread.join(timeout=0.5)

            # Verificar respuesta (puede mezclarse con otros comandos como battery?)
            ok = (str(resp).lower() == "ok" or resp is True)
            if not ok:
                # Respuesta inesperada, pero el movimiento probablemente ocurrió
                # (el SDK mezcla respuestas cuando hay queries concurrentes)
                print(f"[goto] Respuesta inesperada: {resp} (asumiendo movimiento completado)")

            # Siempre actualizar pose final (el dron se movió si no hubo excepción)
            self.pose.x_cm = start_x + dx_world
            self.pose.y_cm = start_y + dy_world
            self.pose.z_cm = start_z + dz_world
            # Limpiar velocidad (movimiento completado)
            self.pose.vx = 0.0
            self.pose.vy = 0.0
            self.pose.vz = 0.0
            print(f"[goto] Movimiento OK. Pose: ({self.pose.x_cm:.1f}, {self.pose.y_cm:.1f}, {self.pose.z_cm:.1f})")

        except Exception as e:
            movement_done.set()
            # Limpiar velocidad en caso de error
            self.pose.vx = 0.0
            self.pose.vy = 0.0
            self.pose.vz = 0.0
            print(f"[goto] Error en comando go: {e}")
            return

        time.sleep(0.2)  # Pequeña pausa entre segmentos

    # Rotación final si se especificó yaw_deg
    if yaw_deg is not None:
        if not _rotate_to_yaw(self, float(yaw_deg)):
            print("[goto] Error en rotación final.")
        time.sleep(0.1)

    # Asegurar que la velocidad esté en 0 al finalizar
    self.pose.vx = 0.0
    self.pose.vy = 0.0
    self.pose.vz = 0.0

    print("[goto] Objetivo alcanzado.")
    if callback:
        try: callback(params)
        except TypeError:
            try: callback()
            except Exception: pass

#Funcion pública
def goto_rel(self,
             dx_cm: float, dy_cm: float, dz_cm: float = 0.0,
             yaw_deg: Optional[float] = None,
             speed_cm_s: Optional[float] = None,
             face_destination: bool = False,
             blocking: bool = True,
             callback: Optional[Callable[..., Any]] = None,
             params: Any = None) -> None:
    """
    Navega a una posición relativa.

    Args:
        dx_cm, dy_cm, dz_cm: Desplazamiento relativo en cm
        yaw_deg: Ángulo de orientación final (opcional)
        speed_cm_s: Velocidad en cm/s (opcional)
        face_destination: Si True, el dron rota para mirar hacia el destino antes de moverse
        blocking: Si True, espera a que termine el movimiento
        callback: Función a llamar al terminar
        params: Parámetros para el callback
    """
    setattr(self, "_goto_abort", False)

    t = threading.Thread(
        target=_goto_rel_worker,
        args=(self, dx_cm, dy_cm, dz_cm, yaw_deg, speed_cm_s, face_destination, callback, params),
        daemon=True
    )
    t.start()
    if blocking:
        t.join()

def abort_goto(self) -> None:
    setattr(self, "_goto_abort", True)