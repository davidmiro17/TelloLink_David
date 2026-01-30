from __future__ import annotations
import math
import threading
import time
from typing import Optional, Callable, Any

# Parámetros ajustables
_MIN_BAT_PCT = 20       # Batería mínima para realizar la operación
_MAX_GO_CM = 500        # Máximo desplazamiento por comando 'go' (límite SDK)
_MIN_GO_CM = 20         # Mínimo desplazamiento (límite SDK)
_DEFAULT_SPEED = 50     # Velocidad por defecto en cm/s
_TOL_XY_CM = 10         # Tolerancia horizontal
_TOL_Z_CM = 15          # Tolerancia vertical

#Función para rotar el dron hasta el angulo deseado
def _rotate_to_yaw(self, target_yaw_deg: float) -> bool:

    pose = getattr(self, "pose", None)
    curr = float(getattr(pose, "yaw_deg", 0.0) or 0.0) if pose else 0.0
    delta = (float(target_yaw_deg) - curr) % 360.0

    if delta < 1.0 or delta > 359.0:  # Tolerancia de 1 grado
        return True

    if delta > 180.0:
        amt = int(round(360.0 - delta))
        if amt < 1:
            return True
        resp = self.ccw(amt)
    else:
        amt = int(round(delta))
        if amt < 1:
            return True
        resp = self.cw(amt)

    return bool(str(resp).lower() == "ok" or resp is True)


def _world_to_local(dx_world: float, dy_world: float, yaw_deg: float) -> tuple:

    yaw_rad = math.radians(yaw_deg)
    cos_y = math.cos(yaw_rad)
    sin_y = math.sin(yaw_rad)

    # Rotación inversa: del mundo al local
    x_local = dx_world * cos_y + dy_world * sin_y   # forward/back
    y_local = -dx_world * sin_y + dy_world * cos_y  # left/right

    print(f"[_world_to_local] yaw={yaw_deg:.1f}° → world({dx_world:.1f}, {dy_world:.1f}) → local({x_local:.1f}, {y_local:.1f})")

    return x_local, y_local


def _send_go_command(self, x_local: int, y_local: int, z: int, speed: int):

    # Asegurar que los valores estén en rango
    x_local = max(-_MAX_GO_CM, min(_MAX_GO_CM, x_local))
    y_local = max(-_MAX_GO_CM, min(_MAX_GO_CM, y_local))
    z = max(-_MAX_GO_CM, min(_MAX_GO_CM, z))
    speed = max(10, min(100, speed))

    # Calcular tiempo estimado de movimiento
    distance = math.sqrt(x_local**2 + y_local**2 + z**2)
    estimated_time = distance / speed if speed > 0 else 0

    cmd = f"go {x_local} {y_local} {z} {speed}"

    try:
        resp = self._send(cmd)
        resp_lower = str(resp).lower().strip()

        # Si la respuesta es numérica (batería mezclada), necesitamos esperar
        is_numeric = False
        try:
            int(resp)
            is_numeric = True
        except ValueError:
            pass

        if is_numeric:
            # Respuesta numérica = probable race condition con batería
            wait_time = max(0.5, estimated_time + 0.5)
            print(f"[goto] Respuesta numérica ({resp}); posible colisión con battery? keepalive")
            print(f"[goto] Movimiento en progreso (~{wait_time:.1f}s)...")
            return (True, wait_time)

        # "ok" puede significar:
        # 1. Comando aceptado y ejecutándose
        # 2. Error silencioso donde el dron no se mueve
        # Para asegurar que el movimiento se complete, siempre esperamos el tiempo estimado
        if resp_lower == "ok":
            wait_time = max(0.5, estimated_time + 0.3)
            print(f"[goto] Respuesta 'ok', esperando movimiento (~{wait_time:.1f}s)...")
            return (True, wait_time)

        # Respuesta inesperada real
        print(f"[goto] Respuesta inesperada: {resp}")
        return (False, 0)
    except Exception as e:
        print(f"[goto] Error enviando comando: {e}")
        return (False, 0)


def _goto_rel_worker(self,
                     dx_cm: float, dy_cm: float, dz_cm: float = 0.0,
                     yaw_deg: Optional[float] = None,
                     speed_cm_s: Optional[float] = None,
                     face_target: bool = False,
                     callback: Optional[Callable[..., Any]] = None,
                     params: Any = None,
                     result_holder: Optional[dict] = None) -> None:
    """
    Worker que ejecuta el movimiento usando el comando 'go x y z speed'.
    """
    # Marcar que goto está en progreso (evita double-update con telemetry)
    setattr(self, "_goto_in_progress", True)

    ok = False
    try:
        ok = _goto_rel_worker_impl(self, dx_cm, dy_cm, dz_cm, yaw_deg, speed_cm_s,
                                   face_target, callback, params)
    finally:
        setattr(self, "_goto_in_progress", False)
        if result_holder is not None:
            result_holder["ok"] = ok


def _get_real_yaw(self) -> Optional[float]:
    """Lee el yaw REAL del dron desde la pose (actualizada por telemetría)."""
    try:
        # Leer yaw relativo desde pose (actualizado por telemetría)
        if hasattr(self, "pose") and self.pose is not None:
            yaw = getattr(self.pose, "yaw_deg", None)
            if yaw is not None:
                return float(yaw)
    except Exception as e:
        print(f"[goto] Error leyendo yaw real: {e}")
    return None


def _goto_rel_worker_impl(self,
                          dx_cm: float, dy_cm: float, dz_cm: float = 0.0,
                          yaw_deg: Optional[float] = None,
                          speed_cm_s: Optional[float] = None,
                          face_target: bool = False,
                          callback: Optional[Callable[..., Any]] = None,
                          params: Any = None) -> bool:

    # Chequeos básicos previos
    if not hasattr(self, "pose") or self.pose is None:
        print("[goto] No hay PoseVirtual; abortando.")
        return False
    if getattr(self, "state", "") == "disconnected":
        print("[goto] Dron desconectado; abortando.")
        return False
    bat = getattr(self, "battery_pct", None)
    if isinstance(bat, int) and bat < _MIN_BAT_PCT:
        print(f"[goto] Batería baja ({bat}%), abortando.")
        return False

    # Si el dron no está volando, despegar
    if getattr(self, "state", "") != "flying":
        ok = self.takeOff(0.5, blocking=True)
        if not ok:
            print("[goto] No se pudo despegar.")
            return False
        time.sleep(0.4)

    # Velocidad
    speed = int(speed_cm_s) if speed_cm_s else _DEFAULT_SPEED
    speed = max(10, min(100, speed))

    # CRÍTICO: Leer el yaw REAL del dron, no confiar en _commanded_yaw
    # El dron puede haber derivado durante el vuelo
    real_yaw = _get_real_yaw(self)
    if real_yaw is not None:
        # Convertir a rango [-180, 180] para consistencia
        if real_yaw > 180:
            real_yaw = real_yaw - 360
        current_yaw_before = real_yaw
        print(f"[goto] Yaw REAL del dron: {current_yaw_before:.1f}°")
    else:
        # Fallback a _commanded_yaw si no podemos leer
        current_yaw_before = getattr(self, "_commanded_yaw", None)
        if current_yaw_before is None:
            current_yaw_before = getattr(self.pose, "yaw_deg", 0.0) or 0.0
        print(f"[goto] Yaw (fallback _commanded): {current_yaw_before:.1f}°")

    # Guardar el yaw comandado para usar en cálculos
    commanded_yaw = current_yaw_before

    if yaw_deg is not None:
        # Calcular rotación necesaria desde el yaw REAL
        delta = (float(yaw_deg) - current_yaw_before) % 360.0
        if delta > 180:
            delta = delta - 360
        if abs(delta) >= 1.0:
            if delta > 0:
                self.cw(int(round(delta)))
            else:
                self.ccw(int(round(-delta)))
            time.sleep(0.3)
            # CRÍTICO: Leer yaw REAL después de rotar
            real_yaw_after = _get_real_yaw(self)
            if real_yaw_after is not None:
                if real_yaw_after > 180:
                    real_yaw_after = real_yaw_after - 360
                commanded_yaw = real_yaw_after
                print(f"[goto] Yaw REAL tras rotar: {commanded_yaw:.1f}°")
            else:
                commanded_yaw = float(yaw_deg)
        else:
            commanded_yaw = current_yaw_before
        self._commanded_yaw = commanded_yaw
    elif face_target and (abs(dx_cm) > _TOL_XY_CM or abs(dy_cm) > _TOL_XY_CM):
        target_angle = math.degrees(math.atan2(dy_cm, dx_cm))
        print(f"[goto] face_target: dx={dx_cm:.1f}, dy={dy_cm:.1f} → ángulo destino: {target_angle:.1f}°")
        # Calcular rotación necesaria desde el yaw REAL
        delta = (target_angle - current_yaw_before) % 360.0
        if delta > 180:
            delta = delta - 360
        if abs(delta) >= 1.0:
            if delta > 0:
                self.cw(int(round(delta)))
            else:
                self.ccw(int(round(-delta)))
            time.sleep(0.3)
            # CRÍTICO: Leer yaw REAL después de rotar
            real_yaw_after = _get_real_yaw(self)
            if real_yaw_after is not None:
                if real_yaw_after > 180:
                    real_yaw_after = real_yaw_after - 360
                commanded_yaw = real_yaw_after
                print(f"[goto] Yaw REAL tras rotar: {commanded_yaw:.1f}°")
            else:
                commanded_yaw = target_angle
        else:
            commanded_yaw = current_yaw_before
        self._commanded_yaw = commanded_yaw
        print(f"[goto] Yaw para movimiento: {commanded_yaw:.1f}°")

    # Calcular desplazamiento total restante
    dx_remaining = float(dx_cm)
    dy_remaining = float(dy_cm)
    dz_remaining = float(dz_cm)

    # Guardar altura comandada para evitar oscilaciones
    z_commanded = self.pose.z_cm

    # Dividir en segmentos si es necesario (máximo 500cm por comando)
    total_dist = math.sqrt(dx_remaining**2 + dy_remaining**2 + dz_remaining**2)

    if total_dist < _MIN_GO_CM:
        print("[goto] Distancia muy pequeña, objetivo alcanzado.")
        if callback:
            try: callback(params)
            except TypeError:
                try: callback()
                except: pass
        return True

    num_segments = max(1, int(math.ceil(total_dist / _MAX_GO_CM)))

    print(f"[goto] Distancia total: {total_dist:.0f}cm en {num_segments} segmento(s)")

    success_any = False
    for seg in range(num_segments):
        # Verificar aborto
        if getattr(self, "_goto_abort", False):
            print("[goto] Abortado por solicitud externa.")
            return False

        # Verificar batería
        bat = getattr(self, "battery_pct", None)
        if isinstance(bat, int) and bat < _MIN_BAT_PCT:
            print(f"[goto] Abortado por batería ({bat}%).")
            return False

        # Calcular el segmento actual
        if seg < num_segments - 1:
            # Segmentos intermedios: fracción igual
            frac = 1.0 / (num_segments - seg)
            seg_dx = dx_remaining * frac
            seg_dy = dy_remaining * frac
            seg_dz = dz_remaining * frac
        else:
            # Último segmento: todo lo que queda
            seg_dx = dx_remaining
            seg_dy = dy_remaining
            seg_dz = dz_remaining

        # Convertir a coordenadas locales del dron
        # IMPORTANTE: Usar commanded_yaw, NO pose.yaw_deg (telemetría puede sobrescribirlo)
        x_local, y_local = _world_to_local(seg_dx, seg_dy, commanded_yaw)

        # Redondear a enteros
        x_local_i = int(round(x_local))
        y_local_i = int(round(y_local))
        z_i = int(round(seg_dz))

        # Si algún componente es muy pequeño, ajustar
        if abs(x_local_i) < _MIN_GO_CM and abs(y_local_i) < _MIN_GO_CM and abs(z_i) < _MIN_GO_CM:
            # Distancia total del segmento muy pequeña
            dist_seg = math.sqrt(x_local_i**2 + y_local_i**2 + z_i**2)
            if dist_seg < _MIN_GO_CM:
                continue  # Saltar este segmento

        print(f"[goto] Segmento {seg+1}/{num_segments}: go {x_local_i} {y_local_i} {z_i} {speed}")

        # Guardar posición inicial para interpolación
        start_x = self.pose.x_cm
        start_y = self.pose.y_cm
        start_z = z_commanded

        # Enviar comando
        success, wait_time = _send_go_command(self, x_local_i, y_local_i, z_i, speed)

        if success:
            success_any = True
            if wait_time > 0:
                # Interpolar posición durante la espera (actualización visual)
                update_interval = 0.1  # 100ms entre actualizaciones
                steps = int(wait_time / update_interval)
                for step in range(steps):
                    if getattr(self, "_goto_abort", False):
                        break
                    progress = (step + 1) / steps
                    try:
                        self.pose.x_cm = start_x + seg_dx * progress
                        self.pose.y_cm = start_y + seg_dy * progress
                        self.pose.z_cm = start_z + seg_dz * progress
                    except Exception:
                        pass
                    time.sleep(update_interval)

            # Asegurar posición final, se suma lo que le ordenamos en el comando go
            try:
                self.pose.x_cm = start_x + seg_dx
                self.pose.y_cm = start_y + seg_dy
                z_commanded = start_z + seg_dz
                self.pose.z_cm = z_commanded
            except Exception:
                pass

            # Actualizar restante
            dx_remaining -= seg_dx
            dy_remaining -= seg_dy
            dz_remaining -= seg_dz

            print(f"[goto] Movimiento OK. Pose: ({self.pose.x_cm:.1f}, {self.pose.y_cm:.1f}, {z_commanded:.1f})")
        else:
            print(f"[goto] Movimiento falló en segmento {seg+1}")
            # Continuar intentando

        time.sleep(0.1)

    print("[goto] Objetivo alcanzado.")
    if callback:
        try: callback(params)
        except TypeError:
            try: callback()
            except: pass
    if not success_any:
        print("[goto] WARNING: Ningún segmento confirmó movimiento; abortando.")
        return False
    return True


def goto_rel(self,
             dx_cm: float, dy_cm: float, dz_cm: float = 0.0,
             yaw_deg: Optional[float] = None,
             speed_cm_s: Optional[float] = None,
             face_target: bool = False,
             blocking: bool = True,
             callback: Optional[Callable[..., Any]] = None,
             params: Any = None) -> None:

    setattr(self, "_goto_abort", False)

    result_holder = {"ok": False}
    t = threading.Thread(
        target=_goto_rel_worker,
        args=(self, dx_cm, dy_cm, dz_cm, yaw_deg, speed_cm_s, face_target, callback, params, result_holder),
        daemon=True
    )
    t.start()
    if blocking:
        t.join()
        return result_holder["ok"]
    return None


def abort_goto(self) -> None:

    setattr(self, "_goto_abort", True)