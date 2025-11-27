from __future__ import annotations
import math
import threading
import time
from typing import List, Tuple, Optional, Dict, Any

_DEFAULT_MAX_X_CM = 150.0
_DEFAULT_MAX_Y_CM = 150.0
_DEFAULT_MAX_Z_CM = 120.0
_MODE_SOFT_ABORT = "soft"
_MODE_HARD_LAND = "hard"
_DEFAULT_POLL_S = 0.10
_HARD_LAND_DELAY = 0.2

# Funciones geométricas

# Función para saber si un punto está dentro del polígono o fuera, a partir del algoritmo "ray casting"

def _point_in_poly(x: float, y: float, poly: List[Tuple[float, float]], eps=1e-6) -> bool:
    n = len(poly)
    if n < 3:
        return False  # Un polígono necesita al menos 3 vértices para ser considerado como un polígono

    #  Verificamos si el punto está exactamente sobre algún borde (caso especial)
    for i in range(n):  # Bucle que recorre todos los vértices
        x1, y1 = poly[i]  # Extraemos las coordenadas
        x2, y2 = poly[(i + 1) % n]  # % n (operador módulo) para cerrar el polígono (del último al primero)
        if _point_on_segment(x, y, x1, y1, x2, y2, eps):
            return True  # Está sobre el borde.

    # Algoritmo de ray casting
    inside = False
    j = n - 1  # Empezamos con el último vértice
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]

        # Verificamos si el rayo horizontal cruza el segmento
        if ((yi > y) != (yj > y)):  # El segmento cruza la altura y del punto
            # Calculamos la coordenada X donde el rayo cruza el segmento
            denom = (yj - yi) if (
                                             yj - yi) != 0 else 1e-9  # Si el segmento fuese horizontal, la diferencia sería 0, pero daría error, por eso ponemos un valor muy pequeño
            x_inter = xi + (xj - xi) * (
                        y - yi) / denom  # Con la interpolación lineal, encontramos la coordenada x donde el segmento cruza la línea horizontal

            if x < x_inter:  # Si  cruce está a la derecha del punto
                inside = not inside  # Alternamos el estado
        j = i

    return inside


def _point_on_segment(px, py, x1, y1, x2, y2, eps=1e-6):
    # Producto vectorial para ver si está alineado: si es ~0, el punto está en la línea que contiene el segmento
    cross = abs((px - x1) * (y2 - y1) - (py - y1) * (x2 - x1))
    if cross > eps:
        return False  # No está alineado con el segmento (arriba o abajo del segmento)

    # Producto escalar: si es ≤0, el punto está entre los dos extremos

    dot = (px - x1) * (px - x2) + (py - y1) * (py - y2)
    if dot > eps:
        return False  # Está fuera del segmento (más allá de algún extremo)

    return True


def _point_in_circle(x, y, cx, cy, r_cm):
    dx, dy = x - cx, y - cy
    # Comparamos distancia² con radio² (evitamos sqrt innecesario)
    # El 1e-6 es tolerancia para puntos exactamente en el borde
    return (dx * dx + dy * dy) <= (r_cm * r_cm + 1e-6)


# Función para construir el geofence

def set_geofence(self,
                 max_x_cm=_DEFAULT_MAX_X_CM,
                 max_y_cm=_DEFAULT_MAX_Y_CM,
                 max_z_cm=_DEFAULT_MAX_Z_CM,
                 z_min_cm=0.0,
                 mode=_MODE_SOFT_ABORT,
                 poll_interval_s=_DEFAULT_POLL_S):
    # Construimos diccionario de límites
    lim: Dict[str, float] = {}
    if max_x_cm and max_x_cm > 0:
        lim["max_x"] = float(max_x_cm)
    if max_y_cm and max_y_cm > 0:
        lim["max_y"] = float(max_y_cm)
    if max_z_cm and max_z_cm > 0:
        lim["max_z"] = float(max_z_cm)

    # z_min siempre se guarda (por defecto 0)
    lim["zmin"] = float(z_min_cm) if z_min_cm is not None else 0.0

    # Guardamos la configuración en atributos del objeto
    self._gf_limits = lim if lim else None
    self._gf_mode = mode if mode in (_MODE_SOFT_ABORT, _MODE_HARD_LAND) else _MODE_SOFT_ABORT
    self._gf_poll_s = max(0.05, float(poll_interval_s))  # Mínimo 50ms

    # Centro del geofence: si no está definido, usamos (0,0)
    if not hasattr(self, "_gf_center"):
        self._gf_center = (0.0, 0.0)

    self._gf_enabled = True  # Activamos el geofence

    # Inicializamos listas de exclusiones si no existen
    if not hasattr(self, "_gf_excl_polys"):
        self._gf_excl_polys = []
    if not hasattr(self, "_gf_excl_circles"):
        self._gf_excl_circles = []

    # Contador de violaciones consecutivas
    self._gf_violation_streak = 0

    # Reiniciamos el hilo monitor (lo paramos si existía, y lo iniciamos nuevo)
    _stop_geofence_monitor(self)
    _start_geofence_monitor(self)

    span_txt = f"ancho={lim.get('max_x', 0):.0f}x{lim.get('max_y', 0):.0f} cm" if lim else "SIN inclusión (solo exclusiones)"
    maxz_txt = f"z_max={lim.get('max_z', 0):.0f} cm" if lim else "z_max=∞"
    zmin_txt = f"z_min={lim.get('zmin', 0):.0f} cm"
    print(f"[geofence] Activado: {span_txt}, {zmin_txt}, {maxz_txt}, modo={self._gf_mode}")


# Función para desactivar el geofence
def disable_geofence(self):
    self._gf_enabled = False
    _stop_geofence_monitor(self)
    print("[geofence] Desactivado.")


# Función para reecentrar el geofence a la posición actual del dron en ese momento
def recenter_geofence(self):
    pose = getattr(self, "pose", None)
    if pose:
        # Extraemos coordenadas actuales (con fallback a 0 si no existen)
        self._gf_center = (float(getattr(pose, "x_cm", 0.0) or 0.0),
                           float(getattr(pose, "y_cm", 0.0) or 0.0))
        print(f"[geofence] Recentrado en {self._gf_center}")
    else:
        print("[geofence] No se pudo recentrar (pose desconocida).")


# Función para añadir un círculo de exclusión
def add_exclusion_circle(self, cx, cy, r_cm, z_min_cm=None, z_max_cm=None):
    # Inicializamos lista si no existe
    if not hasattr(self, "_gf_excl_circles"):
        self._gf_excl_circles = []

    # Normalizamos valores
    cx = float(cx)
    cy = float(cy)
    r = abs(float(r_cm))  # abs() por si pasan radio negativo

    # Creamos el diccionario que describe el círculo
    item = {
        "cx": cx,
        "cy": cy,
        "r": r,
        "zmin": float(z_min_cm) if z_min_cm is not None else None,
        "zmax": float(z_max_cm) if z_max_cm is not None else None
    }

    self._gf_excl_circles.append(item)

    # Nos aseguramos de que el monitor esté corriendo
    _ensure_gf_monitor(self)

    z_range = f"z∈[{item['zmin']},{item['zmax']}]" if item['zmin'] is not None and item[
        'zmax'] is not None else "z=todas"
    print(f"[geofence] Círculo añadido: centro=({cx:.1f},{cy:.1f}), r={r:.1f}cm, {z_range}")

    return item


# Función para añadir un polígono de exclusión
def add_exclusion_poly(self, points, z_min_cm=None, z_max_cm=None):
    if not hasattr(self, "_gf_excl_polys"):
        self._gf_excl_polys = []

    # Convertimos todos los puntos a float
    poly = [(float(x), float(y)) for (x, y) in points]

    item = {
        "poly": poly,
        "zmin": float(z_min_cm) if z_min_cm is not None else None,
        "zmax": float(z_max_cm) if z_max_cm is not None else None
    }

    self._gf_excl_polys.append(item)
    _ensure_gf_monitor(self)

    z_range = f"z∈[{item['zmin']},{item['zmax']}]" if item['zmin'] is not None and item[
        'zmax'] is not None else "z=todas"
    print(f"[geofence] Polígono añadido: {len(poly)} vértices, {z_range}")

    return item


# Función para limpiar las diferentes exclusiones
def clear_exclusions(self):
    self._gf_excl_polys = []
    self._gf_excl_circles = []
    print("[geofence] Exclusiones eliminadas.")


# Función para iniciar el hilo que monitorea constantemente el dron
def _start_geofence_monitor(self, force=False):
    # Si ya está monitoreando y no forzamos reinicio, no hacemos nada
    if getattr(self, "_gf_monitoring", False) and not force:
        return

    self._gf_monitoring = True

    # Creamos hilo daemon (se cierra automáticamente cuando termina el programa)
    t = threading.Thread(target=_gf_monitor_loop, args=(self,), daemon=True)
    self._gf_thread = t
    t.start()

    print("[geofence] Monitor iniciado.")


# Función para detener el hilo de monitoreo
def _stop_geofence_monitor(self):
    self._gf_monitoring = False  # Señal para que el loop termine

    t = getattr(self, "_gf_thread", None)
    if t and isinstance(t, threading.Thread):
        try:
            t.join(timeout=1.0)  # Esperamos máximo 1 segundo
        except Exception:
            pass

    self._gf_thread = None
    print("[geofence] Monitor detenido.")


def _ensure_gf_monitor(self):
    if getattr(self, "_gf_enabled", False) and not getattr(self, "_gf_monitoring", False):
        _start_geofence_monitor(self)


# Función principal del monitor del geofence, que verifica si se encuentra dentro o fuera de las zonas de exclusión e inclusión
def _gf_monitor_loop(self):
    self._gf_violation_streak = 0  # Contador de violaciones consecutivas

    while getattr(self, "_gf_monitoring", False) and getattr(self, "_gf_enabled", False):
        try:
            # Solo verificamos si el dron está en un estado de vuelo activo
            st = getattr(self, "state", "")
            if st not in ("flying", "landing", "hovering", "takingoff"):
                time.sleep(self._gf_poll_s)
                continue  # Si está en tierra, no hay nada que verificar

            # Obtenemos la posición actual del dron
            pose = getattr(self, "pose", None)
            if pose is None:
                time.sleep(self._gf_poll_s)
                continue  # Sin pose, no podemos verificar

            # Extraemos coordenadas
            x = float(getattr(pose, "x_cm", 0.0) or 0.0)
            y = float(getattr(pose, "y_cm", 0.0) or 0.0)
            z = float(getattr(pose, "z_cm", getattr(self, "height_cm", 0.0)) or 0.0)

            violated = (not _inside_inclusion(self, x, y, z)) or _inside_any_exclusion(self, x, y, z)

            if violated:
                self._gf_violation_streak += 1
            else:
                self._gf_violation_streak = 0

            # Actuamos solo si hay 2 lecturas consecutivas de violación

            if self._gf_violation_streak >= 2:
                _handle_violation(self)

                # En modo HARD, salimos del loop después de ordenar aterrizaje
                if getattr(self, "_gf_mode", _MODE_SOFT_ABORT) == _MODE_HARD_LAND:
                    break

        except Exception as e:
            print(f"[geofence] Error monitor: {e}")

        time.sleep(self._gf_poll_s)  # Esperamos antes de la siguiente verificación

    # Salimos del bucle - marcamos que ya no estamos monitoreando
    self._gf_monitoring = False


# Función para verificar si se encuentra dentro de la zona de inclusión
def _inside_inclusion(self, x, y, z):
    lim = getattr(self, "_gf_limits", None)
    if not lim:
        return True

    # Obtenemos el centro del geofence
    cx, cy = getattr(self, "_gf_center", (0.0, 0.0))

    # Obtenemos los límites
    max_x = float(lim.get("max_x", 0.0) or 0.0)
    max_y = float(lim.get("max_y", 0.0) or 0.0)
    max_z = float(lim.get("max_z", 0.0) or 0.0)
    zmin = float(lim.get("zmin", 0.0) or 0.0)

    # max_x/max_y son el ANCHO TOTAL, así que dividimos por 2
    # para obtener la distancia máxima desde el centro
    half_x = max_x / 2.0
    half_y = max_y / 2.0

    # Verificamos cada eje (si el límite es 0, no hay restricción en ese eje)
    in_x = True if max_x <= 0 else (abs(x - cx) <= half_x)
    in_y = True if max_y <= 0 else (abs(y - cy) <= half_y)
    in_z = True if max_z <= 0 else (zmin <= z <= max_z)

    return in_x and in_y and in_z


# Función para verificar si se encuentra dentro de alguna zona de exclusión
def _inside_any_exclusion(self, x, y, z):
    # Verificamos polígonos de exclusión
    for entry in list(getattr(self, "_gf_excl_polys", [])):
        if not isinstance(entry, dict):
            continue  # Ignoramos datos corruptos

        poly = entry.get("poly", [])
        zmin = entry.get("zmin")
        zmax = entry.get("zmax")

        if _point_in_poly(x, y, poly):
            # El punto está dentro del polígono en X,Y
            # Ahora verificamos si también está en el rango de altura
            z_ok = (zmin is None or z >= zmin) and (zmax is None or z <= zmax)
            if z_ok:
                print(f"[geofence]  VIOLACIÓN POLY @ ({x:.1f},{y:.1f},{z:.1f})")
                return True

    # Verificamos círculos de exclusión
    for entry in list(getattr(self, "_gf_excl_circles", [])):
        if not isinstance(entry, dict):
            continue

        cx_c = entry.get("cx")
        cy_c = entry.get("cy")
        r = entry.get("r")
        zmin = entry.get("zmin")
        zmax = entry.get("zmax")

        if _point_in_circle(x, y, cx_c, cy_c, r):
            # Está dentro del círculo en X,Y, verificamos altura
            z_ok = (zmin is None or z >= zmin) and (zmax is None or z <= zmax)
            if z_ok:
                print(f"[geofence]  VIOLACIÓN CIRCLE @ ({x:.1f},{y:.1f},{z:.1f})")
                return True

    return False  # No está en ninguna exclusión


# Función que maneja y decide el tratamiento a las violaciones dependiendo del modo de geofence
def _handle_violation(self):
    mode = getattr(self, "_gf_mode", _MODE_SOFT_ABORT)

    # Evitamos spam de mensajes repetidos
    if getattr(self, "_gf_last_report", None) != mode:
        print(f"[geofence]  Violación detectada (modo={mode}).")
        self._gf_last_report = mode

    # Siempre abortamos comandos de movimiento en curso
    setattr(self, "_goto_abort", True)  # Aborta goto_xy, goto_xyz, etc.
    setattr(self, "_mission_abort", True)  # Aborta misiones/waypoints

    if mode == _MODE_HARD_LAND:
        # Evitamos iniciar múltiples aterrizajes
        if getattr(self, "_gf_landing_initiated", False):
            return
        self._gf_landing_initiated = True

        # Verificamos que realmente estemos volando
        st = getattr(self, "state", "")
        if st not in ("flying", "hovering", "takingoff"):
            return  # Ya está en tierra o aterrizando

        print("[geofence]  Aterrizando de emergencia…")

        # Detenemos el monitor para evitar bucles infinitos
        self._gf_monitoring = False

        # Ejecutamos el aterrizaje en un hilo separado
        def do_land():
            try:
                time.sleep(_HARD_LAND_DELAY)  # Pequeña pausa de seguridad
                self.Land(blocking=True)  # Aterrizaje bloqueante
            except Exception as e:
                print(f"[geofence] Error Land: {e}")
            finally:
                self._gf_landing_initiated = False  # Permitimos futuros aterrizajes

        threading.Thread(target=do_land, daemon=True).start()