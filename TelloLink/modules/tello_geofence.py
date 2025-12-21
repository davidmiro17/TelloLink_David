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

# Sistema de capas de altitud (3 capas por defecto)
_DEFAULT_LAYERS = [
    {"name": "Capa 1", "z_min": 0, "z_max": 60},      # Suelo/mesa (0-60 cm)
    {"name": "Capa 2", "z_min": 60, "z_max": 120},    # Altura media (60-120 cm)
    {"name": "Capa 3", "z_min": 120, "z_max": 200},   # Techo/lámpara (120-200 cm)
]

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
            poll_s = getattr(self, "_gf_poll_s", 0.1)  # Default 100ms

            st = getattr(self, "state", "")
            if st not in ("flying", "landing", "hovering", "takingoff"):
                time.sleep(poll_s)
                continue  # Si está en tierra, no hay nada que verificar

            # Obtenemos la posición actual del dron
            pose = getattr(self, "pose", None)
            if pose is None:
                time.sleep(poll_s)
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

        time.sleep(poll_s)  # Esperamos antes de la siguiente verificación

    # Salimos del bucle - marcamos que ya no estamos monitoreando
    self._gf_monitoring = False


# Función para verificar si se encuentra dentro de la zona de inclusión
def _inside_inclusion(self, x, y, z):
    lim = getattr(self, "_gf_limits", None)
    if not lim:
        return True

    # Soporte para formato nuevo (x1,y1,x2,y2) y antiguo (max_x,max_y,center)
    if "x1" in lim and "x2" in lim:
        # Formato nuevo: coordenadas absolutas
        x1 = float(lim.get("x1", 0.0))
        y1 = float(lim.get("y1", 0.0))
        x2 = float(lim.get("x2", 0.0))
        y2 = float(lim.get("y2", 0.0))
        zmin = float(lim.get("zmin", 0.0) or 0.0)
        zmax = float(lim.get("zmax", 200.0) or 200.0)

        # Normalizar (asegurar x1 < x2, y1 < y2)
        if x1 > x2:
            x1, x2 = x2, x1
        if y1 > y2:
            y1, y2 = y2, y1

        in_x = x1 <= x <= x2
        in_y = y1 <= y <= y2
        in_z = zmin <= z <= zmax

        return in_x and in_y and in_z
    else:
        # Formato antiguo: centro + dimensiones
        cx, cy = getattr(self, "_gf_center", (0.0, 0.0))

        max_x = float(lim.get("max_x", 0.0) or 0.0)
        max_y = float(lim.get("max_y", 0.0) or 0.0)
        max_z = float(lim.get("max_z", 0.0) or 0.0)
        zmin = float(lim.get("zmin", 0.0) or 0.0)

        half_x = max_x / 2.0
        half_y = max_y / 2.0

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










































































































































#Nuevas funciones para  geofence usando el modo rc (joystick)



#Función para convertir las velocidades del joystick relativas al dron a las velocidades en coordenadas del mundo
#Ya que el dron necesitamos saber si forward ("adelante") es hacia X o Y dependiendo del yaw
def joystick_a_mundo(vx_joy, vy_joy, yaw_deg):

    yaw_rad = math.radians(yaw_deg)

    vel_X_mundo = vy_joy * math.cos(yaw_rad) - vx_joy * math.sin(yaw_rad)
    vel_Y_mundo = vy_joy * math.sin(yaw_rad) + vx_joy * math.cos(yaw_rad)

    return vel_X_mundo, vel_Y_mundo



#Función para calcular la distancia al "peligro" (límite) más cercano en la dirección del movimiento
#Como busco que el dron se pueda acercar al "peligro" y que vaya siendo avisado de que está a punto de llegar a un límite, por eso se usa esta función
def calcular_distancia_peligro(x_dron, y_dron, z_dron,
                               vel_X_mundo, vel_Y_mundo, vel_Z,
                               limites, centro,
                               circulos_excl, poligonos_excl):


    # Lista donde guardaremos todas las distancias a "peligros"

    distancias = []


    #Inclusión

    # Aquí calculamos la distancia a los bordes del rectángulo
    # Solo nos importa el borde hacia donde vamos

    if limites:
        # Soporte para formato nuevo (x1,y1,x2,y2) y antiguo (max_x,max_y,center)
        if "x1" in limites and "x2" in limites:
            # Formato nuevo: coordenadas absolutas
            x1 = float(limites.get("x1", 0.0))
            y1 = float(limites.get("y1", 0.0))
            x2 = float(limites.get("x2", 0.0))
            y2 = float(limites.get("y2", 0.0))
            z_min = float(limites.get("zmin", 0.0) or 0.0)
            z_max = float(limites.get("zmax", 200.0) or 200.0)

            # Normalizar
            if x1 > x2:
                x1, x2 = x2, x1
            if y1 > y2:
                y1, y2 = y2, y1

            # Si vamos hacia +X
            if vel_X_mundo > 0:
                dist = x2 - x_dron
                if dist > 0:
                    distancias.append(dist)
            # Si vamos hacia -X
            elif vel_X_mundo < 0:
                dist = x_dron - x1
                if dist > 0:
                    distancias.append(dist)

            # Si vamos hacia +Y
            if vel_Y_mundo > 0:
                dist = y2 - y_dron
                if dist > 0:
                    distancias.append(dist)
            # Si vamos hacia -Y
            elif vel_Y_mundo < 0:
                dist = y_dron - y1
                if dist > 0:
                    distancias.append(dist)

            # Si subimos
            if vel_Z > 0:
                dist = z_max - z_dron
                if dist > 0:
                    distancias.append(dist)
            # Si bajamos
            elif vel_Z < 0:
                dist = z_dron - z_min
                if dist > 0:
                    distancias.append(dist)
        else:
            # Formato antiguo: centro + dimensiones
            cx, cy = centro

            half_x = limites.get("max_x", 0) / 2
            half_y = limites.get("max_y", 0) / 2

            z_min = limites.get("zmin", 0)
            z_max = limites.get("max_z", 200)

            # Si vamos hacia +X (adelante en el mundo)
            if vel_X_mundo > 0 and half_x > 0:
                limite_adelante = cx + half_x
                dist = limite_adelante - x_dron
                distancias.append(dist)
            # Si vamos hacia -X (atras en el mundo)
            elif vel_X_mundo < 0 and half_x > 0:
                limite_atras = cx - half_x
                dist = x_dron - limite_atras
                distancias.append(dist)

            if vel_Y_mundo > 0 and half_y > 0:
                limite_derecha = cy + half_y
                dist = limite_derecha - y_dron
                distancias.append(dist)
            elif vel_Y_mundo < 0 and half_y > 0:
                limite_izquierda = cy - half_y
                dist = y_dron - limite_izquierda
                distancias.append(dist)

            # Si subimos
            if vel_Z > 0 and z_max > 0:
                dist = z_max - z_dron
                distancias.append(dist)
            # Si bajamos
            elif vel_Z < 0:
                dist = z_dron - z_min
                distancias.append(dist)


    #Exclusión (círculos)

    # Para cada círculo, calculamos si vamos hacia él
    # y a qué distancia está su borde

    for c in circulos_excl:
        c_zmin = c.get("zmin")
        c_zmax = c.get("zmax")
        c_zmin = float(c_zmin) if c_zmin is not None else 0.0
        c_zmax = float(c_zmax) if c_zmax is not None else 999.0

        c_x = c["cx"]  # Centro X del círculo
        c_y = c["cy"]  # Centro Y del círculo
        radio = c["r"]  # Radio del círculo

        # Verificar si el dron está horizontalmente dentro del círculo
        dx = c_x - x_dron
        dy = c_y - y_dron
        dist_al_centro = math.sqrt(dx * dx + dy * dy)
        dentro_xy = dist_al_centro <= radio

        # CASO 1: Dron está en el rango Z del obstáculo - bloqueo horizontal normal
        if c_zmin <= z_dron <= c_zmax:
            # Vector que va desde el dron hacia el centro del círculo
            producto = vel_X_mundo * dx + vel_Y_mundo * dy

            if producto > 0:
                dist_al_borde = dist_al_centro - radio
                if dist_al_borde > 0:
                    distancias.append(dist_al_borde)

        # CASO 2: Dron está ENCIMA del obstáculo y bajando - bloqueo vertical
        elif z_dron > c_zmax and vel_Z < 0 and dentro_xy:
            # Si estamos encima del cilindro y bajando, calcular distancia al techo
            dist_vertical = z_dron - c_zmax
            if dist_vertical > 0:
                distancias.append(dist_vertical)


    # Exclusión (polígonos)

    for p in poligonos_excl:
        p_zmin = p.get("zmin")
        p_zmax = p.get("zmax")
        p_zmin = float(p_zmin) if p_zmin is not None else 0.0
        p_zmax = float(p_zmax) if p_zmax is not None else 999.0

        # Extraemos la lista de puntos del polígono
        poly = p["poly"]
        if len(poly) < 3:
            continue

        # Verificar si el dron está horizontalmente dentro del polígono
        dentro_xy = _point_in_poly(x_dron, y_dron, poly)

        # Calcular distancia al borde más cercano
        dist_min_poly = float('inf')
        for i in range(len(poly)):
            x1, y1 = poly[i]
            x2, y2 = poly[(i + 1) % len(poly)]
            dist_segmento = _distancia_punto_a_segmento(x_dron, y_dron, x1, y1, x2, y2)
            if dist_segmento < dist_min_poly:
                dist_min_poly = dist_segmento

        # Centroide para calcular dirección
        centroide_x = sum(pt[0] for pt in poly) / len(poly)
        centroide_y = sum(pt[1] for pt in poly) / len(poly)
        dx = centroide_x - x_dron
        dy = centroide_y - y_dron

        # CASO 1: Dron está en el rango Z del obstáculo - bloqueo horizontal normal
        if p_zmin <= z_dron <= p_zmax:
            producto = vel_X_mundo * dx + vel_Y_mundo * dy
            if producto > 0 and dist_min_poly < float('inf'):
                distancias.append(dist_min_poly)

        # CASO 2: Dron está ENCIMA del obstáculo y bajando - bloqueo vertical
        elif z_dron > p_zmax and vel_Z < 0 and dentro_xy:
            dist_vertical = z_dron - p_zmax
            if dist_vertical > 0:
                distancias.append(dist_vertical)


    # Devolvemos la distancia mínima, es decir el peligro más cercano

    if distancias:

        return max(0, min(distancias))
    else:

        return float('inf')

#Función para calcular la distancia a un segmento
def _distancia_punto_a_segmento(px, py, x1, y1, x2, y2):


    # Vector del segmento (de punto 1 a punto 2)
    dx = x2 - x1
    dy = y2 - y1

    # Longitud al cuadrado del segmento

    len_sq = dx * dx + dy * dy

    if len_sq == 0:
        # Caso especial: el segmento es un punto (los dos extremos son iguales)
        # La distancia es simplemente la distancia al punto
        return math.sqrt((px - x1) ** 2 + (py - y1) ** 2)

    # "t" es la proyección del punto sobre la línea del segmento
    # t=0 significa que el punto más cercano es el extremo (x1,y1)
    # t=1 significa que el punto más cercano es el extremo (x2,y2)
    # t entre 0 y 1 significa que el punto más cercano está en medio del segmento
    t = ((px - x1) * dx + (py - y1) * dy) / len_sq

    # Limitamos t entre 0 y 1
    t = max(0, min(1, t))

    # Calculamos el punto más cercano en el segmento
    cerca_x = x1 + t * dx
    cerca_y = y1 + t * dy

    # Distancia del punto original al punto más cercano (Pitágoras)
    return math.sqrt((px - cerca_x) ** 2 + (py - cerca_y) ** 2)

#Función para calcular el factor de atenuacion en la velocidad a medida que nos acercamos al peligro
#La idea es que a medida que nos acercamos, cada vez nos cuesta mas, finalmente el dron no avanzará
def calcular_factor_atenuacion(distancia, margen=30.0):


    # Si estamos lejos del peligro (fuera del margen)
    if distancia >= margen:
        return 1.0  # Velocidad completa normal

    # Si estamos en el límite o ya lo pasamos
    if distancia <= 0:
        return 0.0  # Parar completamente el dron

    # Si estamos en la zona de transición
    # Factor proporcional: más cerca = más frena el dron
    factor = distancia / margen

    return factor


#Función para aplicar la atenuación
def aplicar_atenuacion_completa(vel_X_mundo, vel_Y_mundo, vel_Z, factor, x_dron, y_dron, peligro_info):


    vel_X_at = vel_X_mundo
    vel_Y_at = vel_Y_mundo
    vel_Z_at = vel_Z

    tipo = peligro_info.get("tipo")

    #Rectangulo
    if tipo == "rectangulo":
        direccion = peligro_info.get("direccion")

        if direccion == "+X" and vel_X_mundo > 0:
            vel_X_at = vel_X_mundo * factor
        elif direccion == "-X" and vel_X_mundo < 0:
            vel_X_at = vel_X_mundo * factor
        elif direccion == "+Y" and vel_Y_mundo > 0:
            vel_Y_at = vel_Y_mundo * factor
        elif direccion == "-Y" and vel_Y_mundo < 0:
            vel_Y_at = vel_Y_mundo * factor
        elif direccion == "+Z" and vel_Z > 0:
            vel_Z_at = vel_Z * factor
        elif direccion == "-Z" and vel_Z < 0:
            vel_Z_at = vel_Z * factor

    #Círculo o polígono
    elif tipo in ("circulo", "poligono"):

        # Atenuamos la componente de velocidad que va hacia él "peligro"

        cx, cy = peligro_info.get("centro", (0, 0))

        # Vector del dron hacia el peligro
        dx = cx - x_dron
        dy = cy - y_dron

        # Normalizamos (longitud = 1)
        longitud = math.sqrt(dx * dx + dy * dy)
        if longitud > 0:
            dx_norm = dx / longitud
            dy_norm = dy / longitud

            # Componente de velocidad hacia el peligro
            vel_hacia_peligro = vel_X_mundo * dx_norm + vel_Y_mundo * dy_norm

            # Solo atenuamos si vamos hacia el peligro
            if vel_hacia_peligro > 0:
                # Calculamos cuánto hay que restar
                reduccion = vel_hacia_peligro * (1 - factor)

                # Restamos en la dirección del peligro
                vel_X_at = vel_X_mundo - reduccion * dx_norm
                vel_Y_at = vel_Y_mundo - reduccion * dy_norm

    return vel_X_at, vel_Y_at, vel_Z_at



#Función que transforma velocidades del mundo a velocidades relativas al dron
def mundo_a_joystick(vel_X_mundo, vel_Y_mundo, yaw_deg):


    # Convertimos el ángulo de grados a radianes

    yaw_rad = math.radians(yaw_deg)

    # Calculamos coseno y seno del yaw
    cos_yaw = math.cos(yaw_rad)
    sin_yaw = math.sin(yaw_rad)



    vy_joy = vel_X_mundo * cos_yaw + vel_Y_mundo * sin_yaw
    vx_joy = -vel_X_mundo * sin_yaw + vel_Y_mundo * cos_yaw

    return vx_joy, vy_joy

#Función principal que aplica el geofence a los comandos rc del joystick
def aplicar_geofence_rc(self, vx_joy, vy_joy, vz, yaw_joy):


    # Si el geofence no está activado, devolvemos los valores sin cambios
    if not getattr(self, "_gf_enabled", False):
        return vx_joy, vy_joy, vz, yaw_joy

    # Obtenemos la posición y yaw del dron
    pose = getattr(self, "pose", None)
    if pose is None:
        return vx_joy, vy_joy, vz, yaw_joy

    x_dron = float(getattr(pose, "x_cm", 0.0) or 0.0)
    y_dron = float(getattr(pose, "y_cm", 0.0) or 0.0)
    z_dron = float(getattr(pose, "z_cm", 0.0) or 0.0)
    yaw_deg = float(getattr(pose, "yaw_deg", 0.0) or 0.0)

    # Obtenemos los límites y exclusiones
    limites = getattr(self, "_gf_limits", None)
    centro = getattr(self, "_gf_center", (0.0, 0.0))
    circulos = getattr(self, "_gf_excl_circles", [])
    poligonos = getattr(self, "_gf_excl_polys", [])
    margen = getattr(self, "_gf_margen", 30.0)

    #Transformarmos joystick a "mundo"
    vel_X_mundo, vel_Y_mundo = joystick_a_mundo(vx_joy, vy_joy, yaw_deg)

    # ===== NUEVO: Verificar si ya estamos FUERA del geofence =====
    # Si estamos fuera, solo permitir movimiento hacia DENTRO
    if limites and "x1" in limites:
        x1 = float(limites.get("x1", -999))
        y1 = float(limites.get("y1", -999))
        x2 = float(limites.get("x2", 999))
        y2 = float(limites.get("y2", 999))
        if x1 > x2:
            x1, x2 = x2, x1
        if y1 > y2:
            y1, y2 = y2, y1

        # Verificar si estamos fuera en X
        if x_dron < x1:
            # Fuera por la izquierda - solo permitir movimiento hacia +X
            if vel_X_mundo < 0:
                vel_X_mundo = 0
        elif x_dron > x2:
            # Fuera por la derecha - solo permitir movimiento hacia -X
            if vel_X_mundo > 0:
                vel_X_mundo = 0

        # Verificar si estamos fuera en Y
        if y_dron < y1:
            # Fuera por abajo - solo permitir movimiento hacia +Y
            if vel_Y_mundo < 0:
                vel_Y_mundo = 0
        elif y_dron > y2:
            # Fuera por arriba - solo permitir movimiento hacia -Y
            if vel_Y_mundo > 0:
                vel_Y_mundo = 0

        # Verificar si estamos fuera en Z (altura)
        z_min = float(limites.get("zmin", 0) or 0)
        z_max = float(limites.get("zmax", 200) or 200)

        if z_dron < z_min:
            # Fuera por abajo - solo permitir subir
            if vz < 0:
                vz = 0
        elif z_dron > z_max:
            # Fuera por arriba - solo permitir bajar
            if vz > 0:
                vz = 0

    #Calcular distancia al peligro más cercano
    distancia = calcular_distancia_peligro(x_dron, y_dron, z_dron, vel_X_mundo, vel_Y_mundo, vz, limites, centro, circulos, poligonos)

    #Calcular factor de atenuación
    factor = calcular_factor_atenuacion(distancia, margen)

    #Aplicar la atenuación
    vel_X_at = vel_X_mundo * factor
    vel_Y_at = vel_Y_mundo * factor
    vz_at = vz * factor

    #Transformamos  de vuelta a joystick
    vx_nuevo, vy_nuevo = mundo_a_joystick(vel_X_at, vel_Y_at, yaw_deg)


    return vx_nuevo, vy_nuevo, vz_at, yaw_joy


# ============================================================================
# SISTEMA DE CAPAS DE ALTITUD
# ============================================================================

def set_layers(self, layers: List[Dict] = None):

    if layers is None:
        self._gf_layers = [dict(layer) for layer in _DEFAULT_LAYERS]
    else:
        # Validamos que sean 3 capas
        if len(layers) != 3:
            print(f"[capas] Error: Se requieren exactamente 3 capas, recibidas {len(layers)}")
            return False

        # Validamos estructura
        self._gf_layers = []
        for i, layer in enumerate(layers):
            self._gf_layers.append({
                "name": layer.get("name", f"Capa {i+1}"),
                "z_min": float(layer.get("z_min", 0)),
                "z_max": float(layer.get("z_max", 200))
            })

    # Guardamos la capa anterior para detectar cambios
    if not hasattr(self, "_gf_last_layer"):
        self._gf_last_layer = None

    layers_info = [f"{l['name']}({l['z_min']}-{l['z_max']}cm)" for l in self._gf_layers]
    print(f"[capas] Configuradas: {layers_info}")
    return True


def get_layers(self) -> List[Dict]:

    if not hasattr(self, "_gf_layers") or not self._gf_layers:
        self._gf_layers = [dict(layer) for layer in _DEFAULT_LAYERS]
    return self._gf_layers


def get_current_layer(self, z_cm: float = None) -> int:

    # Si no se proporciona altura, la obtenemos del dron
    if z_cm is None:
        pose = getattr(self, "pose", None)
        if pose:
            z_cm = float(getattr(pose, "z_cm", 0) or 0)
        else:
            z_cm = float(getattr(self, "height_cm", 0) or 0)

    # Obtenemos las capas
    layers = get_layers(self)

    # Buscamos en qué capa está
    for i, layer in enumerate(layers):
        z_min = layer["z_min"]
        z_max = layer["z_max"]

        # Usamos <= para z_max para incluir el límite superior
        if z_min <= z_cm <= z_max:
            return i + 1  # Capas son 1-indexed

    # Si está por debajo de la capa 1, consideramos capa 1
    if z_cm < layers[0]["z_min"]:
        return 1

    # Si está por encima de la capa 3, consideramos capa 3
    if z_cm > layers[-1]["z_max"]:
        return 3

    return 0  # Fuera de rango (no debería pasar)


def get_exclusion_layers(self, exclusion: Dict) -> List[int]:

    excl_zmin = exclusion.get("zmin")
    excl_zmax = exclusion.get("zmax")

    # Si no tiene límites de altura, ocupa todas las capas
    if excl_zmin is None and excl_zmax is None:
        return [1, 2, 3]

    # Normalizamos valores
    excl_zmin = float(excl_zmin) if excl_zmin is not None else 0.0
    excl_zmax = float(excl_zmax) if excl_zmax is not None else float('inf')

    layers = get_layers(self)
    result = []

    for i, layer in enumerate(layers):
        layer_zmin = layer["z_min"]
        layer_zmax = layer["z_max"]

        # Hay solapamiento si los rangos se intersectan (límites exclusivos)
        # Un obstáculo en la frontera exacta (ej: zmax=60 con layer zmin=60) NO solapa
        if excl_zmin < layer_zmax and excl_zmax > layer_zmin:
            result.append(i + 1)

    return result if result else [1, 2, 3]  # Por defecto todas si algo falla


def get_exclusions_for_layer(self, layer_num: int) -> Tuple[List[Dict], List[Dict]]:

    circles = []
    polys = []

    for c in getattr(self, "_gf_excl_circles", []):
        if layer_num in get_exclusion_layers(self, c):
            circles.append(c)

    for p in getattr(self, "_gf_excl_polys", []):
        if layer_num in get_exclusion_layers(self, p):
            polys.append(p)

    return circles, polys


def check_layer_change(self) -> Optional[int]:

    current = get_current_layer(self)
    last = getattr(self, "_gf_last_layer", None)

    if last is None:
        self._gf_last_layer = current
        return None

    if current != last:
        self._gf_last_layer = current
        return current

    return None