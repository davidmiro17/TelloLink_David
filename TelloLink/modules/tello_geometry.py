import math
from typing import List, Tuple, Dict, Any, Optional


#Función que calcula si la ruta entre un punto y otro del dron cruza un obstáculo circular (círculo)

def line_intersects_circle(x1: float, y1: float, x2: float, y2: float,
                           cx: float, cy: float, r: float) -> bool:

    # Vector del segmento, la línea entre dos waypoints o entre el dron y un waypoint
    dx, dy = x2 - x1, y2 - y1
    # Vector del inicio del segmento al centro del círculo, la línea entre el centro del círculo y la posición del dron
    fx, fy = x1 - cx, y1 - cy

    #Cuadrado de la longitud del segmento
    a = dx * dx + dy * dy
    if a == 0:  # Punto, no segmento (ya que el inicio y el fin es el mismo punto).
        return math.sqrt(fx * fx + fy * fy) <= r #True si el punto está dentro o en el borde del círculo, false si está fuera

    #Producto escalar multiplicado por 2
    b = 2 * (fx * dx + fy * dy)

    #Se compara la distancia del punto al centro del circulo al cuadrado con el radio al cuadrado. Si c es menor que 0,. Es decir c nos dice donde empieza el segmento respecto al círculo
    c = fx * fx + fy * fy - r * r

    discriminante = b * b - 4 * a * c
    if discriminante < 0:
        return False  # No hay solución real, es decir, hay intersección

    discriminant = math.sqrt(discriminante)
    t1 = (-b - discriminant) / (2 * a)  #Primer punto donde la línea toca el círculo (entrada)
    t2 = (-b + discriminant) / (2 * a) #Segundo punto donde la línea toca el círculo (salida)

    # Comprobar si algún punto de intersección está dentro del segmento [0, 1], es decir si el punto de entrada o de salida esta dentro del segmento, o que el segmento esté dentro del círculo
    return (0 <= t1 <= 1) or (0 <= t2 <= 1) or (t1 < 0 and t2 > 1)


#Función que comprueba si dos segmentos se cruzan

def segments_intersect(ax1: float, ay1: float, ax2: float, ay2: float,
                       bx1: float, by1: float, bx2: float, by2: float) -> bool:

    #Función auxiliar para saber si yendo de "a" a "b" y a "c", se va en sentido horario o antihorario
    def ccw(ax: float, ay: float, bx: float, by: float, cx: float, cy: float) -> bool:
        return (cy - ay) * (bx - ax) > (by - ay) * (cx - ax) #Producto vectorial, si es positivo giro antihorario (True), si es negativo giro horario (False)

#Para detectar si dos extremos se cruzan, se han de cumplir las dos condiciones: los extremos del segmento A están en lados opuestos del segmento B y los extremos del segmento B están en lados opuestos del segmento A
    return (ccw(ax1, ay1, bx1, by1, bx2, by2) != ccw(ax2, ay2, bx1, by1, bx2, by2) and
            ccw(ax1, ay1, ax2, ay2, bx1, by1) != ccw(ax1, ay1, ax2, ay2, bx2, by2))


#Función que comprueba si un segmento cruza un rectángulo
def line_intersects_rect(x1: float, y1: float, x2: float, y2: float,
                         rx1: float, ry1: float, rx2: float, ry2: float) -> bool:

    # Comprobar los 4 lados del rectángulo
    edges = [
        (rx1, ry1, rx2, ry1),  # Inferior
        (rx2, ry1, rx2, ry2),  # Derecho
        (rx2, ry2, rx1, ry2),  # Superior
        (rx1, ry2, rx1, ry1),  # Izquierdo
    ]

    #Comprueba con la función anterior si el segmento cruza ese lado
    for ex1, ey1, ex2, ey2 in edges:
        if segments_intersect(x1, y1, x2, y2, ex1, ey1, ex2, ey2):
            return True

    # También comprobar si el segmento está completamente dentro
    if rx1 <= x1 <= rx2 and ry1 <= y1 <= ry2 and rx1 <= x2 <= rx2 and ry1 <= y2 <= ry2:
        return True
    return False

#Función que comprueba si un segmento cruza un polígono
def line_intersects_polygon(x1: float, y1: float, x2: float, y2: float,
                            points: List[Tuple[float, float]]) -> bool:
    #Guarda el número de vertices del polígono
    n = len(points)
    #Recorre cada lado del polígono y comprueba si el segmento lo cruza
    for i in range(n):
        px1, py1 = points[i]
        px2, py2 = points[(i + 1) % n]
        if segments_intersect(x1, y1, x2, y2, px1, py1, px2, py2):
            return True
    return False


#Comprueba si un punto está dentro del polígono, con el algoritmo ray casting
def point_in_polygon(x: float, y: float, points: List[Tuple[float, float]]) -> bool:

    n = len(points)
    inside = False
    j = n - 1
    for i in range(n):
        xi, yi = points[i]
        xj, yj = points[j]
        if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi) + xi):
            inside = not inside
        j = i
    return inside


# ============================================================================
# SOPORTE Z/CAPAS - Verificación de altura para obstáculos 3D
# ============================================================================

def _z_overlaps_obstacle(z: Optional[float], obs: Dict[str, Any]) -> bool:
    """
    Comprueba si una altura Z está dentro del rango vertical del obstáculo.

    Args:
        z: Altura a comprobar (None = asumir que sí solapa)
        obs: Diccionario del obstáculo con posibles 'zmin' y 'zmax'

    Returns:
        True si Z está dentro del rango del obstáculo (o si no hay info de Z)
    """
    if z is None:
        return True  # Sin info de altura, asumir que solapa

    obs_zmin = obs.get('zmin')
    obs_zmax = obs.get('zmax')

    # Si el obstáculo no tiene límites de Z, afecta a todas las alturas
    if obs_zmin is None and obs_zmax is None:
        return True

    obs_zmin = float(obs_zmin) if obs_zmin is not None else 0.0
    obs_zmax = float(obs_zmax) if obs_zmax is not None else 999.0

    # Comprobar si Z está dentro del rango
    return obs_zmin <= z <= obs_zmax


#Función que comprueba si un punto está dentro de un obstáculo, sin importar el tipo de obstáculo
def point_in_obstacle(x: float, y: float, obs: Dict[str, Any], z: Optional[float] = None) -> bool:
    """
    Comprueba si un punto está dentro de un obstáculo de cualquier tipo.

    Args:
        x, y: Punto a comprobar
        obs: Diccionario del obstáculo con 'type' y datos específicos
        z: Altura del punto (opcional). Si se proporciona, verifica también
           que Z esté dentro del rango zmin/zmax del obstáculo.

    Returns:
        True si el punto está dentro del obstáculo
    """
    # Primero verificar si la altura Z solapa con el obstáculo
    if not _z_overlaps_obstacle(z, obs):
        return False  # Fuera del rango vertical, no hay colisión

    obs_type = obs.get('type', 'circle')
    if obs_type == 'circle':
        dist = math.sqrt((x - obs['cx']) ** 2 + (y - obs['cy']) ** 2)
        return dist <= obs['r']
    elif obs_type == 'rect':
        return obs['x1'] <= x <= obs['x2'] and obs['y1'] <= y <= obs['y2']
    elif obs_type == 'poly':
        return point_in_polygon(x, y, obs['points'])
    return False

#Función que comprueba si un segmento cruza un obstáculo sin importar el tipo de obstáculo
def line_intersects_obstacle(x1: float, y1: float, x2: float, y2: float,
                             obs: Dict[str, Any],
                             z1: Optional[float] = None, z2: Optional[float] = None) -> bool:
    """
    Comprueba si el segmento cruza un obstáculo de cualquier tipo.

    Args:
        x1, y1, x2, y2: Segmento en 2D
        obs: Diccionario del obstáculo con 'type' y datos específicos
        z1, z2: Alturas del inicio y fin del segmento (opcionales).
                Si se proporcionan, verifica que el rango Z del segmento
                solape con el rango zmin/zmax del obstáculo.

    Returns:
        True si el segmento cruza el obstáculo
    """
    # Verificar si el rango de Z del segmento solapa con el obstáculo
    obs_zmin = obs.get('zmin')
    obs_zmax = obs.get('zmax')

    # Si el obstáculo tiene límites de Z, verificar solapamiento
    if obs_zmin is not None or obs_zmax is not None:
        obs_zmin = float(obs_zmin) if obs_zmin is not None else 0.0
        obs_zmax = float(obs_zmax) if obs_zmax is not None else 999.0

        # Si tenemos Z del segmento, comprobar solapamiento
        if z1 is not None and z2 is not None:
            seg_zmin = min(z1, z2)
            seg_zmax = max(z1, z2)
            # No hay solapamiento si el segmento está completamente fuera del obstáculo
            if seg_zmax < obs_zmin or seg_zmin > obs_zmax:
                return False  # Segmento fuera del rango vertical del obstáculo
        elif z1 is not None:
            # Solo tenemos Z del inicio
            if z1 < obs_zmin or z1 > obs_zmax:
                return False
        elif z2 is not None:
            # Solo tenemos Z del final
            if z2 < obs_zmin or z2 > obs_zmax:
                return False
        # Si no tenemos Z, asumir que solapa (conservador)

    obs_type = obs.get('type', 'circle')
    if obs_type == 'circle':
        return line_intersects_circle(x1, y1, x2, y2, obs['cx'], obs['cy'], obs['r'])
    elif obs_type == 'rect':
        return line_intersects_rect(x1, y1, x2, y2, obs['x1'], obs['y1'], obs['x2'], obs['y2'])
    elif obs_type == 'poly':
        return line_intersects_polygon(x1, y1, x2, y2, obs['points'])
    return False

#Función que genera una descripción del obstáculo para mostrar en mensajes de error
def get_obstacle_description(obs: Dict[str, Any]) -> str:

    obs_type = obs.get('type', 'circle')
    if obs_type == 'circle':
        return f"círculo en ({obs['cx']}, {obs['cy']})"
    elif obs_type == 'rect':
        return f"rectángulo en ({obs['x1']}, {obs['y1']})"
    elif obs_type == 'poly':
        return f"polígono en ({obs['points'][0][0]}, {obs['points'][0][1]})"
    return "obstáculo"


#Función que valida que toda la misión no cruce ningún obstáculo
def validate_mission_paths(waypoints: List[Dict[str, Any]],
                           obstacles: List[Dict[str, Any]],
                           return_home: bool = False,
                           starting_z: float = 50.0) -> Tuple[bool, Optional[str]]:
    """
    Valida que las rutas entre waypoints no crucen obstáculos.

    Args:
        waypoints: Lista de waypoints [{'x', 'y', 'z', ...}, ...]
        obstacles: Lista de obstáculos [{'type', ...}, ...]
        return_home: Si True, también valida la ruta de vuelta a (0,0)
        starting_z: Altura inicial del dron en cm (default 50cm tras despegue)

    Returns:
        Tupla (válido, mensaje_error). Si válido es True, mensaje_error es None.
    """
    if not waypoints:
        return True, None

    # Comprobar desde origen (0,0) al primer waypoint
    wp0 = waypoints[0]
    z0 = wp0.get('z')  # Altura del primer waypoint (puede ser None)
    for obs in obstacles:
        # z1=starting_z (altura inicial), z2=altura del primer WP
        if line_intersects_obstacle(0, 0, wp0['x'], wp0['y'], obs,
                                    z1=starting_z, z2=z0 if z0 is not None else starting_z):
            return False, f"Ruta de origen a WP1 cruza {get_obstacle_description(obs)}"

    # Comprobar rutas entre waypoints consecutivos
    current_z = z0 if z0 is not None else starting_z  # Trackear altura actual
    for i in range(len(waypoints) - 1):
        wp1 = waypoints[i]
        wp2 = waypoints[i + 1]
        z1 = wp1.get('z')
        z2 = wp2.get('z')
        # Si Z es None, usar la altura actual (mantener)
        z1_val = z1 if z1 is not None else current_z
        z2_val = z2 if z2 is not None else current_z
        for obs in obstacles:
            if line_intersects_obstacle(wp1['x'], wp1['y'], wp2['x'], wp2['y'], obs,
                                        z1=z1_val, z2=z2_val):
                return False, f"Ruta de WP{i + 1} a WP{i + 2} cruza {get_obstacle_description(obs)}"
        # Actualizar altura actual
        if z2 is not None:
            current_z = z2

    # Comprobar ruta de vuelta a casa si está activado
    if return_home and waypoints:
        last_wp = waypoints[-1]
        last_z = last_wp.get('z')
        last_z_val = last_z if last_z is not None else current_z
        for obs in obstacles:
            # Volver a casa a la altura actual
            if line_intersects_obstacle(last_wp['x'], last_wp['y'], 0, 0, obs,
                                        z1=last_z_val, z2=last_z_val):
                return False, f"Ruta de vuelta a casa cruza {get_obstacle_description(obs)}"

    return True, None



# PATH PLANNING - Planificación de rutas evitando obstáculos


_SAFETY_MARGIN = 30  # cm de margen adicional alrededor de obstáculos


def _distance(x1: float, y1: float, x2: float, y2: float) -> float:

    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def _get_obstacle_avoidance_points(obs: Dict[str, Any], margin: float = _SAFETY_MARGIN) -> List[Tuple[float, float]]:

    obs_type = obs.get('type', 'circle')

    if obs_type == 'circle':
        cx, cy, r = obs['cx'], obs['cy'], obs['r']
        r_total = r + margin
        # 8 puntos alrededor del círculo
        points = []
        for angle in range(0, 360, 45):
            rad = math.radians(angle)
            px = cx + r_total * math.cos(rad)
            py = cy + r_total * math.sin(rad)
            points.append((px, py))
        return points

    elif obs_type == 'rect':
        x1, y1, x2, y2 = obs['x1'], obs['y1'], obs['x2'], obs['y2']
        # Generar múltiples puntos alrededor del rectángulo con diferentes distancias
        points = []
        # Margen más grande para tener más opciones
        margins = [margin, margin * 2, margin * 3]
        for m in margins:
            # 4 esquinas
            points.append((x1 - m, y1 - m))  # Inferior izquierda
            points.append((x2 + m, y1 - m))  # Inferior derecha
            points.append((x2 + m, y2 + m))  # Superior derecha
            points.append((x1 - m, y2 + m))  # Superior izquierda
            # Puntos en los centros de cada lado
            mid_x = (x1 + x2) / 2
            mid_y = (y1 + y2) / 2
            points.append((mid_x, y1 - m))  # Centro inferior
            points.append((mid_x, y2 + m))  # Centro superior
            points.append((x1 - m, mid_y))  # Centro izquierdo
            points.append((x2 + m, mid_y))  # Centro derecho
        return points

    elif obs_type == 'poly':
        points = obs['points']
        # Calcular centroide
        cx = sum(p[0] for p in points) / len(points)
        cy = sum(p[1] for p in points) / len(points)
        # Expandir cada vértice hacia afuera desde el centroide
        expanded = []
        for px, py in points:
            dx, dy = px - cx, py - cy
            dist = math.sqrt(dx * dx + dy * dy)
            if dist > 0:
                nx, ny = dx / dist, dy / dist
                expanded.append((px + nx * margin, py + ny * margin))
            else:
                expanded.append((px, py))
        return expanded

    return []


def _find_blocking_obstacle(x1: float, y1: float, x2: float, y2: float,
                            obstacles: List[Dict[str, Any]],
                            z1: Optional[float] = None,
                            z2: Optional[float] = None) -> Optional[Dict[str, Any]]:
    """
    Encuentra el primer obstáculo que bloquea el camino entre dos puntos.

    Args:
        x1, y1: Punto inicial
        x2, y2: Punto final
        obstacles: Lista de obstáculos
        z1, z2: Alturas del inicio y fin del segmento (opcionales)
    """
    for obs in obstacles:
        if line_intersects_obstacle(x1, y1, x2, y2, obs, z1=z1, z2=z2):
            return obs
    return None


def _path_is_clear(x1: float, y1: float, x2: float, y2: float,
                   obstacles: List[Dict[str, Any]],
                   z1: Optional[float] = None,
                   z2: Optional[float] = None) -> bool:
    """Comprueba si el camino entre dos puntos está libre de obstáculos."""
    return _find_blocking_obstacle(x1, y1, x2, y2, obstacles, z1=z1, z2=z2) is None


def _point_is_safe(x: float, y: float, obstacles: List[Dict[str, Any]],
                   margin: float = _SAFETY_MARGIN,
                   z: Optional[float] = None) -> bool:
    """
    Comprueba si un punto está fuera de todos los obstáculos (con margen).

    Args:
        x, y: Coordenadas del punto
        obstacles: Lista de obstáculos
        margin: Margen de seguridad adicional en cm
        z: Altura del punto (opcional). Si se proporciona, solo comprueba
           obstáculos cuyo rango zmin/zmax incluya esta altura.
    """
    for obs in obstacles:
        # Si tenemos Z, verificar si el obstáculo afecta a esta altura
        if z is not None and not _z_overlaps_obstacle(z, obs):
            continue  # Este obstáculo no afecta a esta altura

        obs_type = obs.get('type', 'circle')
        if obs_type == 'circle':
            dist = _distance(x, y, obs['cx'], obs['cy'])
            if dist < obs['r'] + margin:
                return False
        elif obs_type == 'rect':
            if (obs['x1'] - margin <= x <= obs['x2'] + margin and
                obs['y1'] - margin <= y <= obs['y2'] + margin):
                return False
        elif obs_type == 'poly':
            if point_in_polygon(x, y, obs['points']):
                return False
    return True


def plan_path_around_obstacles(start_x: float, start_y: float,
                                end_x: float, end_y: float,
                                obstacles: List[Dict[str, Any]],
                                max_depth: int = 5,
                                z: Optional[float] = None) -> List[Tuple[float, float]]:
    """
    Planifica una ruta evitando obstáculos.

    Args:
        start_x, start_y: Punto de inicio
        end_x, end_y: Punto de destino
        obstacles: Lista de obstáculos a evitar
        max_depth: Profundidad máxima de recursión
        z: Altura del vuelo (opcional). Si se proporciona, solo considera
           obstáculos que afecten a esta altura (según zmin/zmax).

    Returns:
        Lista de puntos [(x, y), ...] que forman la ruta.
    """
    print(f"[plan_path] Inicio: ({start_x}, {start_y}) -> ({end_x}, {end_y}), z={z}, depth={max_depth}")

    # Caso base: camino directo está libre
    is_clear = _path_is_clear(start_x, start_y, end_x, end_y, obstacles, z1=z, z2=z)
    print(f"[plan_path] Camino directo libre: {is_clear}")
    if is_clear:
        return [(start_x, start_y), (end_x, end_y)]

    # Caso base: máxima profundidad alcanzada
    if max_depth <= 0:
        print("[plan_path] Max depth alcanzado, retornando camino directo")
        return [(start_x, start_y), (end_x, end_y)]

    # Encontrar el obstáculo que bloquea (a esta altura)
    blocking_obs = _find_blocking_obstacle(start_x, start_y, end_x, end_y, obstacles, z1=z, z2=z)
    print(f"[plan_path] Obstáculo bloqueante: {blocking_obs}")
    if blocking_obs is None:
        return [(start_x, start_y), (end_x, end_y)]

    # Obtener puntos candidatos para rodear el obstáculo
    avoidance_points = _get_obstacle_avoidance_points(blocking_obs)
    print(f"[plan_path] Puntos candidatos: {len(avoidance_points)} -> {avoidance_points}")

    # Filtrar puntos que estén dentro de otros obstáculos (a esta altura)
    safe_points = [(px, py) for px, py in avoidance_points
                   if _point_is_safe(px, py, obstacles, z=z)]
    print(f"[plan_path] Puntos seguros: {len(safe_points)} -> {safe_points}")

    if not safe_points:
        print("[plan_path] No hay puntos seguros, retornando camino directo")
        return [(start_x, start_y), (end_x, end_y)]

    # Buscar el mejor punto intermedio
    # Criterio: menor distancia total (start -> punto -> end) + camino libre desde start
    best_point = None
    best_score = float('inf')

    for px, py in safe_points:
        # NO elegir el punto desde el que ya estamos (evita bucles infinitos)
        if abs(px - start_x) < 2 and abs(py - start_y) < 2:
            continue

        # Solo considerar puntos alcanzables desde el inicio
        if not _path_is_clear(start_x, start_y, px, py, obstacles, z1=z, z2=z):
            continue

        # Calcular distancia total
        dist_to_point = _distance(start_x, start_y, px, py)
        dist_to_end = _distance(px, py, end_x, end_y)
        total_dist = dist_to_point + dist_to_end

        if total_dist < best_score:
            best_score = total_dist
            best_point = (px, py)

    # Si no encontramos punto alcanzable, probar todos
    if best_point is None:
        for px, py in safe_points:
            # NO elegir el punto desde el que ya estamos (evita bucles infinitos)
            if abs(px - start_x) < 2 and abs(py - start_y) < 2:
                continue

            dist_to_point = _distance(start_x, start_y, px, py)
            dist_to_end = _distance(px, py, end_x, end_y)
            total_dist = dist_to_point + dist_to_end

            if total_dist < best_score:
                best_score = total_dist
                best_point = (px, py)

    if best_point is None:
        return [(start_x, start_y), (end_x, end_y)]

    # Planificar recursivamente: start -> punto intermedio -> end
    path_to_intermediate = plan_path_around_obstacles(
        start_x, start_y, best_point[0], best_point[1],
        obstacles, max_depth - 1, z=z
    )

    path_from_intermediate = plan_path_around_obstacles(
        best_point[0], best_point[1], end_x, end_y,
        obstacles, max_depth - 1, z=z
    )

    # Combinar rutas (evitar duplicar el punto intermedio)
    full_path = path_to_intermediate[:-1] + path_from_intermediate

    return full_path


def plan_mission_with_avoidance(waypoints: List[Dict[str, Any]],
                                 obstacles: List[Dict[str, Any]],
                                 return_home: bool = False,
                                 starting_z: float = 50.0) -> List[Dict[str, Any]]:
    """
    Genera una nueva lista de waypoints que evita obstáculos.

    Args:
        waypoints: Lista original de waypoints [{'x', 'y', 'z', ...}, ...]
        obstacles: Lista de obstáculos
        return_home: Si True, incluye ruta de vuelta a (0,0)
        starting_z: Altura inicial del dron en cm (default 50cm tras despegue)

    Returns:
        Nueva lista de waypoints con puntos intermedios añadidos.
    """
    if not waypoints:
        return []

    if not obstacles:
        return list(waypoints)

    result = []

    # Ruta desde origen (0,0) al primer waypoint
    wp0 = waypoints[0]
    # Usar la altura inicial para el path planning al primer WP
    z0 = wp0.get('z')
    initial_z = z0 if z0 is not None else starting_z
    path_to_first = plan_path_around_obstacles(0, 0, wp0['x'], wp0['y'], obstacles, z=initial_z)

    # Añadir puntos intermedios (excluir origen, incluir destino)
    for i, (px, py) in enumerate(path_to_first[1:-1]):
        result.append({
            'x': px, 'y': py, 'z': None,  # z=None = mantener altura
            '_intermediate': True
        })

    # Trackear la altura actual para el path planning
    current_z = initial_z

    # Procesar cada waypoint original
    for i, wp in enumerate(waypoints):
        # Añadir el waypoint original
        result.append(dict(wp))

        # Actualizar altura actual si el waypoint la especifica
        wp_z = wp.get('z')
        if wp_z is not None:
            current_z = wp_z

        # Si hay siguiente waypoint, planificar ruta
        if i < len(waypoints) - 1:
            next_wp = waypoints[i + 1]
            # Usar la altura del siguiente WP si la especifica, sino la actual
            next_z = next_wp.get('z')
            plan_z = next_z if next_z is not None else current_z
            path = plan_path_around_obstacles(wp['x'], wp['y'], next_wp['x'], next_wp['y'], obstacles, z=plan_z)
            print(f"[path_planning] WP{i+1} a WP{i+2} (z={plan_z}): path retornó {len(path)} puntos: {path}")

            # Añadir puntos intermedios (excluir inicio y fin)
            intermediate_points = path[1:-1]
            print(f"[path_planning] Puntos intermedios a agregar: {intermediate_points}")
            for px, py in intermediate_points:
                result.append({
                    'x': px, 'y': py, 'z': None,
                    '_intermediate': True
                })

    # Ruta de vuelta a casa
    if return_home and waypoints:
        last_wp = waypoints[-1]
        last_z = last_wp.get('z')
        home_z = last_z if last_z is not None else current_z
        path_home = plan_path_around_obstacles(last_wp['x'], last_wp['y'], 0, 0, obstacles, z=home_z)

        # Añadir puntos intermedios (excluir inicio, incluir destino 0,0)
        for px, py in path_home[1:]:
            result.append({
                'x': px, 'y': py, 'z': None,
                '_intermediate': True
            })

    return result