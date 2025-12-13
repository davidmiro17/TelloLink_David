"""
Módulo de geometría y detección de colisiones para TelloLink.
Contiene funciones para:
- Intersección línea-círculo
- Intersección línea-rectángulo
- Intersección línea-polígono
- Punto dentro de obstáculo
- Validación de rutas contra obstáculos
- Path planning: planificación de rutas evitando obstáculos
"""

import math
from typing import List, Tuple, Dict, Any, Optional


def line_intersects_circle(x1: float, y1: float, x2: float, y2: float,
                           cx: float, cy: float, r: float) -> bool:
    """
    Comprueba si el segmento de (x1,y1) a (x2,y2) cruza el círculo en (cx,cy) con radio r.

    Args:
        x1, y1: Punto inicial del segmento
        x2, y2: Punto final del segmento
        cx, cy: Centro del círculo
        r: Radio del círculo

    Returns:
        True si el segmento cruza o toca el círculo
    """
    # Vector del segmento
    dx, dy = x2 - x1, y2 - y1
    # Vector del inicio del segmento al centro del círculo
    fx, fy = x1 - cx, y1 - cy

    a = dx * dx + dy * dy
    if a == 0:  # Punto, no segmento
        return math.sqrt(fx * fx + fy * fy) <= r

    b = 2 * (fx * dx + fy * dy)
    c = fx * fx + fy * fy - r * r

    discriminant = b * b - 4 * a * c
    if discriminant < 0:
        return False  # No hay intersección

    discriminant = math.sqrt(discriminant)
    t1 = (-b - discriminant) / (2 * a)
    t2 = (-b + discriminant) / (2 * a)

    # Comprobar si algún punto de intersección está dentro del segmento [0, 1]
    return (0 <= t1 <= 1) or (0 <= t2 <= 1) or (t1 < 0 and t2 > 1)


def segments_intersect(ax1: float, ay1: float, ax2: float, ay2: float,
                       bx1: float, by1: float, bx2: float, by2: float) -> bool:
    """
    Comprueba si dos segmentos se cruzan usando el algoritmo CCW.

    Args:
        ax1, ay1, ax2, ay2: Primer segmento
        bx1, by1, bx2, by2: Segundo segmento

    Returns:
        True si los segmentos se cruzan
    """
    def ccw(ax: float, ay: float, bx: float, by: float, cx: float, cy: float) -> bool:
        return (cy - ay) * (bx - ax) > (by - ay) * (cx - ax)

    return (ccw(ax1, ay1, bx1, by1, bx2, by2) != ccw(ax2, ay2, bx1, by1, bx2, by2) and
            ccw(ax1, ay1, ax2, ay2, bx1, by1) != ccw(ax1, ay1, ax2, ay2, bx2, by2))


def line_intersects_rect(x1: float, y1: float, x2: float, y2: float,
                         rx1: float, ry1: float, rx2: float, ry2: float) -> bool:
    """
    Comprueba si el segmento cruza el rectángulo.

    Args:
        x1, y1, x2, y2: Segmento
        rx1, ry1: Esquina inferior izquierda del rectángulo
        rx2, ry2: Esquina superior derecha del rectángulo

    Returns:
        True si el segmento cruza o está dentro del rectángulo
    """
    # Comprobar los 4 lados del rectángulo
    edges = [
        (rx1, ry1, rx2, ry1),  # Inferior
        (rx2, ry1, rx2, ry2),  # Derecho
        (rx2, ry2, rx1, ry2),  # Superior
        (rx1, ry2, rx1, ry1),  # Izquierdo
    ]
    for ex1, ey1, ex2, ey2 in edges:
        if segments_intersect(x1, y1, x2, y2, ex1, ey1, ex2, ey2):
            return True

    # También comprobar si el segmento está completamente dentro
    if rx1 <= x1 <= rx2 and ry1 <= y1 <= ry2 and rx1 <= x2 <= rx2 and ry1 <= y2 <= ry2:
        return True
    return False


def line_intersects_polygon(x1: float, y1: float, x2: float, y2: float,
                            points: List[Tuple[float, float]]) -> bool:
    """
    Comprueba si el segmento cruza el polígono.

    Args:
        x1, y1, x2, y2: Segmento
        points: Lista de vértices del polígono [(x, y), ...]

    Returns:
        True si el segmento cruza algún lado del polígono
    """
    n = len(points)
    for i in range(n):
        px1, py1 = points[i]
        px2, py2 = points[(i + 1) % n]
        if segments_intersect(x1, y1, x2, y2, px1, py1, px2, py2):
            return True
    return False


def point_in_polygon(x: float, y: float, points: List[Tuple[float, float]]) -> bool:
    """
    Comprueba si un punto está dentro de un polígono usando ray casting.

    Args:
        x, y: Punto a comprobar
        points: Lista de vértices del polígono [(x, y), ...]

    Returns:
        True si el punto está dentro del polígono
    """
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


def point_in_obstacle(x: float, y: float, obs: Dict[str, Any]) -> bool:
    """
    Comprueba si un punto está dentro de un obstáculo de cualquier tipo.

    Args:
        x, y: Punto a comprobar
        obs: Diccionario del obstáculo con 'type' y datos específicos

    Returns:
        True si el punto está dentro del obstáculo
    """
    obs_type = obs.get('type', 'circle')
    if obs_type == 'circle':
        dist = math.sqrt((x - obs['cx']) ** 2 + (y - obs['cy']) ** 2)
        return dist <= obs['r']
    elif obs_type == 'rect':
        return obs['x1'] <= x <= obs['x2'] and obs['y1'] <= y <= obs['y2']
    elif obs_type == 'poly':
        return point_in_polygon(x, y, obs['points'])
    return False


def line_intersects_obstacle(x1: float, y1: float, x2: float, y2: float,
                             obs: Dict[str, Any]) -> bool:
    """
    Comprueba si el segmento cruza un obstáculo de cualquier tipo.

    Args:
        x1, y1, x2, y2: Segmento
        obs: Diccionario del obstáculo con 'type' y datos específicos

    Returns:
        True si el segmento cruza el obstáculo
    """
    obs_type = obs.get('type', 'circle')
    if obs_type == 'circle':
        return line_intersects_circle(x1, y1, x2, y2, obs['cx'], obs['cy'], obs['r'])
    elif obs_type == 'rect':
        return line_intersects_rect(x1, y1, x2, y2, obs['x1'], obs['y1'], obs['x2'], obs['y2'])
    elif obs_type == 'poly':
        return line_intersects_polygon(x1, y1, x2, y2, obs['points'])
    return False


def get_obstacle_description(obs: Dict[str, Any]) -> str:
    """
    Devuelve una descripción legible del obstáculo.

    Args:
        obs: Diccionario del obstáculo

    Returns:
        String descriptivo del obstáculo
    """
    obs_type = obs.get('type', 'circle')
    if obs_type == 'circle':
        return f"círculo en ({obs['cx']}, {obs['cy']})"
    elif obs_type == 'rect':
        return f"rectángulo en ({obs['x1']}, {obs['y1']})"
    elif obs_type == 'poly':
        return f"polígono en ({obs['points'][0][0]}, {obs['points'][0][1]})"
    return "obstáculo"


def validate_mission_paths(waypoints: List[Dict[str, Any]],
                           obstacles: List[Dict[str, Any]],
                           return_home: bool = False) -> Tuple[bool, Optional[str]]:
    """
    Valida que las rutas entre waypoints no crucen obstáculos.

    Args:
        waypoints: Lista de waypoints [{'x', 'y', 'z', ...}, ...]
        obstacles: Lista de obstáculos [{'type', ...}, ...]
        return_home: Si True, también valida la ruta de vuelta a (0,0)

    Returns:
        Tupla (válido, mensaje_error). Si válido es True, mensaje_error es None.
    """
    if not waypoints:
        return True, None

    # Comprobar desde origen (0,0) al primer waypoint
    wp0 = waypoints[0]
    for obs in obstacles:
        if line_intersects_obstacle(0, 0, wp0['x'], wp0['y'], obs):
            return False, f"Ruta de origen a WP1 cruza {get_obstacle_description(obs)}"

    # Comprobar rutas entre waypoints consecutivos
    for i in range(len(waypoints) - 1):
        wp1 = waypoints[i]
        wp2 = waypoints[i + 1]
        for obs in obstacles:
            if line_intersects_obstacle(wp1['x'], wp1['y'], wp2['x'], wp2['y'], obs):
                return False, f"Ruta de WP{i + 1} a WP{i + 2} cruza {get_obstacle_description(obs)}"

    # Comprobar ruta de vuelta a casa si está activado
    if return_home and waypoints:
        last_wp = waypoints[-1]
        for obs in obstacles:
            if line_intersects_obstacle(last_wp['x'], last_wp['y'], 0, 0, obs):
                return False, f"Ruta de vuelta a casa cruza {get_obstacle_description(obs)}"

    return True, None


# ============================================================================
# PATH PLANNING - Planificación de rutas evitando obstáculos
# ============================================================================

_SAFETY_MARGIN = 30  # cm de margen adicional alrededor de obstáculos


def _distance(x1: float, y1: float, x2: float, y2: float) -> float:
    """Calcula la distancia euclidiana entre dos puntos."""
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def _get_obstacle_avoidance_points(obs: Dict[str, Any], margin: float = _SAFETY_MARGIN) -> List[Tuple[float, float]]:
    """
    Genera puntos candidatos para rodear un obstáculo.

    Args:
        obs: Diccionario del obstáculo
        margin: Margen de seguridad adicional en cm

    Returns:
        Lista de puntos (x, y) que rodean el obstáculo
    """
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
                            obstacles: List[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
    """Encuentra el primer obstáculo que bloquea el camino entre dos puntos."""
    for obs in obstacles:
        if line_intersects_obstacle(x1, y1, x2, y2, obs):
            return obs
    return None


def _path_is_clear(x1: float, y1: float, x2: float, y2: float,
                   obstacles: List[Dict[str, Any]]) -> bool:
    """Comprueba si el camino entre dos puntos está libre de obstáculos."""
    return _find_blocking_obstacle(x1, y1, x2, y2, obstacles) is None


def _point_is_safe(x: float, y: float, obstacles: List[Dict[str, Any]],
                   margin: float = _SAFETY_MARGIN) -> bool:
    """Comprueba si un punto está fuera de todos los obstáculos (con margen)."""
    for obs in obstacles:
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
                                max_depth: int = 5) -> List[Tuple[float, float]]:
    """
    Planifica una ruta desde (start_x, start_y) hasta (end_x, end_y) evitando obstáculos.

    Usa un algoritmo greedy que:
    1. Si el camino directo está libre, lo usa
    2. Si hay un obstáculo, busca el mejor punto para rodearlo
    3. Recursivamente planifica desde ese punto intermedio

    Args:
        start_x, start_y: Punto de inicio
        end_x, end_y: Punto de destino
        obstacles: Lista de obstáculos a evitar
        max_depth: Profundidad máxima de recursión (evita bucles infinitos)

    Returns:
        Lista de puntos [(x, y), ...] que forman la ruta.
        El primer punto es el inicio, el último es el destino.
        Si no se puede encontrar ruta, devuelve camino directo.
    """
    print(f"[plan_path] Inicio: ({start_x}, {start_y}) -> ({end_x}, {end_y}), depth={max_depth}")

    # Caso base: camino directo está libre
    is_clear = _path_is_clear(start_x, start_y, end_x, end_y, obstacles)
    print(f"[plan_path] Camino directo libre: {is_clear}")
    if is_clear:
        return [(start_x, start_y), (end_x, end_y)]

    # Caso base: máxima profundidad alcanzada
    if max_depth <= 0:
        print("[plan_path] Max depth alcanzado, retornando camino directo")
        return [(start_x, start_y), (end_x, end_y)]

    # Encontrar el obstáculo que bloquea
    blocking_obs = _find_blocking_obstacle(start_x, start_y, end_x, end_y, obstacles)
    print(f"[plan_path] Obstáculo bloqueante: {blocking_obs}")
    if blocking_obs is None:
        return [(start_x, start_y), (end_x, end_y)]

    # Obtener puntos candidatos para rodear el obstáculo
    avoidance_points = _get_obstacle_avoidance_points(blocking_obs)
    print(f"[plan_path] Puntos candidatos: {len(avoidance_points)} -> {avoidance_points}")

    # Filtrar puntos que estén dentro de otros obstáculos
    safe_points = [(px, py) for px, py in avoidance_points
                   if _point_is_safe(px, py, obstacles)]
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
        if not _path_is_clear(start_x, start_y, px, py, obstacles):
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
        obstacles, max_depth - 1
    )

    path_from_intermediate = plan_path_around_obstacles(
        best_point[0], best_point[1], end_x, end_y,
        obstacles, max_depth - 1
    )

    # Combinar rutas (evitar duplicar el punto intermedio)
    full_path = path_to_intermediate[:-1] + path_from_intermediate

    return full_path


def plan_mission_with_avoidance(waypoints: List[Dict[str, Any]],
                                 obstacles: List[Dict[str, Any]],
                                 return_home: bool = False) -> List[Dict[str, Any]]:
    """
    Genera una nueva lista de waypoints que evita obstáculos.

    Para cada tramo entre waypoints, si hay obstáculo, inserta
    waypoints intermedios para rodearlo.

    Args:
        waypoints: Lista original de waypoints [{'x', 'y', 'z', ...}, ...]
        obstacles: Lista de obstáculos
        return_home: Si True, incluye ruta de vuelta a (0,0)

    Returns:
        Nueva lista de waypoints con puntos intermedios añadidos.
        Los waypoints intermedios tienen z=None (mantiene altura actual).
    """
    if not waypoints:
        return []

    if not obstacles:
        return list(waypoints)

    result = []

    # Ruta desde origen (0,0) al primer waypoint
    wp0 = waypoints[0]
    path_to_first = plan_path_around_obstacles(0, 0, wp0['x'], wp0['y'], obstacles)

    # Añadir puntos intermedios (excluir origen, incluir destino)
    for i, (px, py) in enumerate(path_to_first[1:-1]):
        result.append({
            'x': px, 'y': py, 'z': None,  # z=None = mantener altura
            '_intermediate': True
        })

    # Procesar cada waypoint original
    for i, wp in enumerate(waypoints):
        # Añadir el waypoint original
        result.append(dict(wp))

        # Si hay siguiente waypoint, planificar ruta
        if i < len(waypoints) - 1:
            next_wp = waypoints[i + 1]
            path = plan_path_around_obstacles(wp['x'], wp['y'], next_wp['x'], next_wp['y'], obstacles)
            print(f"[path_planning] WP{i+1} a WP{i+2}: path retornó {len(path)} puntos: {path}")

            # Añadir puntos intermedios (excluir inicio y fin)
            intermediate_points = path[1:-1]
            print(f"[path_planning] Puntos intermedios a agregar: {intermediate_points}")
            for px, py in intermediate_points:
                result.append({
                    'x': px, 'y': py, 'z': None,
                    '_intermediate': True
                })

    # Ruta de vuelta  casa
    if return_home and waypoints:
        last_wp = waypoints[-1]
        path_home = plan_path_around_obstacles(last_wp['x'], last_wp['y'], 0, 0, obstacles)

        # Añadir puntos intermedios (excluir inicio, incluir destino 0,0)
        for px, py in path_home[1:]:
            result.append({
                'x': px, 'y': py, 'z': None,
                '_intermediate': True
            })

    return result