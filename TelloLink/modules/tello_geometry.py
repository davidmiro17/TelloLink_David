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

#Función que comprueba si un punto está dentro de un obstáculo, sin importar el tipo de obstáculo
def point_in_obstacle(x: float, y: float, obs: Dict[str, Any]) -> bool:

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
                             obs: Dict[str, Any]) -> bool:

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
                           return_home: bool = False) -> Tuple[bool, Optional[str]]:

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
                            obstacles: List[Dict[str, Any]]) -> Optional[Dict[str, Any]]:

    for obs in obstacles:
        if line_intersects_obstacle(x1, y1, x2, y2, obs):
            return obs
    return None


def _path_is_clear(x1: float, y1: float, x2: float, y2: float,
                   obstacles: List[Dict[str, Any]]) -> bool:

    return _find_blocking_obstacle(x1, y1, x2, y2, obstacles) is None


def _point_is_safe(x: float, y: float, obstacles: List[Dict[str, Any]],
                   margin: float = _SAFETY_MARGIN) -> bool:

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