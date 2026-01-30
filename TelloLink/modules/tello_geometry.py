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






#Funcion para comprobar si una altura Z está dentro del rango vertical del obstáculo
def _z_overlaps_obstacle(z: Optional[float], obs: Dict[str, Any]) -> bool:

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
                           starting_z: float = 50.0) -> Tuple[bool, Optional[str]]:

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

    return True, None