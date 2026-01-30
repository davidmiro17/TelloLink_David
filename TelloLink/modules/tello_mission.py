from __future__ import annotations
import threading
import time
from typing import Any, Dict, List, Optional, Callable


_MIN_BAT_PCT = 20  # batería mínima para ejecutar una misión

def _is_abs_wp(wp: Dict[str, Any]) -> bool:
    has_abs = all(k in wp for k in ("x", "y", "z"))
    has_rel = all(k in wp for k in ("dx", "dy", "dz"))
    if not has_abs and not has_rel:
        raise ValueError("Waypoint incoherente: debe ser absoluto (x,y,z) o relativo (dx,dy,dz)")
    return has_abs  # True si absoluto, False si relativo


def _validate_and_normalize(waypoints: List[Dict[str, Any]]) -> List[Dict[str, Any]]: #Función que sirve para normalizar y preparar la lista de waypoints

    norm: List[Dict[str, Any]] = [] #Crea una lista vacía donde se guardarán los waypoints limpios
    for i, wp in enumerate(waypoints, start=1): #recorre cada waypoint junto con su índice
        if not isinstance(wp, dict): #Comprueba que cada waypoint sea un diccionario, si se pasa otra cosa lanza error
            raise ValueError(f"WP{i}: cada waypoint debe ser dict, recibido: {type(wp)}")
        yaw = wp.get("yaw", None) #Si se especifica el valor "yaw", lo extrae, si no, es None.
        delay = float(wp.get("delay", 0.0) or 0.0) #Convierte el valor "delay" a float, y si no existe usa 0.0
        if delay < 0: #Si es negativo, lanza error
            raise ValueError(f"WP{i}: delay no puede ser negativo")

        if _is_abs_wp(wp):
            # Absoluto: x/y/z en cm
            x = wp.get("x", None); y = wp.get("y", None); z = wp.get("z", None) #Se leen "x", "y" y "z"
            x = None if x is None else float(x) #Si el valor es None, lo deja en None, si existe lo convierte a float
            y = None if y is None else float(y)
            z = None if z is None else float(z)
            yaw_f = None if yaw is None else float(yaw) #Se realiza lo mismo
            norm.append({"mode": "abs", "x": x, "y": y, "z": z, "yaw": yaw_f, "delay": delay}) #Se añade a lista normalizada un diccionario limpio
        else:
            # Relativo: dx/dy/dz en cm (por defecto 0). Se buscan los valores y si no existen se dejan en 0.0.
            dx = float(wp.get("dx", 0.0) or 0.0)
            dy = float(wp.get("dy", 0.0) or 0.0)
            dz = float(wp.get("dz", 0.0) or 0.0)
            yaw_f = None if yaw is None else float(yaw)
            norm.append({"mode": "rel", "dx": dx, "dy": dy, "dz": dz, "yaw": yaw_f, "delay": delay})
    return norm


#Ésta función convierte un waypoint absoluto en un movimiento relativo, así se puede ejecutar, ya que Tello no entiende de coordenadas absolutas
def _rel_from_abs(self, wp_abs: Dict[str, Any]) -> tuple[float, float, float]:

    if not hasattr(self, "pose") or self.pose is None: #Si el atributo "pose" no existe o es None
        raise RuntimeError("PoseVirtual requerida para waypoints absolutos.") #lanza un error

    #Se calcula la posición absoluta de destino. Si en los waypoints están las coordenadas, se usan, si falta alguna, toma la actual del dron
    x_goal = wp_abs["x"] if wp_abs["x"] is not None else float(self.pose.x_cm)
    y_goal = wp_abs["y"] if wp_abs["y"] is not None else float(self.pose.y_cm)
    z_goal = wp_abs["z"] if wp_abs["z"] is not None else float(self.pose.z_cm)

    #Se calcula el desplazamiento necesario
    dx = float(x_goal) - float(self.pose.x_cm)
    dy = float(y_goal) - float(self.pose.y_cm)
    dz = float(z_goal) - float(self.pose.z_cm)

    # DEBUG: Mostrar cálculo de movimiento
    print(f"[_rel_from_abs] ════════════════════════════════════")
    print(f"[_rel_from_abs] Pose actual: x={self.pose.x_cm:.1f}, y={self.pose.y_cm:.1f}, z={self.pose.z_cm:.1f}, yaw={self.pose.yaw_deg:.1f}°")
    print(f"[_rel_from_abs] Destino WP:  x={x_goal:.1f}, y={y_goal:.1f}, z={z_goal:.1f}")
    print(f"[_rel_from_abs] Delta calc:  dx={dx:.1f}, dy={dy:.1f}, dz={dz:.1f}")
    print(f"[_rel_from_abs] ════════════════════════════════════")

    return dx, dy, dz


def _get_layer_for_z(z: float, layers: List[Dict[str, Any]]) -> int:
    """Determina en qué capa está una altura z. Retorna índice 0-based."""
    for i, layer in enumerate(layers):
        if layer["z_min"] <= z < layer["z_max"]:
            return i
    # Si está por encima de todas las capas, retorna la última
    return len(layers) - 1


def _mission_worker(self,
                    waypoints: List[Dict[str, Any]],
                    do_land: bool = True,
                    face_target: bool = False,
                    layers: Optional[List[Dict[str, Any]]] = None,
                    on_wp: Optional[Callable[[int, Dict[str, Any]], None]] = None,
                    on_action: Optional[Callable[[int, str], None]] = None,
                    on_finish: Optional[Callable[[], None]] = None) -> None:
    #Flag de aborto
    setattr(self, "_mission_abort", False)

    # Validación inicial con la función anteriormente diseñada, si algo falla la mision se aborta
    try:
        wps = _validate_and_normalize(waypoints)
    except Exception as e:
        print(f"[mission] Waypoints inválidos: {e}")
        return

    # Chequeos básicos de seguridad (se asegura de que el dron esté conectado y que tenga batería suficiente)
    if getattr(self, "state", "") == "disconnected":
        print("[mission] Dron desconectado; abortando.")
        return
    bat = getattr(self, "battery_pct", None)
    if isinstance(bat, int) and bat < _MIN_BAT_PCT:
        print(f"[mission] Batería baja ({bat}%), abortando.")
        return

    # Despegue si hace falta. Si el dron está en tierra despega a una altura segura de 0,5 metros, si no se puede despegar, aborta la misión
    if getattr(self, "state", "") != "flying":
        print("[mission] Dron en tierra: despegando a 0.5 m")
        if not self.takeOff(0.5, blocking=True):
            print("[mission] No se pudo despegar; abortando.")
            return
        time.sleep(0.4)

    # Recorremos los waypoints numerados.
    for idx, wp in enumerate(wps, start=1):
        if getattr(self, "_mission_abort", False): #Si se pide _mission_abort desde fuera, el bucle termina
            print("[mission] Abortada por solicitud externa.")
            break

        # Se vuelve a comprobar la batería antes de cada movimiento
        bat = getattr(self, "battery_pct", None)
        if isinstance(bat, int) and bat < _MIN_BAT_PCT:
            print(f"[mission] Abortada por batería ({bat}%).")
            break

        #Si el destino es relativo, se usa directamente dx/dy/dz
        if wp["mode"] == "rel":
            dx, dy, dz = wp["dx"], wp["dy"], wp["dz"]
            target_desc = f"REL: dx={dx:.1f}, dy={dy:.1f}, dz={dz:.1f}"
        #Si es absoluto, llama a _rel_from_abs para convertirlo en relativo
        else:
            try:
                dx, dy, dz = _rel_from_abs(self, wp)
            except Exception as e:
                print(f"[mission] WP{idx} absoluto inválido: {e}")
                break
            target_desc = f"ABS: x={wp['x'] if wp['x'] is not None else 'poseX'}," \
                          f" y={wp['y'] if wp['y'] is not None else 'poseY'}," \
                          f" z={wp['z'] if wp['z'] is not None else 'poseZ'}"

        yaw = wp.get("yaw", None)
        delay = float(wp.get("delay", 0.0) or 0.0)

        print(f"[mission] WP{idx} → {target_desc}, yaw={yaw}, delay={delay}s")


        if on_wp:
            try:
                on_wp(idx, dict(wp))
            except Exception:
                pass

        # Ejecuta el movimiento y manda el dron hacia el waypoint
        try:
            # Nota: goto_rel ya maneja yaw opcional al inicio del movimiento
            # face_target hace que el dron rote para mirar hacia el destino antes de moverse
            moved_ok = self.goto_rel(dx_cm=dx, dy_cm=dy, dz_cm=dz, yaw_deg=yaw,
                                     face_target=face_target, blocking=True)
        except Exception as e:
            print(f"[mission] Error en goto_rel de WP{idx}: {e}")
            break
        if moved_ok is False:
            print(f"[mission] Movimiento fallido en WP{idx}; abortando acciones.")
            break

        # ═══════════════════════════════════════════════════════════════
        # EJECUTAR ACCIONES DEL WAYPOINT
        # ═══════════════════════════════════════════════════════════════
        original_wp = waypoints[idx - 1]  # waypoints pasado (idx empieza en 1)

        # Acción: FOTO
        if original_wp.get('photo', False):
            print(f"[mission] WP{idx}: Tomando foto 📷")
            if on_action:
                try:
                    on_action(idx - 1, 'photo')
                    time.sleep(0.5)  # Pequeña pausa para que la foto se complete
                except Exception as e:
                    print(f"[mission] Error en callback de foto: {e}")

        # Acción: VIDEO (duración fija en este waypoint)
        if original_wp.get('video', False):
            video_duration = float(original_wp.get('video_duration', 5) or 5)
            print(f"[mission] WP{idx}: Grabando video por {video_duration}s 🎥")
            # Iniciar grabación antes de medir la duración real del clip
            if on_action:
                try:
                    on_action(idx - 1, 'video')  # Iniciar grabación
                except Exception:
                    pass
            aborted = False
            wait_until = time.time() + max(0.0, video_duration)
            while time.time() < wait_until:
                if getattr(self, "_mission_abort", False):
                    print("[mission] Video interrumpido por aborto.")
                    aborted = True
                    break
                time.sleep(0.1)
            # Detener grabación (siempre, incluso si se abortó)
            if on_action:
                try:
                    on_action(idx - 1, 'video_stop')  # Detener grabación
                except Exception:
                    pass
            if aborted:
                break

        # Acción: VIDEO_START (iniciar grabación continua)
        if original_wp.get('video_start', False):
            print(f"[mission] WP{idx}: Iniciando grabación continua ▶")
            if on_action:
                try:
                    on_action(idx - 1, 'video_start')
                except Exception:
                    pass
            # No esperar - continuar inmediatamente al siguiente waypoint

        # Acción: VIDEO_STOP (detener grabación continua)
        if original_wp.get('video_stop', False):
            print(f"[mission] WP{idx}: Deteniendo grabación ⏹")
            if on_action:
                try:
                    on_action(idx - 1, 'video_stop')
                except Exception:
                    pass

        # Acción: ROTATE (rotación adicional)
        if original_wp.get('rotate', False):
            rotate_deg = float(original_wp.get('rotate_deg', 90) or 90)
            print(f"[mission] WP{idx}: Rotando {rotate_deg}° 🔄")
            if on_action:
                try:
                    on_action(idx - 1, 'rotate')
                except Exception:
                    pass
            try:
                if hasattr(self, 'set_heading'):
                    current_heading = getattr(self, 'heading_deg', 0)
                    new_heading = current_heading + rotate_deg
                    self.set_heading(new_heading, blocking=True)
            except Exception as e:
                print(f"[mission] Error rotando: {e}")

        # Acción: WAIT (hover en el punto)
        if original_wp.get('wait', False):
            wait_sec = float(original_wp.get('wait_sec', 2) or 2)
            print(f"[mission] WP{idx}: Esperando {wait_sec}s ⏱")
            if on_action:
                try:
                    on_action(idx - 1, 'wait')
                except Exception:
                    pass
            t0 = time.time()
            while time.time() - t0 < wait_sec:
                if getattr(self, "_mission_abort", False):
                    print("[mission] Wait interrumpido por aborto.")
                    break
                time.sleep(0.1)
            if getattr(self, "_mission_abort", False):
                break

        # Acción: LAYER_UP (subir una capa)
        if original_wp.get('layer_up', False) and layers:
            current_z = float(self.pose.z_cm) if hasattr(self, 'pose') and self.pose else 50.0
            current_layer = _get_layer_for_z(current_z, layers)
            if current_layer < len(layers) - 1:
                # Subir a la siguiente capa (al centro de la capa)
                next_layer = layers[current_layer + 1]
                target_z = (next_layer["z_min"] + next_layer["z_max"]) / 2
                dz = target_z - current_z
                print(f"[mission] WP{idx}: Subiendo de capa {current_layer + 1} → {current_layer + 2} (z={current_z:.0f} → {target_z:.0f}) ⬆")
                if on_action:
                    try:
                        on_action(idx - 1, 'layer_up')
                    except Exception:
                        pass
                try:
                    self.goto_rel(dx_cm=0, dy_cm=0, dz_cm=dz, blocking=True)
                except Exception as e:
                    print(f"[mission] Error subiendo de capa: {e}")
            else:
                print(f"[mission] WP{idx}: Ya está en la capa más alta ({current_layer + 1})")

        # Acción: LAYER_DOWN (bajar una capa)
        if original_wp.get('layer_down', False) and layers:
            current_z = float(self.pose.z_cm) if hasattr(self, 'pose') and self.pose else 50.0
            current_layer = _get_layer_for_z(current_z, layers)
            if current_layer > 0:
                # Bajar a la capa anterior (al centro de la capa)
                prev_layer = layers[current_layer - 1]
                target_z = (prev_layer["z_min"] + prev_layer["z_max"]) / 2
                dz = target_z - current_z
                print(f"[mission] WP{idx}: Bajando de capa {current_layer + 1} → {current_layer} (z={current_z:.0f} → {target_z:.0f}) ⬇")
                if on_action:
                    try:
                        on_action(idx - 1, 'layer_down')
                    except Exception:
                        pass
                try:
                    self.goto_rel(dx_cm=0, dy_cm=0, dz_cm=dz, blocking=True)
                except Exception as e:
                    print(f"[mission] Error bajando de capa: {e}")
            else:
                print(f"[mission] WP{idx}: Ya está en la capa más baja (1)")

        # Delay en el punto (con posible aborto) - delay original del waypoint
        if delay > 0:
            t0 = time.time()
            while time.time() - t0 < delay:
                if getattr(self, "_mission_abort", False):
                    print("[mission] Abortada durante delay.")
                    break
                time.sleep(0.05)
            if getattr(self, "_mission_abort", False):
                break

    # Final de misión
    if do_land:
        print("[mission] Final de misión → Land")
        try:
            self.Land(blocking=True)
        except Exception:
            pass

    if on_finish:
        try:
            on_finish()
        except Exception:
            pass


def run_mission(self,
                waypoints: List[Dict[str, Any]],
                do_land: bool = True,
                face_target: bool = False,
                layers: Optional[List[Dict[str, Any]]] = None,
                blocking: bool = True,
                on_wp: Optional[Callable[[int, Dict[str, Any]], None]] = None,
                on_action: Optional[Callable[[int, str], None]] = None,
                on_finish: Optional[Callable[[], None]] = None) -> None:

    th = threading.Thread(target=_mission_worker,
                          args=(self, waypoints, do_land, face_target, layers, on_wp, on_action, on_finish),
                          daemon=True)
    th.start()
    if blocking:
        th.join()

def abort_mission(self) -> None:
    setattr(self, "_mission_abort", True)
    setattr(self, "_goto_abort", True)  # por si hay un goto_rel en progreso