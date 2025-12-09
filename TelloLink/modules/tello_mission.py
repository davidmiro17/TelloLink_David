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


def _validate_and_normalize(waypoints: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """
    Normaliza y valida la lista de waypoints.
    Soporta acciones: photo, video, rotate, wait
    """
    norm: List[Dict[str, Any]] = []
    for i, wp in enumerate(waypoints, start=1):
        if not isinstance(wp, dict):
            raise ValueError(f"WP{i}: cada waypoint debe ser dict, recibido: {type(wp)}")

        yaw = wp.get("yaw", None)
        delay = float(wp.get("delay", 0.0) or 0.0)
        if delay < 0:
            raise ValueError(f"WP{i}: delay no puede ser negativo")

        # Extraer acciones del waypoint
        actions = {
            'photo': wp.get('photo', False),
            'video': wp.get('video', False),
            'video_duration': float(wp.get('video_duration', 0) or 0),
            'rotate': wp.get('rotate', False),
            'rotate_deg': float(wp.get('rotate_deg', 0) or 0),
            'wait': wp.get('wait', False),
            'wait_sec': float(wp.get('wait_sec', 0) or 0),
        }

        if _is_abs_wp(wp):
            x = wp.get("x", None); y = wp.get("y", None); z = wp.get("z", None)
            x = None if x is None else float(x)
            y = None if y is None else float(y)
            z = None if z is None else float(z)
            yaw_f = None if yaw is None else float(yaw)
            norm.append({
                "mode": "abs", "x": x, "y": y, "z": z,
                "yaw": yaw_f, "delay": delay, "actions": actions
            })
        else:
            dx = float(wp.get("dx", 0.0) or 0.0)
            dy = float(wp.get("dy", 0.0) or 0.0)
            dz = float(wp.get("dz", 0.0) or 0.0)
            yaw_f = None if yaw is None else float(yaw)
            norm.append({
                "mode": "rel", "dx": dx, "dy": dy, "dz": dz,
                "yaw": yaw_f, "delay": delay, "actions": actions
            })
    return norm


def _execute_waypoint_actions(self, actions: Dict[str, Any],
                               on_action: Optional[Callable[[str], None]] = None) -> None:
    """
    Ejecuta las acciones configuradas para un waypoint.
    Acciones soportadas: rotate, photo, video, wait
    """
    # 1. Rotar primero (antes de foto/video)
    if actions.get('rotate') and actions.get('rotate_deg', 0) != 0:
        deg = actions['rotate_deg']
        if on_action:
            on_action('rotate')
        try:
            if deg > 0:
                self.cw(deg)
            else:
                self.ccw(abs(deg))
            time.sleep(0.5)
        except Exception as e:
            print(f"[mission] Error rotando: {e}")

    # 2. Tomar foto
    if actions.get('photo'):
        if on_action:
            on_action('photo')
        try:
            if hasattr(self, 'snapshot'):
                self.snapshot()
            time.sleep(0.5)
        except Exception as e:
            print(f"[mission] Error tomando foto: {e}")

    # 3. Grabar video
    if actions.get('video') and actions.get('video_duration', 0) > 0:
        duration = actions['video_duration']
        if on_action:
            on_action('video')
        try:
            # El dron no tiene grabación nativa, esto es manejado por la app
            # Solo hacemos el delay correspondiente
            time.sleep(duration)
        except Exception as e:
            print(f"[mission] Error en video: {e}")

    # 4. Esperar
    if actions.get('wait') and actions.get('wait_sec', 0) > 0:
        wait_time = actions['wait_sec']
        if on_action:
            on_action('wait')
        t0 = time.time()
        while time.time() - t0 < wait_time:
            if getattr(self, "_mission_abort", False):
                break
            time.sleep(0.05)


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
    return dx, dy, dz


def _mission_worker(self,
                    waypoints: List[Dict[str, Any]],
                    do_land: bool = True,
                    return_home: bool = False,
                    on_wp_arrived: Optional[Callable[[int, Dict[str, Any]], None]] = None,
                    on_action: Optional[Callable[[int, str], None]] = None,
                    on_finish: Optional[Callable[[], None]] = None) -> None:
    """
    Worker interno que ejecuta la misión en un hilo.

    Args:
        waypoints: Lista de waypoints con coordenadas y acciones
        do_land: Si True, aterriza al final de la misión
        return_home: Si True, vuelve a (0,0) antes de aterrizar
        on_wp_arrived: Callback al LLEGAR a cada waypoint (idx 0-based, wp_data)
        on_action: Callback al ejecutar cada acción (idx 0-based, action_name)
        on_finish: Callback al terminar la misión
    """
    setattr(self, "_mission_abort", False)

    # Validación inicial
    try:
        wps = _validate_and_normalize(waypoints)
    except Exception as e:
        print(f"[mission] Waypoints inválidos: {e}")
        return

    # Chequeos de seguridad
    if getattr(self, "state", "") == "disconnected":
        print("[mission] Dron desconectado; abortando.")
        return
    bat = getattr(self, "battery_pct", None)
    if isinstance(bat, int) and bat < _MIN_BAT_PCT:
        print(f"[mission] Batería baja ({bat}%), abortando.")
        return

    # Despegue si hace falta
    if getattr(self, "state", "") != "flying":
        print("[mission] Dron en tierra: despegando a 0.5 m")
        if not self.takeOff(0.5, blocking=True):
            print("[mission] No se pudo despegar; abortando.")
            return
        time.sleep(0.4)

    total_wps = len(wps)

    # Recorrer waypoints
    for idx, wp in enumerate(wps):
        if getattr(self, "_mission_abort", False):
            print("[mission] Abortada por solicitud externa.")
            break

        # Comprobar batería
        bat = getattr(self, "battery_pct", None)
        if isinstance(bat, int) and bat < _MIN_BAT_PCT:
            print(f"[mission] Abortada por batería ({bat}%).")
            break

        # Calcular movimiento
        if wp["mode"] == "rel":
            dx, dy, dz = wp["dx"], wp["dy"], wp["dz"]
            target_desc = f"REL: dx={dx:.1f}, dy={dy:.1f}, dz={dz:.1f}"
        else:
            try:
                dx, dy, dz = _rel_from_abs(self, wp)
            except Exception as e:
                print(f"[mission] WP{idx+1} absoluto inválido: {e}")
                break
            target_desc = f"ABS: x={wp['x']}, y={wp['y']}, z={wp['z']}"

        yaw = wp.get("yaw", None)
        delay = float(wp.get("delay", 0.0) or 0.0)

        print(f"[mission] WP{idx+1}/{total_wps} → {target_desc}")

        # Ejecutar movimiento
        try:
            self.goto_rel(dx_cm=dx, dy_cm=dy, dz_cm=dz, yaw_deg=yaw, blocking=True)
        except Exception as e:
            print(f"[mission] Error en goto_rel de WP{idx+1}: {e}")
            break

        # Callback al llegar (DESPUÉS de llegar, no antes)
        if on_wp_arrived:
            try:
                on_wp_arrived(idx, dict(wp))
            except Exception:
                pass

        # Ejecutar acciones del waypoint
        actions = wp.get("actions", {})
        if actions:
            def action_callback(action_name):
                if on_action:
                    try:
                        on_action(idx, action_name)
                    except Exception:
                        pass
            _execute_waypoint_actions(self, actions, action_callback)

        # Delay adicional
        if delay > 0:
            t0 = time.time()
            while time.time() - t0 < delay:
                if getattr(self, "_mission_abort", False):
                    print("[mission] Abortada durante delay.")
                    break
                time.sleep(0.05)
            if getattr(self, "_mission_abort", False):
                break

    # Return to home si está activado
    if return_home and not getattr(self, "_mission_abort", False):
        print("[mission] Volviendo a casa (0, 0)...")
        try:
            # Calcular movimiento a origen
            if hasattr(self, "pose") and self.pose:
                dx = -float(self.pose.x_cm)
                dy = -float(self.pose.y_cm)
                self.goto_rel(dx_cm=dx, dy_cm=dy, dz_cm=0, blocking=True)
        except Exception as e:
            print(f"[mission] Error volviendo a casa: {e}")

    # Final de misión
    if do_land and not getattr(self, "_mission_abort", False):
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
                return_home: bool = False,
                blocking: bool = True,
                on_wp_arrived: Optional[Callable[[int, Dict[str, Any]], None]] = None,
                on_action: Optional[Callable[[int, str], None]] = None,
                on_finish: Optional[Callable[[], None]] = None) -> None:
    """
    Ejecuta una misión de vuelo con waypoints y acciones.

    Args:
        waypoints: Lista de waypoints. Cada waypoint puede incluir:
            - x, y, z: Coordenadas absolutas (cm)
            - dx, dy, dz: Coordenadas relativas (cm)
            - photo: True para tomar foto al llegar
            - video: True para grabar video
            - video_duration: Duración del video (seg)
            - rotate: True para rotar
            - rotate_deg: Grados de rotación (+cw, -ccw)
            - wait: True para esperar
            - wait_sec: Segundos de espera
            - delay: Delay adicional (seg)
        do_land: Si True, aterriza al final
        return_home: Si True, vuelve a (0,0) antes de aterrizar
        blocking: Si True, espera a que termine la misión
        on_wp_arrived: Callback al llegar a cada waypoint (idx 0-based, wp_data)
        on_action: Callback al ejecutar acción (idx 0-based, action_name)
        on_finish: Callback al terminar
    """
    th = threading.Thread(target=_mission_worker,
                          args=(self, waypoints, do_land, return_home,
                                on_wp_arrived, on_action, on_finish),
                          daemon=True)
    th.start()
    if blocking:
        th.join()

def abort_mission(self) -> None:
    setattr(self, "_mission_abort", True)
    setattr(self, "_goto_abort", True)  # por si hay un goto_rel en progreso