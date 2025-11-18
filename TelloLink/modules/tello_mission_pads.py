# Dirección de detección
DIRECTION_DOWN = 0  # Solo cámara inferior (20Hz)
DIRECTION_FORWARD = 1  # Solo cámara frontal (20Hz)
DIRECTION_BOTH = 2  # Ambas cámaras (10Hz)


def enable_mission_pads(self, direction=DIRECTION_DOWN):

    self._require_connected()

    try:
        # Activar detección
        resp = self._send("mon")
        if str(resp).strip().lower() != "ok":
            print(f"[mission_pads] Error activando: {resp}")
            return False

        # Configurar dirección de detección
        resp_dir = self._send(f"mdirection {direction}")
        if str(resp_dir).strip().lower() != "ok":
            print(f"[mission_pads] Error configurando dirección: {resp_dir}")
            return False

        self._mission_pads_enabled = True
        self._mission_pads_direction = direction

        dir_names = {0: "ABAJO", 1: "ADELANTE", 2: "AMBAS"}
        print(f"[mission_pads] ✓ Activado (detección: {dir_names.get(direction, direction)})")
        return True

    except Exception as e:
        print(f"[mission_pads] Error: {e}")
        return False


def disable_mission_pads(self):

    self._require_connected()

    try:
        resp = self._send("moff")
        self._mission_pads_enabled = False
        print("[mission_pads]     Desactivado")
        return str(resp).strip().lower() == "ok"
    except Exception as e:
        print(f"[mission_pads] Error desactivando: {e}")
        return False


def get_mission_pad_id(self):

    if not getattr(self, "_mission_pads_enabled", False):
        return -1

    try:
        mid = self._tello.get_mission_pad_id()
        return int(mid) if mid is not None else -1
    except Exception:
        return -1


def get_mission_pad_distance_x(self):

    if not getattr(self, "_mission_pads_enabled", False):
        return -1

    try:
        x = self._tello.get_mission_pad_distance_x()
        return int(x) if x is not None else -1
    except Exception:
        return -1


def get_mission_pad_distance_y(self):

    if not getattr(self, "_mission_pads_enabled", False):
        return -1

    try:
        y = self._tello.get_mission_pad_distance_y()
        return int(y) if y is not None else -1
    except Exception:
        return -1


def get_mission_pad_distance_z(self):

    if not getattr(self, "_mission_pads_enabled", False):
        return -1

    try:
        z = self._tello.get_mission_pad_distance_z()
        return int(z) if z is not None else -1
    except Exception:
        return -1


def is_mission_pad_detected(self):

    return get_mission_pad_id(self) != -1


def get_mission_pad_position(self):

    mid = get_mission_pad_id(self)
    if mid == -1:
        return None

    return {
        "id": mid,
        "x": get_mission_pad_distance_x(self),
        "y": get_mission_pad_distance_y(self),
        "z": get_mission_pad_distance_z(self)
    }