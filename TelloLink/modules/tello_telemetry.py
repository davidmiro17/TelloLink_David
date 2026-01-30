import threading
import time

# Intentamos importar la PoseVirtual
try:
    from TelloLink.modules.tello_pose import PoseVirtual
except Exception:
    PoseVirtual = None


def _telemetry_loop(self, period_s: float):
    if not hasattr(self, "_pose_takeoff_synced"):
        self._pose_takeoff_synced = False

    while not getattr(self, "_telemetry_stop", True):

        # Si no hay conexión, esperamos y reintentamos
        if getattr(self, "_tello", None) is None or getattr(self, "state", "disconnected") == "disconnected":
            time.sleep(period_s)
            continue

        # Valores locales para sincronización de la pose
        height_val = None
        yaw_val = None

        # Altura (cm)
        try:
            h = self._tello.get_height()
            if h is not None:
                self.height_cm = max(0, int(h))
                height_val = self.height_cm
        except Exception:
            pass

        # Yaw (grados)
        try:
            # Primero intentamos get_yaw()
            gy = getattr(self._tello, "get_yaw", None)
            if callable(gy):
                y = gy()
            else:
                # Alternativa: leer del estado  si existe
                y = None
                gc = getattr(self._tello, "get_current_state", None)
                if callable(gc):
                    try:
                        st = gc()

                        if isinstance(st, dict):
                            y = st.get("yaw", None)
                    except Exception:
                        y = None

            if y is not None:
                self.yaw_deg = float(y)
                yaw_val = self.yaw_deg
        except Exception:
            pass

        # Batería (%)
        try:
            b = self._tello.get_battery()
            if b is not None:
                self.battery_pct = max(0, int(b))
        except Exception:
            pass

        # Temperatura (°C)
        try:
            t = self._tello.get_temperature()
            if t is not None:
                self.temp_c = float(t)
        except Exception:
            pass

        # WiFi SNR (Signal-to-Noise Ratio)
        try:
            w = self._tello.query_wifi_signal_noise_ratio()
            if w is not None:
                self.wifi = int(w)
        except Exception:
            pass

        #  Tiempo de vuelo (s)
        try:
            ft = self._tello.get_flight_time()
            if ft is not None:
                self.flight_time_s = max(0, int(ft))
        except Exception:
            pass

        # Velocidades (cm/s)
        vx_local = 0
        vy_local = 0
        try:
            vx = self._tello.get_speed_x()
            vy = self._tello.get_speed_y()
            vz = self._tello.get_speed_z()
            if vx is not None:
                self.vx_cm_s = int(-vx)
                vx_local = -vx
            if vy is not None:
                self.vy_cm_s = int(-vy)
                vy_local = -vy
            if vz is not None:
                self.vz_cm_s = int(vz)
        except Exception:
            pass

        # Integración de velocidad para actualizar pose x/y en tiempo real
        # NOTA: Saltar integración si goto o RC está en progreso (evita double-update)
        goto_in_progress = getattr(self, "_goto_in_progress", False)
        rc_active = getattr(self, "_rc_active", False)
        try:
            now = time.time()
            dt = now - getattr(self, "_last_pose_update_ts", now)
            self._last_pose_update_ts = now

            # Solo integrar si NO hay goto NI RC en progreso, estamos volando y hay velocidad significativa
            if not goto_in_progress and not rc_active and getattr(self, "state", "") == "flying" and hasattr(self, "pose") and self.pose is not None:
                speed = (vx_local**2 + vy_local**2)**0.5
                if speed > 5 and dt > 0 and dt < 1.0:  # Velocidad > 5 cm/s y dt razonable
                    import math
                    # vx_local es forward/back, vy_local es left/right (en frame del dron)
                    # Convertir a coordenadas mundo usando yaw
                    yaw_rad = math.radians(self.pose.yaw_deg)
                    cos_y = math.cos(yaw_rad)
                    sin_y = math.sin(yaw_rad)
                    dx_world = vx_local * cos_y - vy_local * sin_y
                    dy_world = vx_local * sin_y + vy_local * cos_y
                    self.pose.x_cm += dx_world * dt
                    self.pose.y_cm += dy_world * dt
                    # Guardar velocidad en COORDENADAS MUNDO para flecha de movimiento
                    self.pose.vx = dx_world  # velocidad en X mundo (cm/s)
                    self.pose.vy = dy_world  # velocidad en Y mundo (cm/s)
                else:
                    # Sin movimiento significativo
                    if hasattr(self.pose, 'vx'):
                        self.pose.vx = 0
                        self.pose.vy = 0
        except Exception:
            pass


        try:
            # Si aún no existe pose, la creamos
            if not hasattr(self, "pose") or self.pose is None:
                if PoseVirtual is not None:
                    self.pose = PoseVirtual()

            if hasattr(self, "pose") and self.pose is not None:
                # Altura (z)
                self.pose.set_from_telemetry(height_cm=height_val)

                # Yaw absoluto -> relativo
                if yaw_val is not None:
                    try:
                        self.pose.set_heading_from_absolute_yaw(yaw_val)
                    except Exception:
                        pass

                # Al pasar a estado 'flying' por primera vez, fijamos referencia de yaw del vuelo
                try:
                    if getattr(self, "state", "") == "flying":
                        if yaw_val is not None and not getattr(self, "_pose_takeoff_synced", False):
                            self.pose.set_takeoff_reference(yaw_val)
                            self._pose_takeoff_synced = True
                    else:
                        # cuando no estamos volando, reseteamos la marca para el siguiente vuelo
                        self._pose_takeoff_synced = False
                except Exception:
                    pass
        except Exception:
            pass

        self.telemetry_ts = time.time()

        time.sleep(period_s)


def startTelemetry(self, freq_hz: int = 5):
    if freq_hz <= 0:
        freq_hz = 5

    if getattr(self, "_telemetry_thread", None) and self._telemetry_thread.is_alive():
        return False

    # Inicialización de atributos (si no existen)
    self.height_cm = getattr(self, "height_cm", 0)
    self.battery_pct = getattr(self, "battery_pct", None)
    self.temp_c = getattr(self, "temp_c", None)
    self.wifi = getattr(self, "wifi", None)
    self.flight_time_s = getattr(self, "flight_time_s", 0)
    self.vx_cm_s = getattr(self, "vx_cm_s", 0)
    self.vy_cm_s = getattr(self, "vy_cm_s", 0)
    self.vz_cm_s = getattr(self, "vz_cm_s", 0)
    self._last_pose_update_ts = time.time()
    self.telemetry_ts = time.time()

    # Creamos aqui la pose
    if not hasattr(self, "pose") or self.pose is None:
        if PoseVirtual is not None:
            self.pose = PoseVirtual()

    self._telemetry_stop = False
    period_s = 1.0 / float(freq_hz)

    th = threading.Thread(target=_telemetry_loop, args=(self, period_s), daemon=True)
    self._telemetry_thread = th
    th.start()
    return True


def stopTelemetry(self):
    self._telemetry_stop = True
    th = getattr(self, "_telemetry_thread", None)
    if th and th.is_alive():
        th.join(timeout=2.0)
        if th.is_alive():
            print("[telemetry] WARNING: Thread no terminó después de 2s, continuando...")
            # No ponemos a None para evitar crear duplicados si se llama a start otra vez
            return False
    self._telemetry_thread = None
    return True