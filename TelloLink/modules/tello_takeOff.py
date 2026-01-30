
import time

_MIN_BAT_PCT = 10


#Funcio n en la que intentamos leer la altura del backend del Tello, y si falla usamos el valor guardado en self.height_cm
def _read_height_cm_runtime(self) -> int:

    try:
        if hasattr(self, "_tello") and self._tello:
            h = self._tello.get_height()
            if h is not None:
                return max(0, int(h))
    except Exception:
        pass
    try:
        return max(0, int(getattr(self, "height_cm", 0) or 0))
    except Exception:
        return 0


#Funcion para subir los centímetros restantes en el despegue
def _ascend_to_target(self, target_h_cm):

    try:
        self._send(f"up {int(target_h_cm)}")
    except Exception as e:
        print(f"[WARN] Subida adicional falló: {e}")

#Función que contiene toda la lógica del despegue completo
def _takeOff(self, altura_objetivo_m=0.5, blocking=True):

    try:
        if getattr(self, "state", "") == "disconnected":
            print("[ERROR] Dron desconectado, abortando despegue.")
            return False

        bat = getattr(self, "battery_pct", None)
        if isinstance(bat, int) and bat < _MIN_BAT_PCT:
            print(f"[WARN] Batería muy baja ({bat}%), riesgo en despegue.")

        print("Empezamos a despegar")
        resp = self._send("takeoff")
        print(f"[INFO] tello_takeOff -> respuesta inicial: {resp}")

        #Espera a que suba al menos a ~20 cm ---
        t0 = time.time()
        ok_alt = False
        h = 0
        while time.time() - t0 < 5.0:
            h = _read_height_cm_runtime(self)
            if h >= 20:
                ok_alt = True
                break
            time.sleep(0.2)

        # Empujón extra de subida si no llegó, por si las condiciones del suelo son malas para el despegue
        if not ok_alt:
            print("[WARN] Altura <20 cm tras 5s, aplico empujón 'up 20'")
            try:
                self._send("up 20")
            except Exception as e:
                print(f"[WARN] Empujón falló: {e}")
            time.sleep(2.0)
            h = _read_height_cm_runtime(self)
            if h >= 20:
                ok_alt = True

        if not ok_alt:
            print("[ERROR] No se confirmó despegue (altura <20 cm tras reintento).")
            return False

        # Confirmamos que ya está en el aire
        print(f"Altura inicial confirmada: {h} cm (ok).")
        self.state = "flying"

        #Reseteamos la pose
        try:
            pose = getattr(self, "pose", None)
            if pose is not None:
                pose.reset()
                pose.z_cm = float(h)  # Establecemos Z a la altura barométrica actual

                # IMPORTANTE: Leer el yaw REAL del dron, no asumir 0
                real_yaw = None
                try:
                    if hasattr(self, "_tello") and self._tello:
                        gy = getattr(self._tello, "get_yaw", None)
                        if callable(gy):
                            real_yaw = gy()
                            print(f"[takeOff] Yaw real del dron: {real_yaw}°")
                except Exception as e:
                    print(f"[takeOff] No se pudo leer yaw: {e}")

                # Si obtuvimos el yaw real, guardarlo como referencia
                # El sistema usará yaw relativo = 0, pero yaw0_deg guarda el absoluto
                if real_yaw is not None:
                    pose.yaw0_deg = float(real_yaw)  # Yaw absoluto de referencia
                    pose.yaw_deg = 0.0  # Yaw relativo empieza en 0
                    self._pose_takeoff_synced = True  # Evitar que telemetry lo sobrescriba
                    self._commanded_yaw = 0.0  # Resetear yaw comandado para goto
                    print(f"[takeOff] Pose inicializada: yaw_relativo=0°, yaw_referencia={real_yaw}°")
                else:
                    pose.yaw_deg = 0.0
                    pose.yaw0_deg = 0.0
                    self._pose_takeoff_synced = True  # Marcar igualmente
                    self._commanded_yaw = 0.0  # Resetear yaw comandado para goto
                    print("[takeOff] WARNING: No se pudo leer yaw, asumiendo 0°")
        except Exception as e:
            print(f"[takeOff] Error inicializando pose: {e}")

        # Subida adicional si la altura objetivo es mayor que la actual
        target_h_cm = int(altura_objetivo_m * 100)
        if h < target_h_cm:
            _ascend_to_target(self, target_h_cm - h)
            # Actualizar pose.z_cm a la altura objetivo
            try:
                pose = getattr(self, "pose", None)
                if pose is not None:
                    pose.z_cm = float(target_h_cm)
                    print(f"[takeOff] Pose.z actualizada: {target_h_cm} cm")
            except Exception:
                pass
            print(f"Altura objetivo alcanzada (~{target_h_cm} cm).")

        #Pausa al final
        try:
            self._after_takeoff_ts = time.time()
            self._after_takeoff_mono = time.monotonic()
            self._stream_needs_refresh = True
        except Exception:
            pass
        time.sleep(0.7)  # ~0.5–0.8s suele ser suficiente

        print("Despegue completado (≈1 m)")
        return True

    except Exception as e:
        print(f"[ERROR] takeOff -> {e}")
        return False

#Función pública del despegue
def takeOff(self, altura_objetivo_m=0.5, blocking=True, callback=None):

    if getattr(self, "_takeoff_in_progress", False):
        print("[takeOff] Ya hay un despegue en curso; ignoro la petición duplicada.")
        return True

    setattr(self, "_takeoff_in_progress", True)

    if blocking:
        try:
            result = _takeOff(self, altura_objetivo_m, blocking=False)
            if callback and callable(callback):
                callback(result)
            return result
        finally:
            setattr(self, "_takeoff_in_progress", False)
    else:
        import threading
        def _runner():
            try:
                result = _takeOff(self, altura_objetivo_m, False)
                if callback and callable(callback):
                    callback(result)
            finally:
                setattr(self, "_takeoff_in_progress", False)

        threading.Thread(target=_runner, daemon=True).start()
        return True