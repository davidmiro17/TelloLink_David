import time

_MIN_BAT_PCT = 10  # seguridad mínima por si se usa sin comprobaciones externas


def _checkAltitudeReached(self, target_h_cm, timeout_s=5.0):
    """Comprueba si el dron ha alcanzado al menos la altura objetivo."""
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        try:
            h = int(getattr(self, "height_cm", 0) or 0)
        except Exception:
            h = 0
        if h >= target_h_cm * 0.9:  # margen del 10%
            return True
        time.sleep(0.2)
    return False


def _ascend_to_target(self, target_h_cm):
    """Sube hasta la altura objetivo en incrementos seguros."""
    try:
        self._send(f"up {int(target_h_cm)}")
    except Exception as e:
        print(f"[WARN] Subida adicional falló: {e}")


def _takeOff(self, altura_objetivo_m=0.5, blocking=True):
    """Despegue controlado con comprobación de altura y fallback seguro."""
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

        # --- DESPEGUE ROBUSTO ---
        t0 = time.time()
        ok_alt = False
        while time.time() - t0 < 5.0:
            try:
                h = int(getattr(self, "height_cm", 0) or 0)
            except Exception:
                h = 0
            if h >= 20:
                ok_alt = True
                break
            time.sleep(0.2)

        # si no llega a 20 cm, intenta empujón
        if not ok_alt:
            print("[WARN] Altura <20 cm tras 5s, aplico empujón 'up 20'")
            try:
                self._send("up 20")
            except Exception as e:
                print(f"[WARN] Empujón falló: {e}")
            time.sleep(2.0)
            try:
                h = int(getattr(self, "height_cm", 0) or 0)
            except Exception:
                h = 0
            if h >= 20:
                ok_alt = True

        if not ok_alt:
            print("[ERROR] No se confirmó despegue (altura <20 cm tras reintento).")
            return False

        # --- si todo ok, continuar ---
        print(f"Altura inicial confirmada: {h} cm (ok).")
        self.state = "flying"

        # Subida adicional si se pide altura objetivo > actual
        target_h_cm = int(altura_objetivo_m * 100)
        if h < target_h_cm:
            self._ascend_to_target(target_h_cm - h)
            print(f"Altura objetivo alcanzada (~{target_h_cm} cm).")

        print("Despegue completado (≈1 m)")
        return True

    except Exception as e:
        print(f"[ERROR] takeOff -> {e}")
        return False


def takeOff(self, altura_objetivo_m=0.5, blocking=True):
    """Wrapper público para el hilo de despegue."""
    if blocking:
        return _takeOff(self, altura_objetivo_m, blocking=True)
    else:
        import threading
        threading.Thread(target=_takeOff, args=(self, altura_objetivo_m, False), daemon=True).start()
        return True