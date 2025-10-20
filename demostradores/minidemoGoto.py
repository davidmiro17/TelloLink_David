# Silencia todos los popups de messagebox (info/warning/error)
def _silence_all_popups():
    try:
        import tkinter.messagebox as _mb
        def _noop(*args, **kwargs):  # no-op que no bloquea
            return None
        _mb.showinfo = _noop
        _mb.showwarning = _noop
        _mb.showerror = _noop
    except Exception:
        pass

_silence_all_popups()  # activar antes de cargar nada más


import tkinter as tk
from tkinter import messagebox
import threading
import time
import sys
import math

from TelloLink.Tello import TelloDron

BAT_MIN_SAFE   = 15    # % mínimo recomendado para volar en pruebas
DEFAULT_STEP   = 20    # cm (mover a mano)
DEFAULT_ANGLE  = 30    # grados (giro manual)
DEFAULT_SPEED  = 20    # cm/s
GOTO_CHUNK_CM  = 50    # tamaño de subpaso en GOTO (20..100 recomendable)
COOLDOWN_S     = 0.35  # pequeña pausa tras cada comando


class MiniGotoApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Demo Tello GOTO")
        self.root.geometry("560x520")
        self.root.resizable(True, True)

        self.dron = TelloDron()

        # Variables interfaz gráfica (heredamos idea del “mando”)
        self.step_var  = tk.IntVar(value=DEFAULT_STEP)
        self.speed_var = tk.IntVar(value=DEFAULT_SPEED)
        self.angle_var = tk.IntVar(value=DEFAULT_ANGLE)

        self.state_var = tk.StringVar(value="disconnected")
        self.bat_var   = tk.StringVar(value="—")
        self.h_var     = tk.StringVar(value="0 cm")
        self.wifi_var  = tk.StringVar(value="—")

        # Campos GOTO (deltas relativos)
        self.dx_var   = tk.StringVar(value="0")   # ΔX cm (forward +)
        self.dy_var   = tk.StringVar(value="0")   # ΔY cm (right +)
        self.dz_var   = tk.StringVar(value="0")   # ΔZ cm (up +)
        self.dyaw_var = tk.StringVar(value="0")   # ΔYaw ° (cw +)

        self.objetivo_var = tk.StringVar(value="—")
        self.progreso_var = tk.StringVar(value="—")
        self.aviso_var    = tk.StringVar(value="—")

        # Flags de ejecución
        self._telemetry_running = False
        self._keepalive_running = False
        self._keepalive_paused  = False
        self._goto_thread       = None
        self._goto_abort        = False

        self._build_ui()
        self._bind_keys()

    # --------- UI ----------
    def _build_ui(self):
        pad = dict(padx=6, pady=6)

        # Telemetría
        top = tk.Frame(self.root)
        top.pack(fill="x", **pad)

        tk.Label(top, text="Estado:").grid(row=0, column=0, sticky="w")
        tk.Label(top, textvariable=self.state_var, width=12).grid(row=0, column=1, sticky="w")

        tk.Label(top, text="Batería:").grid(row=0, column=2, sticky="e")
        tk.Label(top, textvariable=self.bat_var, width=6).grid(row=0, column=3, sticky="w")

        tk.Label(top, text="Altura:").grid(row=0, column=4, sticky="e")
        tk.Label(top, textvariable=self.h_var, width=7).grid(row=0, column=5, sticky="w")

        tk.Label(top, text="WiFi:").grid(row=0, column=6, sticky="e")
        tk.Label(top, textvariable=self.wifi_var, width=6).grid(row=0, column=7, sticky="w")

        # Conexión
        conn = tk.Frame(self.root, bd=1, relief="groove")
        conn.pack(fill="x", **pad)
        tk.Button(conn, text="Conectar",    width=12, command=self.on_connect,    bg="#ffb347").grid(row=0, column=0, **pad)
        tk.Button(conn, text="Desconectar", width=12, command=self.on_disconnect).grid(row=0, column=1, **pad)
        tk.Button(conn, text="Salir",       width=12, command=self.on_exit,       bg="#ff6961").grid(row=0, column=2, **pad)

        # Parámetros básicos (velocidad manual, por si ajustas antes del GOTO)
        params = tk.Frame(self.root, bd=1, relief="groove")
        params.pack(fill="x", **pad)

        tk.Label(params, text="Velocidad (cm/s):").grid(row=0, column=0, sticky="e")
        tk.Entry(params, textvariable=self.speed_var, width=6).grid(row=0, column=1, sticky="w")
        tk.Button(params, text="Aplicar", command=self.apply_speed).grid(row=0, column=2, padx=10)

        # Vuelo básico
        flight = tk.Frame(self.root, bd=1, relief="groove")
        flight.pack(fill="x", **pad)
        tk.Button(flight, text="Despegar (Enter)", width=16, command=self.on_takeoff, bg="#90ee90").grid(row=0, column=0, **pad)
        tk.Button(flight, text="Aterrizar (Espacio)", width=16, command=self.on_land, bg="#ff6961").grid(row=0, column=1, **pad)

        # Panel GOTO
        goto = tk.Frame(self.root, bd=1, relief="groove")
        goto.pack(fill="x", **pad)

        tk.Label(goto, text="ΔX (cm) +forward").grid(row=0, column=0, sticky="e")
        tk.Entry(goto, textvariable=self.dx_var, width=7).grid(row=0, column=1, sticky="w")

        tk.Label(goto, text="ΔY (cm) +right").grid(row=0, column=2, sticky="e")
        tk.Entry(goto, textvariable=self.dy_var, width=7).grid(row=0, column=3, sticky="w")

        tk.Label(goto, text="ΔZ (cm) +up").grid(row=1, column=0, sticky="e")
        tk.Entry(goto, textvariable=self.dz_var, width=7).grid(row=1, column=1, sticky="w")

        tk.Label(goto, text="ΔYaw (°) +cw").grid(row=1, column=2, sticky="e")
        tk.Entry(goto, textvariable=self.dyaw_var, width=7).grid(row=1, column=3, sticky="w")

        tk.Button(goto, text="Ir",      width=12, command=self.on_goto,   bg="#87CEEB").grid(row=0, column=4, rowspan=1, **pad)
        tk.Button(goto, text="Abortar", width=12, command=self.on_abort,  bg="#f0ad4e").grid(row=1, column=4, rowspan=1, **pad)

        # Estado GOTO
        st = tk.Frame(self.root, bd=1, relief="groove")
        st.pack(fill="x", **pad)
        tk.Label(st, text="Objetivo:").grid(row=0, column=0, sticky="e")
        tk.Label(st, textvariable=self.objetivo_var, anchor="w").grid(row=0, column=1, sticky="w")
        tk.Label(st, text="Progreso:").grid(row=1, column=0, sticky="e")
        tk.Label(st, textvariable=self.progreso_var, anchor="w").grid(row=1, column=1, sticky="w")
        tk.Label(st, text="Avisos:").grid(row=2, column=0, sticky="e")
        tk.Label(st, textvariable=self.aviso_var, anchor="w", fg="#555").grid(row=2, column=1, sticky="w")

        # Nota
        note = tk.Label(
            self.root,
            fg="#555",
            text=("Consejos: define deltas moderados y prueba en un espacio amplio.\n"
                  "Abortar detiene la secuencia en el siguiente subpaso.")
        )
        note.pack(pady=4)

    def _bind_keys(self):
        self.root.bind("<space>",  lambda e: self.on_land())
        self.root.bind("<Return>", lambda e: self.on_takeoff())
        self.root.bind("<Escape>", lambda e: self.on_abort())

    # --------- Acciones básicas ----------
    def on_connect(self):
        if self.dron.state == "connected":
            return
        try:
            self.dron.connect()
            self.state_var.set(self.dron.state)
            # Telemetría
            self.dron.startTelemetry(freq_hz=5)
            self._telemetry_running = True
            self._schedule_telemetry_pull()
            # Keepalive
            self._start_keepalive()
        except Exception as e:
            messagebox.showerror("Conectar", f"No se pudo conectar: {e}")

    def on_disconnect(self):
        self._goto_abort = True
        self._stop_keepalive()
        try:
            self._telemetry_running = False
            try:
                self.dron.stopTelemetry()
            except Exception:
                pass
            self.dron.disconnect()
        except Exception as e:
            messagebox.showwarning("Desconectar", f"Aviso: {e}")
        finally:
            self.state_var.set("disconnected")
            self.bat_var.set("—")
            self.h_var.set("0 cm")
            self.wifi_var.set("—")
            self.objetivo_var.set("—")
            self.progreso_var.set("—")
            self.aviso_var.set("—")

    def on_takeoff(self):
        if self.dron.state not in ("connected", "flying"):
            messagebox.showwarning("TakeOff", "Conecta primero.")
            return
        bat = getattr(self.dron, "battery_pct", None)
        if isinstance(bat, int) and bat < BAT_MIN_SAFE:
            if not messagebox.askokcancel("Batería baja", f"Batería {bat}%. ¿Despegar igualmente?"):
                return
        try:
            self._pause_keepalive()
            ok = self.dron.takeOff(0.5, blocking=True)
            if not ok:
                messagebox.showerror("TakeOff", "No se pudo despegar.")
        except Exception as e:
            messagebox.showerror("TakeOff", str(e))
        finally:
            self.root.after(200, self._resume_keepalive)

    def on_land(self):
        try:
            self._pause_keepalive()
            self.dron.Land(blocking=False)
        except Exception as e:
            messagebox.showerror("Land", str(e))
        finally:
            self.root.after(200, self._resume_keepalive)

    def apply_speed(self):
        try:
            sp = int(self.speed_var.get())
            sp = max(10, min(100, sp))
            self.dron.set_speed(sp)
            self.speed_var.set(sp)
        except Exception as e:
            messagebox.showerror("Velocidad", f"No se pudo aplicar velocidad: {e}")

    # --------- Telemetría básica ----------
    def _schedule_telemetry_pull(self):
        if not self._telemetry_running:
            return
        try:
            self.state_var.set(getattr(self.dron, "state", "unknown"))
            bat = getattr(self.dron, "battery_pct", None)
            self.bat_var.set(f"{bat}%" if bat is not None else "—")
            h = getattr(self.dron, "height_cm", 0)
            self.h_var.set(f"{h} cm")
            wifi = getattr(self.dron, "wifi", None)
            self.wifi_var.set(str(wifi) if wifi is not None else "—")
        except Exception:
            pass
        finally:
            self.root.after(200, self._schedule_telemetry_pull)

    # --------- GOTO ----------
    def on_goto(self):
        # Prechecks
        if self.dron.state != "flying":
            self.aviso_var.set("Despega primero.")
            return
        try:
            dx = int(float(self.dx_var.get()))
            dy = int(float(self.dy_var.get()))
            dz = int(float(self.dz_var.get()))
            dyaw = int(float(self.dyaw_var.get()))
        except Exception:
            self.aviso_var.set("Valores inválidos. Usa enteros/decimales.")
            return

        self.objetivo_var.set(f"ΔX={dx} cm, ΔY={dy} cm, ΔZ={dz} cm, ΔYaw={dyaw}°")
        self.progreso_var.set("Planificando…")
        self.aviso_var.set("—")

        # Evita dos GOTO a la vez
        if self._goto_thread and self._goto_thread.is_alive():
            self.aviso_var.set("Ya hay un GOTO en marcha.")
            return

        self._goto_abort = False
        self._goto_thread = threading.Thread(
            target=self._run_goto, args=(dx, dy, dz, dyaw), daemon=True
        )
        self._goto_thread.start()

    def on_abort(self):
        self._goto_abort = True
        self.aviso_var.set("Abortando…")

    def _run_goto(self, dx_cm: int, dy_cm: int, dz_cm: int, dyaw_deg: int):
        """
        Secuencia simple: yaw -> X -> Y -> Z. Troceado en subpasos.
        """
        try:
            # 1) Girar si hace falta
            if self._goto_abort: return
            if dyaw_deg != 0:
                if not self._do_turn(dyaw_deg):
                    self._ui_set(self.aviso_var, "Giro fallido; abortado.")
                    return

            # 2) Mover X (forward/back) relativo al yaw actual
            if self._goto_abort: return
            if dx_cm != 0:
                if not self._do_axis_move("X", dx_cm):
                    self._ui_set(self.aviso_var, "Movimiento X fallido; abortado.")
                    return

            # 3) Mover Y (right/left)
            if self._goto_abort: return
            if dy_cm != 0:
                if not self._do_axis_move("Y", dy_cm):
                    self._ui_set(self.aviso_var, "Movimiento Y fallido; abortado.")
                    return

            # 4) Mover Z (up/down) con techo
            if self._goto_abort: return
            if dz_cm != 0:
                if not self._do_axis_move("Z", dz_cm):
                    self._ui_set(self.aviso_var, "Movimiento Z fallido; abortado.")
                    return

            self._ui_set(self.progreso_var, "Completado.")
            self._ui_set(self.aviso_var, "—")
        finally:
            pass

    # --- helpers GOTO ---
    def _do_turn(self, dyaw_deg: int) -> bool:
        """
        Trocea el giro, pausa keepalive, actualiza pose yaw si existe.
        """
        remaining = abs(int(dyaw_deg))
        cw = dyaw_deg > 0
        while remaining > 0 and not self._goto_abort:
            step = min(remaining, 360)
            try:
                self._pause_keepalive()
                if cw:
                    self.dron.cw(step)
                    # actualizar pose (si existe)
                    try:
                        if hasattr(self.dron, "pose"):
                            self.dron.pose.update_yaw(step)
                    except Exception:
                        pass
                else:
                    self.dron.ccw(step)
                    try:
                        if hasattr(self.dron, "pose"):
                            self.dron.pose.update_yaw(-step)
                    except Exception:
                        pass
                remaining -= step
                self._ui_set(self.progreso_var, f"Giro restante: {remaining}°")
            except Exception as e:
                self._ui_set(self.aviso_var, f"cw/ccw error: {e}")
                return False
            finally:
                self.root.after(150, self._resume_keepalive)
                time.sleep(COOLDOWN_S)
        return not self._goto_abort

    def _do_axis_move(self, axis: str, delta: int) -> bool:
        """
        Ejes:
          X: +forward / -back
          Y: +right   / -left
          Z: +up      / -down (respeta techo via tus métodos)
        """
        remaining = int(delta)
        sign = 1 if remaining >= 0 else -1

        def send_move(verb, dist):
            # Pausa keepalive y manda
            self._pause_keepalive()
            try:
                getattr(self.dron, verb)(dist)
                # actualización de pose si existe
                try:
                    if hasattr(self.dron, "pose"):
                        self.dron.pose.update_move(verb, dist)
                except Exception:
                    pass
            finally:
                self.root.after(150, self._resume_keepalive)
            time.sleep(COOLDOWN_S)

        while abs(remaining) > 0 and not self._goto_abort:
            step = min(abs(remaining), GOTO_CHUNK_CM)
            if axis == "X":
                verb = "forward" if sign > 0 else "back"
            elif axis == "Y":
                verb = "right" if sign > 0 else "left"
            else:  # "Z"
                verb = "up" if sign > 0 else "down"

            try:
                # Para Z, tu función up() ya recorta por TECHO_M; para down ya recorta hasta suelo.
                send_move(verb, step)
                remaining -= sign * step
                self._ui_set(self.progreso_var, f"{axis} restante: {remaining} cm")
            except Exception as e:
                self._ui_set(self.aviso_var, f"{verb} error: {e}")
                # reintento 1 vez el mismo subpaso
                try:
                    send_move(verb, step)
                    remaining -= sign * step
                    self._ui_set(self.progreso_var, f"{axis} restante: {remaining} cm (reintento ok)")
                except Exception as e2:
                    self._ui_set(self.aviso_var, f"{verb} fallo tras reintento: {e2}")
                    return False

        return not self._goto_abort

    # --------- Keepalive ---------
    def _start_keepalive(self):
        if self._keepalive_running:
            return
        self._keepalive_running = True
        self._keepalive_paused  = False

        def _loop():
            while self._keepalive_running:
                try:
                    if self._keepalive_paused:
                        time.sleep(0.05)
                        continue
                    if getattr(self.dron, "state", "") == "flying":
                        self.dron._send("battery?")
                except Exception:
                    pass
                time.sleep(0.5)
        threading.Thread(target=_loop, daemon=True).start()

    def _stop_keepalive(self):
        self._keepalive_running = False

    def _pause_keepalive(self):
        self._keepalive_paused = True

    def _resume_keepalive(self):
        self._keepalive_paused = False

    # --------- Utilidades ----------
    def _ui_set(self, tk_var, text):
        try:
            self.root.after(0, lambda: tk_var.set(text))
        except Exception:
            pass

    def on_exit(self):
        try:
            self._goto_abort = True
            self._telemetry_running = False
            try:
                self.dron.stopTelemetry()
            except Exception:
                pass
            try:
                self._stop_keepalive()
            except Exception:
                pass
            if getattr(self.dron, "state", "") == "connected":
                try:
                    self.dron.disconnect()
                except Exception:
                    pass
        except Exception as e:
            messagebox.showwarning("Salir", f"Aviso: {e}")
        finally:
            self.root.destroy()


def main():
    root = tk.Tk()
    if sys.platform == "darwin":
        root.configure(bg="#f0f0f0")
        root.tk_setPalette(
            background="#f0f0f0",
            foreground="black",
            activeBackground="#ffb347",
            activeForeground="black"
        )
    app = MiniGotoApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()