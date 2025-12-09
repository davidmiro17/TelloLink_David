#Silencia todos los popups de messagebox (info/warning/error)
def _silence_all_popups():
    try:
        import tkinter.messagebox as _mb
        def _noop(*args, **kwargs):
            return None

        _mb.showinfo = _noop
        _mb.showwarning = _noop
        _mb.showerror = _noop
    except Exception:
        pass


_silence_all_popups()

import tkinter as tk
from tkinter import messagebox, filedialog, ttk
import threading
import time
import sys
import math
import json
import os
import datetime

# FPV / Imagen
import cv2
from PIL import Image, ImageTk
import numpy as np

try:
    from shapely.geometry import Point, Polygon
except ImportError:
    print("ADVERTENCIA: shapely no está instalado. Instala con: pip install shapely")
    Point = None
    Polygon = None

from TelloLink.Tello import TelloDron
from TelloLink import JoystickController
from TelloLink.modules.tello_session import SessionManager, migrate_legacy_files
from TelloLink.modules.tello_geometry import (
    point_in_obstacle, validate_mission_paths, plan_mission_with_avoidance
)

BAT_MIN_SAFE = 20
DEFAULT_STEP = 20
DEFAULT_ANGLE = 30
DEFAULT_SPEED = 20

# Constantes del minimapa geofence
MAP_SIZE_PX = 900
MAP_BG_COLOR = "#fafafa"
MAP_AXES_COLOR = "#cccccc"
MAP_DRONE_COLOR = "#1f77b4"
MAP_TARGET_COLOR = "#d62728"
PX_PER_CM = 1
GRID_STEP_CM = 50
CENTER_MARK_COLOR = "#666"


def _ts():
    return datetime.datetime.now().strftime("%Y%m%d_%H%M%S")


class MiniRemoteApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Demo Tello Geofence + FPV + Joystick")
        self.root.geometry("740x650")
        self.root.resizable(True, True)
        self.root.configure(bg="#f0f0f0")

        self.dron = TelloDron()

        # Variables interfaz gráfica
        self.step_var = tk.IntVar(value=DEFAULT_STEP)
        self.speed_var = tk.IntVar(value=DEFAULT_SPEED)
        self.angle_var = tk.IntVar(value=DEFAULT_ANGLE)
        self.state_var = tk.StringVar(value="disconnected")
        self.bat_var = tk.StringVar(value="—")
        self.h_var = tk.StringVar(value="0 cm")
        self.wifi_var = tk.StringVar(value="—")

        self._telemetry_running = False
        self._ui_landing = False

        # Geofence (coordenadas absolutas, no simétricas)
        self.gf_x1_var = tk.StringVar(value="-100")
        self.gf_y1_var = tk.StringVar(value="-100")
        self.gf_x2_var = tk.StringVar(value="100")
        self.gf_y2_var = tk.StringVar(value="100")
        self.gf_zmin_var = tk.StringVar(value="0")
        self.gf_zmax_var = tk.StringVar(value="200")
        self.gf_mode_var = tk.StringVar(value="soft")

        # Mapa
        self._map_win = None
        self.map_canvas = None
        self._map_static_drawn = False
        self._map_drone_item = None
        self._last_pose_key = None
        self._gf_center_item = None
        self._tool_var = None
        self._circle_radius_var = None
        self._poly_points = []
        self._incl_pts = []
        self._incl_rect = None
        self._incl_zmin_var = None
        self._incl_zmax_var = None
        self._excl_circles = []
        self._excl_polys = []
        self._draw_layer_var = None

        # Sistema de capas
        self._current_layer = 0
        self._last_layer = 0
        self._layer_label = None
        self._layer1_min_var = None
        self._layer1_max_var = None
        self._layer2_min_var = None
        self._layer2_max_var = None
        self._layer3_min_var = None
        self._layer3_max_var = None

        # FPV
        self.fpv_label = None
        self._fpv_running = False
        self._fpv_thread = None
        self._frame_lock = threading.Lock()
        self._last_bgr = None
        self._want_stream_on = False
        self._rec_running = False
        self._rec_writer = None
        self._rec_path = None
        self._rec_fps = 30
        self._rec_size = (640, 480)
        # FPV externo (ventana OpenCV)
        self._fpv_ext_running = False
        self._fpv_ext_thread = None
        self._fpv_ext_cap = None
        # Directorios legacy (para migración)
        self._legacy_shots_dir = os.path.abspath("captures")
        self._legacy_recs_dir = os.path.abspath("videos")

        # Gestor de sesiones
        self._sessions_dir = os.path.abspath("sesiones")
        self._session_manager = SessionManager(self._sessions_dir)

        # Migrar archivos antiguos si existen
        if os.path.exists(self._legacy_shots_dir) or os.path.exists(self._legacy_recs_dir):
            migrate_legacy_files(self._legacy_shots_dir, self._legacy_recs_dir, self._sessions_dir)
        self._rec_badge_var = tk.StringVar(value="")
        self._hud_msg = None
        self._hud_until = 0.0
        self._cv_cap = None

        # Joystick
        self._joy_thread = None
        self._joy_running = False
        self._joy_max_rc = 100
        self._joy_speed_var = tk.IntVar(value=self._joy_max_rc)
        self._joy_label_var = tk.StringVar(value="Joystick: —")
        self._joy_evt_var = tk.StringVar(value="")

        # Configuración de botones del joystick (FIJA)
        self._joy_bindings = {
            'photo': 0,
            'rec': 1,
            'takeoff': 2,
            'land': 3
        }

        # Keepalive
        self._keepalive_running = False
        self._keepalive_paused = False
        self._last_rc_sent = 0

        # Para guardar última posición de aterrizaje
        self._last_land_x = 0.0
        self._last_land_y = 0.0
        self._last_land_z = 0.0
        self._last_land_yaw = 0.0

        self._build_ui()
        self._bind_keys()
        self._init_joystick()

    def _hud_show(self, msg: str, duration_sec: float = 1.5):
        try:
            self._hud_msg = str(msg)
            self._hud_until = time.time() + float(duration_sec)
        except Exception:
            self._hud_msg = None
            self._hud_until = 0.0

    def _draw_overlays(self, canvas: np.ndarray):
        h, w = canvas.shape[:2]
        try:
            if self._rec_running:
                cv2.circle(canvas, (18, 18), 8, (0, 0, 255), thickness=-1)
                cv2.putText(canvas, "REC", (34, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
        except Exception:
            pass
        try:
            now = time.time()
            if self._hud_msg and now < self._hud_until:
                txt = self._hud_msg
                (tw, th), _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
                pad = 10
                box_w = tw + pad * 2
                box_h = th + pad * 2
                x0 = max(6, (w - box_w) // 2)
                y0 = max(6, h - box_h - 10)
                x1 = min(w - 6, x0 + box_w)
                y1 = min(h - 6, y0 + box_h)
                overlay = canvas.copy()
                cv2.rectangle(overlay, (x0, y0), (x1, y1), (0, 0, 0), thickness=-1)
                alpha = 0.6
                cv2.addWeighted(overlay, alpha, canvas, 1 - alpha, 0, dst=canvas)
                cv2.rectangle(canvas, (x0, y0), (x1, y1), (255, 255, 255), thickness=1)
                tx = x0 + pad
                ty = y0 + pad + th
                cv2.putText(canvas, txt, (tx, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
            elif self._hud_msg and now >= self._hud_until:
                self._hud_msg = None
        except Exception:
            pass

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
        tk.Button(conn, text="Conectar", width=12, command=self.on_connect,
                  bg="#ffb347", font=("Arial", 10, "bold")).grid(row=0, column=0, **pad)
        tk.Button(conn, text="Desconectar", width=12, command=self.on_disconnect,
                  font=("Arial", 10, "bold")).grid(row=0, column=1, **pad)
        tk.Button(conn, text="Salir", width=12, command=self.on_exit,
                  bg="#ff6961", font=("Arial", 10, "bold")).grid(row=0, column=2, **pad)

        # Vuelo
        flight = tk.Frame(self.root, bd=1, relief="groove")
        flight.pack(fill="x", **pad)
        tk.Button(flight, text="Despegar (Enter)", width=16, command=self.on_takeoff,
                  bg="#90ee90", font=("Arial", 10, "bold")).grid(row=0, column=0, **pad)
        tk.Button(flight, text="Aterrizar (Espacio)", width=16, command=self.on_land,
                  bg="#ff6961", font=("Arial", 10, "bold")).grid(row=0, column=1, **pad)
        tk.Label(flight, textvariable=self._rec_badge_var, fg="#d00", font=("Arial", 10, "bold")).grid(row=0, column=2,
                                                                                                       sticky="w",
                                                                                                       padx=10)

        # PANEL DE TELEMETRÍA (reemplaza FPV Tkinter)
        telemetry_frame = tk.Frame(self.root, bd=2, relief="groove", bg="#1a1a1a")
        telemetry_frame.pack(fill="x", padx=8, pady=8)

        # Fila 1: Batería, Estado, Tiempo
        row1 = tk.Frame(telemetry_frame, bg="#1a1a1a")
        row1.pack(fill="x", padx=10, pady=(10, 5))

        # Batería
        bat_frame = tk.Frame(row1, bg="#1a1a1a")
        bat_frame.pack(side="left", padx=(0, 20))
        tk.Label(bat_frame, text="🔋", font=("Arial", 12), bg="#1a1a1a", fg="white").pack(side="left")
        self._telem_bat_var = tk.StringVar(value="—%")
        self._telem_bat_label = tk.Label(bat_frame, textvariable=self._telem_bat_var,
                                          font=("Arial", 11, "bold"), bg="#1a1a1a", fg="#28a745")
        self._telem_bat_label.pack(side="left", padx=5)

        # Estado
        state_frame = tk.Frame(row1, bg="#1a1a1a")
        state_frame.pack(side="left", padx=20)
        tk.Label(state_frame, text="📡", font=("Arial", 12), bg="#1a1a1a", fg="white").pack(side="left")
        self._telem_state_var = tk.StringVar(value="Desconectado")
        self._telem_state_label = tk.Label(state_frame, textvariable=self._telem_state_var,
                                            font=("Arial", 10), bg="#1a1a1a", fg="#6c757d")
        self._telem_state_label.pack(side="left", padx=5)

        # Tiempo de vuelo
        time_frame = tk.Frame(row1, bg="#1a1a1a")
        time_frame.pack(side="right", padx=(20, 0))
        tk.Label(time_frame, text="⏱", font=("Arial", 12), bg="#1a1a1a", fg="white").pack(side="left")
        self._telem_time_var = tk.StringVar(value="00:00")
        tk.Label(time_frame, textvariable=self._telem_time_var,
                 font=("Arial", 11, "bold"), bg="#1a1a1a", fg="#17a2b8").pack(side="left", padx=5)

        # Separador
        tk.Frame(telemetry_frame, height=1, bg="#333").pack(fill="x", padx=10, pady=5)

        # Fila 2: Posición X, Y, Z, Yaw
        row2 = tk.Frame(telemetry_frame, bg="#1a1a1a")
        row2.pack(fill="x", padx=10, pady=5)

        self._telem_x_var = tk.StringVar(value="X: —")
        self._telem_y_var = tk.StringVar(value="Y: —")
        self._telem_z_var = tk.StringVar(value="Z: —")
        self._telem_yaw_var = tk.StringVar(value="Yaw: —")

        for var in [self._telem_x_var, self._telem_y_var, self._telem_z_var, self._telem_yaw_var]:
            tk.Label(row2, textvariable=var, font=("Consolas", 10), bg="#1a1a1a", fg="#adb5bd",
                     width=12, anchor="w").pack(side="left", padx=5)

        # Fila 3: Velocidades Vx, Vy, Vz
        row3 = tk.Frame(telemetry_frame, bg="#1a1a1a")
        row3.pack(fill="x", padx=10, pady=5)

        self._telem_vx_var = tk.StringVar(value="Vx: —")
        self._telem_vy_var = tk.StringVar(value="Vy: —")
        self._telem_vz_var = tk.StringVar(value="Vz: —")

        for var in [self._telem_vx_var, self._telem_vy_var, self._telem_vz_var]:
            tk.Label(row3, textvariable=var, font=("Consolas", 10), bg="#1a1a1a", fg="#adb5bd",
                     width=12, anchor="w").pack(side="left", padx=5)

        # Separador
        tk.Frame(telemetry_frame, height=1, bg="#333").pack(fill="x", padx=10, pady=5)

        # Fila 4: Botones de control
        row4 = tk.Frame(telemetry_frame, bg="#1a1a1a")
        row4.pack(fill="x", padx=10, pady=(5, 10))

        tk.Button(row4, text="🖥 FPV", command=self.toggle_fpv_external,
                  bg="#6f42c1", fg="white", font=("Arial", 9, "bold"), bd=0,
                  activebackground="#5a32a3", padx=12, pady=5).pack(side="left", padx=3)
        tk.Button(row4, text="📷 Foto", command=self.take_snapshot,
                  bg="#17a2b8", fg="white", font=("Arial", 9, "bold"), bd=0,
                  activebackground="#138496", padx=12, pady=5).pack(side="left", padx=3)
        tk.Button(row4, text="⏺ Grabar", command=self.toggle_recording,
                  bg="#dc3545", fg="white", font=("Arial", 9, "bold"), bd=0,
                  activebackground="#c82333", padx=12, pady=5).pack(side="left", padx=3)
        tk.Button(row4, text="🛬 Aterrizar", command=self._quick_land,
                  bg="#fd7e14", fg="white", font=("Arial", 9, "bold"), bd=0,
                  activebackground="#e76a00", padx=12, pady=5).pack(side="left", padx=3)

        tk.Label(row4, textvariable=self._joy_label_var, fg="#6c757d", bg="#1a1a1a",
                 font=("Arial", 8)).pack(side="right", padx=4)

        # Variables de tiempo de vuelo
        self._flight_start_time = None
        self._telemetry_update_running = False

        # Mapa, Geofence y Galería
        gf = tk.Frame(self.root, bd=1, relief="groove")
        gf.pack(fill="x", **pad)
        btn_container = tk.Frame(gf)
        btn_container.pack(pady=6)
        tk.Button(btn_container, text="🗺 Abrir Mapa y Geofence", command=self.open_map_window,
                  bg="#87CEEB", font=("Arial", 10, "bold"), padx=20, pady=5).pack(side="left", padx=4)
        tk.Button(btn_container, text="📁 Galería de Vuelos", command=self.open_gallery_window,
                  bg="#DDA0DD", font=("Arial", 10, "bold"), padx=20, pady=5).pack(side="left", padx=4)
        tk.Button(btn_container, text="📋 Editor de Misiones", command=self.open_mission_window,
                  bg="#98D8C8", font=("Arial", 10, "bold"), padx=20, pady=5).pack(side="left", padx=4)

        # Nota compacta
        note = tk.Label(self.root, fg="#555", font=("Arial", 8),
                        text="Joystick: IZQ=XY | DER=altura+yaw | B0=Foto | B1=Video | B2=Despegar | B3=Aterrizar")
        note.pack(pady=2)

    def _bind_keys(self):
        self.root.bind("<Up>", lambda e: self.do_move("forward"))
        self.root.bind("<Down>", lambda e: self.do_move("back"))
        self.root.bind("<Left>", lambda e: self.do_move("left"))
        self.root.bind("<Right>", lambda e: self.do_move("right"))
        self.root.bind("<Next>", lambda e: self.do_move("down"))
        self.root.bind("<Prior>", lambda e: self.do_move("up"))
        self.root.bind("<space>", lambda e: self.on_land())
        self.root.bind("<Return>", lambda e: self.on_takeoff())
        self.root.bind("<Key-q>", lambda e: self.do_turn("ccw"))
        self.root.bind("<Key-e>", lambda e: self.do_turn("cw"))

    #  Conexión / Telemetría
    def on_connect(self):
        if self.dron.state == "connected":
            return
        try:
            self.dron.connect()
            self.state_var.set(self.dron.state)
            self._ensure_pose_origin()
            self.dron.startTelemetry(freq_hz=20)
            self._telemetry_running = True
            self._schedule_telemetry_pull()
            self._start_keepalive()
            # Iniciar telemetría visual
            self._start_telemetry_update()
            # Iniciar sesión de vuelo
            session_id = self._session_manager.start_session()
            print(f"[Session] Nueva sesión iniciada: {session_id}")
        except Exception as e:
            messagebox.showerror("Conectar", f"No se pudo conectar: {e}")

    def on_disconnect(self):
        self._stop_keepalive()
        self._stop_telemetry_update()
        self.stop_fpv()
        self.stop_fpv_external()
        self._stop_recording()
        # Terminar sesión de vuelo
        if self._session_manager.is_session_active():
            self._session_manager.end_session()
            print("[Session] Sesión finalizada")
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
            try:
                if self._map_win and tk.Toplevel.winfo_exists(self._map_win):
                    self._map_win.destroy()
            except Exception:
                pass
            self._map_win = None
            self.map_canvas = None

    def on_takeoff(self):
        if self.dron.state not in ("connected", "flying"):
            messagebox.showwarning("TakeOff", "Conecta primero.")
            return
        bat = getattr(self.dron, "battery_pct", None)
        if isinstance(bat, int) and bat < BAT_MIN_SAFE:
            if not messagebox.askokcancel("Batería baja", f"Batería {bat}%. ¿Despegar?"):
                return
        try:
            self._pause_keepalive()
            ok = self.dron.takeOff(0.5, blocking=False)
            if not ok:
                messagebox.showerror("TakeOff", "No se pudo despegar.")
            else:
                self._ensure_pose_origin()
                self._restart_gf_monitor(force=True)
                try:
                    self._reapply_exclusions_to_backend()
                except Exception:
                    pass
                self._hud_show("Despegado", 1.5)
        except Exception as e:
            messagebox.showerror("TakeOff", str(e))
        finally:
            self.root.after(200, self._resume_keepalive)

    def on_land(self):
        if getattr(self, "_ui_landing", False):
            return
        self._ui_landing = True
        try:
            st = getattr(self.dron, "state", "")
            try:
                h = float(getattr(self.dron, "height_cm", 0) or 0)
            except Exception:
                h = 0.0
            if h <= 20:
                return
            if st not in ("flying", "hovering", "takingoff", "landing"):
                return
            self._pause_keepalive()
            self.dron.Land(blocking=False)
            self._hud_show(" Aterrizando", 1.5)
        except Exception as e:
            messagebox.showerror("Land", str(e))
        finally:
            self.root.after(150, self._resume_keepalive)
            self.root.after(4500, lambda: setattr(self, "_ui_landing", False))

    def on_exit(self):
        try:
            self._stop_joystick()
            self.stop_fpv()
            self.stop_fpv_external()
            self._stop_recording()
            self.on_disconnect()
        finally:
            try:
                self.root.destroy()
            except Exception:
                pass

    def apply_speed(self):
        try:
            sp = int(self.speed_var.get())
            sp = max(10, min(100, sp))
            self.dron.set_speed(sp)
            self.speed_var.set(sp)
        except Exception as e:
            messagebox.showerror("Velocidad", f"No se pudo aplicar: {e}")

    def do_move(self, cmd: str):
        if self.dron.state != "flying":
            return
        try:
            step = int(self.step_var.get())
            step = max(20, min(500, step))
            self._pause_keepalive()
            try:
                if cmd == "forward":
                    self.dron.forward(step)
                elif cmd == "back":
                    self.dron.back(step)
                elif cmd == "left":
                    self.dron.left(step)
                elif cmd == "right":
                    self.dron.right(step)
                elif cmd == "up":
                    self.dron.up(step)
                elif cmd == "down":
                    self.dron.down(step)
            except Exception as e:
                print(f"[move] Error: {e}")
            finally:
                self.root.after(150, self._resume_keepalive)
        except Exception:
            pass

    def do_turn(self, direction: str):
        if self.dron.state != "flying":
            return
        try:
            angle = int(self.angle_var.get())
            angle = max(1, min(360, angle))
            self._pause_keepalive()
            if direction == "cw":
                self.dron.cw(angle)
            else:
                self.dron.ccw(angle)
        except Exception:
            pass
        finally:
            self.root.after(150, self._resume_keepalive)

    #KEEPALIVE
    def _start_keepalive(self):
        if getattr(self, "_keepalive_running", False):
            return
        self._keepalive_running = True
        self._keepalive_paused = False

        def _loop():
            while self._keepalive_running:
                try:
                    if self._keepalive_paused:
                        time.sleep(0.05)
                        continue

                    st = getattr(self.dron, "state", "")
                    if st == "flying":
                        last_rc_time = getattr(self, "_last_rc_sent", 0)
                        if time.time() - last_rc_time > 2.0:
                            try:
                                self.dron._send("battery?")
                            except Exception:
                                pass
                except Exception:
                    pass
                time.sleep(1.0)

        threading.Thread(target=_loop, daemon=True).start()

    def _stop_keepalive(self):
        self._keepalive_running = False

    def _pause_keepalive(self):
        self._keepalive_paused = True

    def _resume_keepalive(self):
        self._keepalive_paused = False

    #TELEMETRÍA
    def _schedule_telemetry_pull(self):
        if not self._telemetry_running:
            return
        self._pull_telemetry()
        self.root.after(100, self._schedule_telemetry_pull)

    def _pull_telemetry(self):
        try:
            bat = getattr(self.dron, "battery_pct", None)
            h = getattr(self.dron, "height_cm", None)
            snr = getattr(self.dron, "wifi_snr", None)
            st = getattr(self.dron, "state", "disconnected")
            self.state_var.set(st)
            self.bat_var.set(f"{bat}%" if isinstance(bat, int) else "—")
            self.h_var.set(f"{h} cm" if isinstance(h, (int, float)) else "0 cm")
            self.wifi_var.set(f"{snr}" if isinstance(snr, int) else "—")

            pose = getattr(self.dron, "pose", None)
            if pose is not None and h is not None and isinstance(h, (int, float)):
                pose.z_cm = float(h)

            if pose:
                x = getattr(pose, "x_cm", None)
                y = getattr(pose, "y_cm", None)
                z = getattr(pose, "z_cm", None)
                yaw = getattr(pose, "yaw_deg", None)
                if x is not None and y is not None:
                    try:
                        self._last_land_x = float(x)
                        self._last_land_y = float(y)
                        self._last_land_z = float(z) if z is not None else 0.0
                        self._last_land_yaw = float(yaw) if yaw is not None else 0.0
                    except Exception:
                        pass

            self._update_map_drone()
            self._update_mission_drone()
        except Exception as e:
            print(f"[telemetry] Error: {e}")

    def _ensure_pose_origin(self):
        try:
            if not hasattr(self.dron, "pose") or self.dron.pose is None:
                if hasattr(self.dron, "set_pose_origin"):
                    self.dron.set_pose_origin()
        except Exception:
            pass

    #GEOFENCE
    def on_gf_activate(self):
        try:
            # Coordenadas absolutas (no simétricas)
            x1 = float(self.gf_x1_var.get() or 0.0)
            y1 = float(self.gf_y1_var.get() or 0.0)
            x2 = float(self.gf_x2_var.get() or 0.0)
            y2 = float(self.gf_y2_var.get() or 0.0)
            zmin = float(self.gf_zmin_var.get() or 0.0)
            zmax = float(self.gf_zmax_var.get() or 200.0)
            mode = self.gf_mode_var.get()

            # Asegurar que x1 < x2, y1 < y2
            if x1 > x2:
                x1, x2 = x2, x1
            if y1 > y2:
                y1, y2 = y2, y1

            width_x = abs(x2 - x1)
            width_y = abs(y2 - y1)

            if width_x <= 0 or width_y <= 0:
                messagebox.showwarning("Geofence", "Define una zona válida (dibuja o introduce coordenadas).")
                return

            # Calcular centro para compatibilidad con backend
            cx = (x1 + x2) / 2
            cy = (y1 + y2) / 2

            if hasattr(self.dron, "set_geofence"):
                self.dron.set_geofence(max_x_cm=width_x, max_y_cm=width_y, max_z_cm=zmax, z_min_cm=zmin, mode=mode)
            else:
                setattr(self.dron, "_gf_enabled", True)
                setattr(self.dron, "_gf_center", (cx, cy))
                setattr(self.dron, "_gf_limits", {"x1": x1, "y1": y1, "x2": x2, "y2": y2, "zmin": zmin, "zmax": zmax})
                setattr(self.dron, "_gf_mode", mode)

            # Guardar el rectángulo de inclusión para dibujarlo (coordenadas absolutas)
            self._incl_rect = (x1, y1, x2, y2)

            self._reapply_exclusions_to_backend()
            self._hud_show(f"Geofence {mode} activado", 1.5)
            if self._map_win and tk.Toplevel.winfo_exists(self._map_win):
                self._redraw_map_static()
        except Exception as e:
            messagebox.showerror("Geofence", f"Error: {e}")

    def on_gf_disable(self):
        try:
            if hasattr(self.dron, "disable_geofence"):
                self.dron.disable_geofence()
            else:
                setattr(self.dron, "_gf_enabled", False)
            self._hud_show("Geofence desactivado", 1.5)
            if self._map_win and tk.Toplevel.winfo_exists(self._map_win):
                self._redraw_map_static()
        except Exception as e:
            messagebox.showerror("Geofence", f"Error: {e}")

    def _clear_geofence(self):
        """Limpia la zona de geofence dibujada."""
        # Limpiar variables
        self.gf_x1_var.set("-100")
        self.gf_y1_var.set("-100")
        self.gf_x2_var.set("100")
        self.gf_y2_var.set("100")
        self._incl_rect = None

        # Desactivar geofence
        if hasattr(self.dron, "disable_geofence"):
            self.dron.disable_geofence()
        else:
            setattr(self.dron, "_gf_enabled", False)
            setattr(self.dron, "_gf_limits", {})

        # Limpiar puntos temporales
        if hasattr(self, '_gf_pts'):
            self._gf_pts.clear()
        if self.map_canvas:
            self.map_canvas.delete("gf_temp")

        self._hud_show("Geofence limpiado", 1.5)
        if self._map_win and tk.Toplevel.winfo_exists(self._map_win):
            self._redraw_map_static()

    def _restart_gf_monitor(self, force=False):
        try:
            if hasattr(self.dron, "_stop_geofence_monitor"):
                self.dron._stop_geofence_monitor()
            if hasattr(self.dron, "_start_geofence_monitor"):
                self.dron._start_geofence_monitor()
        except Exception:
            pass

    def _reapply_exclusions_to_backend(self):
        try:
            if not hasattr(self.dron, "_gf_excl_circles"):
                self.dron._gf_excl_circles = []
            if not hasattr(self.dron, "_gf_excl_polys"):
                self.dron._gf_excl_polys = []
            self.dron._gf_excl_circles.clear()
            self.dron._gf_excl_polys.clear()
            for c in self._excl_circles:
                self.dron._gf_excl_circles.append(c)
            for p in self._excl_polys:
                self.dron._gf_excl_polys.append(p)
        except Exception:
            pass

    #FPV
    def start_fpv(self):
        if self._fpv_running:
            return
        try:
            self._want_stream_on = True
            if hasattr(self.dron, "startVideo"):
                self.dron.startVideo()
            else:
                try:
                    self.dron._send("streamon")
                except Exception:
                    pass
        except Exception:
            pass
        try:
            if self._cv_cap is None or not self._cv_cap.isOpened():
                self._cv_cap = cv2.VideoCapture("udp://0.0.0.0:11111", cv2.CAP_FFMPEG)
                # Reducir buffer para minimizar latencia
                self._cv_cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass
        self._fpv_running = True
        self._fpv_thread = threading.Thread(target=self._fpv_loop, daemon=True)
        self._fpv_thread.start()
        self._hud_show("FPV iniciado", 1.0)

    def stop_fpv(self):
        self._fpv_running = False
        self._want_stream_on = False
        time.sleep(0.05)
        try:
            if hasattr(self.dron, "stopVideo"):
                self.dron.stopVideo()
            else:
                try:
                    self.dron._send("streamoff")
                except Exception:
                    pass
        except Exception:
            pass
        try:
            if self._cv_cap is not None:
                self._cv_cap.release()
        except Exception:
            pass
        finally:
            self._cv_cap = None
        self._hud_show("FPV detenido", 1.0)

    def _read_frame_generic(self):
        try:
            if self._cv_cap is not None and self._cv_cap.isOpened():
                # Vaciar frames antiguos del buffer para reducir latencia
                # grab() descarta frames sin decodificar (rápido)
                # Usamos 10 grabs para vaciar más agresivamente el buffer
                for _ in range(10):
                    self._cv_cap.grab()
                # Leer el frame más reciente (grab + decode)
                ok, frame = self._cv_cap.read()
                if ok and isinstance(frame, np.ndarray) and frame.size > 0:
                    return frame
        except Exception:
            pass
        return None

    def _fpv_loop(self):
        last_badge_toggle = 0.0
        rec_on = False
        try:
            while self._fpv_running:
                frame_bgr = self._read_frame_generic()
                if frame_bgr is None:
                    self._set_fpv_text("(esperando…)")
                    time.sleep(0.04)
                    continue
                with self._frame_lock:
                    self._last_bgr = frame_bgr
                h, w = frame_bgr.shape[:2]
                target_w, target_h = self._fpv_w, self._fpv_h
                scale = min(target_w / max(1, w), target_h / max(1, h))
                new_w, new_h = max(1, int(w * scale)), max(1, int(h * scale))
                bgr_resized = cv2.resize(frame_bgr, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
                off_x = (target_w - new_w) // 2
                off_y = (target_h - new_h) // 2
                canvas_base = np.zeros((target_h, target_w, 3), dtype=np.uint8)
                canvas_base[off_y:off_y + new_h, off_x:off_x + new_w] = bgr_resized
                canvas_preview = canvas_base.copy()
                self._draw_overlays(canvas_preview)
                rgb = cv2.cvtColor(canvas_preview, cv2.COLOR_BGR2RGB)
                img = Image.fromarray(rgb)
                imgtk = ImageTk.PhotoImage(image=img)
                self.fpv_label.configure(image=imgtk, text="")
                self.fpv_label.image = imgtk
                if self._rec_running:
                    if self._rec_writer is None:
                        self._start_writer((target_w, target_h))
                    try:
                        self._rec_writer.write(canvas_base)
                    except Exception:
                        pass
                    now = time.time()
                    if now - last_badge_toggle > 0.5:
                        rec_on = not rec_on
                        self._rec_badge_var.set("[REC]" if rec_on else "REC")
                        last_badge_toggle = now
                else:
                    self._rec_badge_var.set("")
                time.sleep(0.01)
            self._rec_badge_var.set("")
        except Exception:
            pass

    def _set_fpv_text(self, text):
        self.fpv_label.configure(text=text, image="")
        self.fpv_label.image = None

    def take_snapshot(self):
        with self._frame_lock:
            frame = None if self._last_bgr is None else self._last_bgr.copy()
        if frame is None:
            self._hud_show(" Sin frame", 1.5)
            return
        # Usar gestor de sesiones para obtener ruta
        path = self._session_manager.get_photo_path()
        try:
            cv2.imwrite(path, frame)
            self._hud_show(f"📷 Guardada", 1.8)
        except Exception:
            self._hud_show("Error", 1.5)

    def toggle_recording(self):
        if self._rec_running:
            self._stop_recording()
        else:
            self._start_recording()

    def _start_writer(self, size_wh):
        self._rec_size = size_wh
        fourcc = cv2. VideoWriter_fourcc(*"mp4v")
        self._rec_writer = cv2.VideoWriter(self._rec_path, fourcc, self._rec_fps, self._rec_size)

    def _start_recording(self):
        # Usar gestor de sesiones para obtener ruta
        self._rec_path = self._session_manager.get_video_path()
        self._rec_running = True
        self._rec_writer = None
        self._hud_show("⏺ Grabando", 1.5)

    def _stop_recording(self):
        if self._rec_running:
            self._rec_running = False
            try:
                if self._rec_writer is not None:
                    self._rec_writer.release()
            except Exception:
                pass
            self._rec_writer = None
            self._hud_show("Guardado", 1.5)

    # FPV EXTERNO (ventana OpenCV - baja latencia)
    def toggle_fpv_external(self):
        """Alterna la ventana FPV externa (OpenCV)."""
        if self._fpv_ext_running:
            self.stop_fpv_external()
        else:
            self.start_fpv_external()

    def start_fpv_external(self):
        """Abre ventana FPV con OpenCV (baja latencia)."""
        if self._fpv_ext_running:
            return
        # Activar stream si no está activo
        try:
            if hasattr(self.dron, "_tello"):
                self.dron._tello.streamoff()
                time.sleep(0.2)
                self.dron._tello.streamon()
                time.sleep(0.3)
        except Exception:
            pass
        self._fpv_ext_running = True
        self._fpv_ext_thread = threading.Thread(target=self._fpv_external_loop, daemon=True)
        self._fpv_ext_thread.start()
        self._hud_show("🖥 FPV Externo", 1.5)

    def stop_fpv_external(self):
        """Cierra la ventana FPV externa."""
        self._fpv_ext_running = False
        if self._fpv_ext_thread is not None:
            try:
                self._fpv_ext_thread.join(timeout=1.0)
            except Exception:
                pass
        self._fpv_ext_thread = None
        if self._fpv_ext_cap is not None:
            try:
                self._fpv_ext_cap.release()
            except Exception:
                pass
            self._fpv_ext_cap = None
        try:
            cv2.destroyWindow("Tello FPV (Low Latency)")
        except Exception:
            pass

    def _fpv_external_loop(self):
        """Loop de la ventana FPV externa con cv2.imshow (baja latencia)."""
        window_name = "Tello FPV (Low Latency)"
        try:
            # Abrir captura UDP directa
            self._fpv_ext_cap = cv2.VideoCapture("udp://0.0.0.0:11111", cv2.CAP_FFMPEG)
            if not self._fpv_ext_cap.isOpened():
                self._hud_show("Error UDP", 2.0)
                self._fpv_ext_running = False
                return
            cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(window_name, 640, 480)

            while self._fpv_ext_running:
                ok, frame = self._fpv_ext_cap.read()
                if not ok or frame is None:
                    time.sleep(0.01)
                    continue

                # Guardar frame para snapshots y grabación
                with self._frame_lock:
                    self._last_bgr = frame.copy()

                # Grabación de video
                if self._rec_running:
                    if self._rec_writer is None:
                        h, w = frame.shape[:2]
                        self._start_writer((w, h))
                    try:
                        self._rec_writer.write(frame)
                    except Exception:
                        pass

                cv2.imshow(window_name, frame)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
        except Exception:
            pass
        finally:
            self._fpv_ext_running = False
            if self._fpv_ext_cap is not None:
                try:
                    self._fpv_ext_cap.release()
                except Exception:
                    pass
                self._fpv_ext_cap = None
            try:
                cv2.destroyWindow(window_name)
            except Exception:
                pass

    # TELEMETRÍA
    def _start_telemetry_update(self):
        """Inicia el loop de actualización de telemetría."""
        if self._telemetry_update_running:
            return
        self._telemetry_update_running = True
        self._flight_start_time = time.time()
        self._telemetry_loop()

    def _stop_telemetry_update(self):
        """Detiene el loop de actualización de telemetría."""
        self._telemetry_update_running = False
        self._flight_start_time = None

    def _telemetry_loop(self):
        """Loop que actualiza los datos de telemetría cada 300ms."""
        if not self._telemetry_update_running:
            return

        try:
            # Batería
            bat = getattr(self.dron, "battery_pct", None)
            if bat is not None:
                self._telem_bat_var.set(f"{bat}%")
                # Color según nivel
                if bat > 50:
                    self._telem_bat_label.configure(fg="#28a745")  # Verde
                elif bat > 20:
                    self._telem_bat_label.configure(fg="#ffc107")  # Amarillo
                else:
                    self._telem_bat_label.configure(fg="#dc3545")  # Rojo
            else:
                self._telem_bat_var.set("—%")

            # Estado
            state = getattr(self.dron, "state", "disconnected")
            state_names = {
                "connected": "Conectado",
                "flying": "Volando",
                "hovering": "Hovering",
                "landing": "Aterrizando",
                "takingoff": "Despegando",
                "disconnected": "Desconectado"
            }
            state_colors = {
                "connected": "#28a745",
                "flying": "#17a2b8",
                "hovering": "#17a2b8",
                "landing": "#ffc107",
                "takingoff": "#ffc107",
                "disconnected": "#6c757d"
            }
            self._telem_state_var.set(state_names.get(state, state))
            self._telem_state_label.configure(fg=state_colors.get(state, "#6c757d"))

            # Tiempo de vuelo
            if self._flight_start_time:
                elapsed = int(time.time() - self._flight_start_time)
                mins, secs = divmod(elapsed, 60)
                self._telem_time_var.set(f"{mins:02d}:{secs:02d}")

            # Posición
            pose = getattr(self.dron, "pose", None)
            if pose:
                self._telem_x_var.set(f"X: {int(pose.x_cm)}")
                self._telem_y_var.set(f"Y: {int(pose.y_cm)}")
                self._telem_z_var.set(f"Z: {int(pose.z_cm)}")
                yaw = getattr(pose, "yaw_deg", 0) or 0
                self._telem_yaw_var.set(f"Yaw: {int(yaw)}°")
            else:
                self._telem_x_var.set("X: —")
                self._telem_y_var.set("Y: —")
                self._telem_z_var.set("Z: —")
                self._telem_yaw_var.set("Yaw: —")

            # Velocidades
            vx = getattr(self.dron, "velocity_x", None)
            vy = getattr(self.dron, "velocity_y", None)
            vz = getattr(self.dron, "velocity_z", None)
            self._telem_vx_var.set(f"Vx: {int(vx)}" if vx is not None else "Vx: —")
            self._telem_vy_var.set(f"Vy: {int(vy)}" if vy is not None else "Vy: —")
            self._telem_vz_var.set(f"Vz: {int(vz)}" if vz is not None else "Vz: —")

        except Exception:
            pass

        # Programar siguiente actualización
        if self._telemetry_update_running:
            self.root.after(300, self._telemetry_loop)

    def _quick_land(self):
        """Aterrizaje rápido desde el panel de telemetría."""
        if self.dron.state in ("flying", "hovering"):
            try:
                self.dron.land()
            except Exception:
                pass

    # JOYSTICK
    def _init_joystick(self):
        if self._joy_running:
            return

        self._joy_running = True

        def _joy_thread_fn():
            controller = JoystickController(
                axis_left_x=0,
                axis_left_y=1,
                axis_right_x=2,
                axis_right_y=4,
                invert_left_y=True,
                invert_right_y=True,
                deadzone=0.1,
                expo=1.5
            )

            if not controller.connect():
                print("[joy] No se pudo conectar al joystick")
                def _ui():
                    self._joy_label_var.set(" —")
                self.root.after(0, _ui)
                return

            joy_name = controller.joystick.get_name()
            def _ui_name():
                self._joy_label_var.set(f" {joy_name}")
            self.root.after(0, _ui_name)

            print(f"[Joystick] Conectado: {joy_name}")

            last_takeoff = False
            last_land = False
            last_photo = False
            last_rec = False

            while self._joy_running:
                try:
                    vx, vy, vz, yaw = controller.read_axes()


                    # Capturar valores para el closure
                    _vx, _vy, _vz, _yaw = vx, vy, vz, yaw

                    def _send():
                        if self.dron.state == "flying":
                            vx_gf, vy_gf, vz_gf, yaw_gf = _vx, _vy, _vz, _yaw

                            try:
                                vx_gf, vy_gf, vz_gf, yaw_gf = self.dron.aplicar_geofence_rc(_vx, _vy, _vz, _yaw)
                                self.dron.rc(int(vx_gf), int(vy_gf), int(vz_gf), int(yaw_gf))
                                self._last_rc_sent = time.time()
                            except Exception:
                                self.dron.rc(int(_vx), int(_vy), int(_vz), int(_yaw))

                            if hasattr(self.dron, "pose") and self.dron.pose:
                                self.dron.pose.update_from_rc(vy_gf, vx_gf, vz_gf, yaw_gf, dt_sec=0.05)

                    self.root.after(0, _send)

                    takeoff_pressed = controller.get_button(2)
                    land_pressed = controller.get_button(3)
                    photo_pressed = controller.get_button(0)
                    rec_pressed = controller.get_button(1)

                    if takeoff_pressed and not last_takeoff:
                        def _t():
                            print("[joy] Botón 2 - Despegar")
                            self.on_takeoff()
                        self.root.after(0, _t)
                    last_takeoff = takeoff_pressed

                    if land_pressed and not last_land:
                        def _l():
                            print("[joy] Botón 3 - Aterrizar")
                            self.on_land()
                        self.root.after(0, _l)
                    last_land = land_pressed

                    if photo_pressed and not last_photo:
                        def _p():
                            print("[joy] Botón 0 - Foto")
                            self.take_snapshot()
                        self.root.after(0, _p)
                    last_photo = photo_pressed

                    if rec_pressed and not last_rec:
                        def _r():
                            print("[joy] Botón 1 - Grabar/Parar")
                            self.toggle_recording()
                        self.root.after(0, _r)
                    last_rec = rec_pressed

                    time.sleep(0.05)

                except Exception as e:
                    print(f"[joy] Error: {e}")
                    time.sleep(0.1)

            try:
                controller.disconnect()
            except Exception:
                pass
            print("[joy] Thread finalizado")

        self._joy_thread = threading.Thread(target=_joy_thread_fn, daemon=True)
        self._joy_thread.start()

    def _stop_joystick(self):
        self._joy_running = False

    #MAPA
    def open_map_window(self):

        if self._map_win and tk.Toplevel.winfo_exists(self._map_win):
            self._map_win.lift()
            return

        self._map_win = tk.Toplevel(self.root)
        self._map_win.title("Mapa Geofence")
        self._map_win.geometry(f"{MAP_SIZE_PX + 300}x{MAP_SIZE_PX + 50}")

        # Canvas
        self.map_canvas = tk.Canvas(
            self._map_win,
            width=MAP_SIZE_PX,
            height=MAP_SIZE_PX,
            bg=MAP_BG_COLOR,
            highlightthickness=0
        )
        self.map_canvas.pack(side="left", padx=10, pady=10)

        # Panel lateral con scroll
        side_container = tk.Frame(self._map_win, bd=1, relief="groove")
        side_container.pack(side="right", fill="y", padx=10, pady=10)

        # Canvas para scroll
        side_canvas = tk.Canvas(side_container, width=220, highlightthickness=0)
        scrollbar = tk.Scrollbar(side_container, orient="vertical", command=side_canvas.yview)
        side_panel = tk.Frame(side_canvas)

        # Configurar scroll
        side_panel.bind("<Configure>", lambda e: side_canvas.configure(scrollregion=side_canvas.bbox("all")))
        side_canvas.create_window((0, 0), window=side_panel, anchor="nw")
        side_canvas.configure(yscrollcommand=scrollbar.set)

        # Scroll con rueda del ratón
        def _on_mousewheel(event):
            side_canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")
        side_canvas.bind_all("<MouseWheel>", _on_mousewheel)

        scrollbar.pack(side="right", fill="y")
        side_canvas.pack(side="left", fill="both", expand=True)

        # Colores del tema
        BG_CARD = "#f8f9fa"
        BG_HEADER = "#343a40"
        FG_HEADER = "#ffffff"
        ACCENT = "#007bff"
        layer_colors = ["#28a745", "#fd7e14", "#007bff"]  # Verde, Naranja, Azul

        # ═══════════════════════════════════════════════════════
        # SECCIÓN: DIBUJAR OBSTÁCULOS
        # ═══════════════════════════════════════════════════════
        card1 = tk.Frame(side_panel, bg=BG_CARD, bd=1, relief="solid")
        card1.pack(fill="x", padx=4, pady=4)

        tk.Label(card1, text="  DIBUJAR OBSTÁCULOS", font=("Arial", 9, "bold"),
                 bg=BG_HEADER, fg=FG_HEADER, anchor="w").pack(fill="x", ipady=4)

        content1 = tk.Frame(card1, bg=BG_CARD)
        content1.pack(fill="x", padx=8, pady=8)

        # Tipo de dibujo
        self._tool_var = tk.StringVar(value="none")
        tool_frame = tk.Frame(content1, bg=BG_CARD)
        tool_frame.pack(fill="x", pady=2)
        tk.Radiobutton(tool_frame, text="Nada", variable=self._tool_var, value="none",
                       bg=BG_CARD, activebackground=BG_CARD).pack(side="left")
        tk.Radiobutton(tool_frame, text="⭕", variable=self._tool_var, value="circle",
                       bg=BG_CARD, activebackground=BG_CARD, font=("Arial", 12)).pack(side="left")
        tk.Radiobutton(tool_frame, text="⬜", variable=self._tool_var, value="rectangle",
                       bg=BG_CARD, activebackground=BG_CARD, font=("Arial", 12)).pack(side="left")
        tk.Radiobutton(tool_frame, text="⬡", variable=self._tool_var, value="polygon",
                       bg=BG_CARD, activebackground=BG_CARD, font=("Arial", 12)).pack(side="left")

        # Radio
        radio_frame = tk.Frame(content1, bg=BG_CARD)
        radio_frame.pack(fill="x", pady=4)
        tk.Label(radio_frame, text="Radio:", bg=BG_CARD, font=("Arial", 8)).pack(side="left")
        self._circle_radius_var = tk.IntVar(value=30)
        tk.Entry(radio_frame, textvariable=self._circle_radius_var, width=5,
                 justify="center").pack(side="left", padx=4)
        tk.Label(radio_frame, text="cm", bg=BG_CARD, font=("Arial", 8)).pack(side="left")

        # Selector de capa
        self._draw_layer_var = tk.StringVar(value="1")
        self._draw_layer_var.trace_add("write", self._on_draw_layer_change)

        tk.Label(content1, text="Capa:", bg=BG_CARD, font=("Arial", 8)).pack(anchor="w")
        layer_btn_frame = tk.Frame(content1, bg=BG_CARD)
        layer_btn_frame.pack(fill="x", pady=2)

        for val, color, txt in [("1", layer_colors[0], "C1"), ("2", layer_colors[1], "C2"),
                                 ("3", layer_colors[2], "C3"), ("all", "#6c757d", "ALL")]:
            tk.Radiobutton(layer_btn_frame, text=txt, variable=self._draw_layer_var, value=val,
                           bg=color, fg="white", selectcolor=color, activebackground=color,
                           indicatoron=0, width=4, font=("Arial", 8, "bold")).pack(side="left", padx=1)

        # Botones de acción
        btn_row = tk.Frame(content1, bg=BG_CARD)
        btn_row.pack(fill="x", pady=(8, 0))
        tk.Button(btn_row, text="✓ Cerrar ⬡", command=self._close_polygon,
                  bg="#6c757d", fg="white", font=("Arial", 8), bd=0,
                  activebackground="#5a6268").pack(side="left", fill="x", expand=True, padx=1)
        tk.Button(btn_row, text="🗑 Limpiar", command=self._clear_exclusions,
                  bg="#dc3545", fg="white", font=("Arial", 8), bd=0,
                  activebackground="#c82333").pack(side="left", fill="x", expand=True, padx=1)

        # Variables para inclusión (usadas en templates)
        self._incl_zmin_var = tk.IntVar(value=0)
        self._incl_zmax_var = tk.IntVar(value=120)

        # ═══════════════════════════════════════════════════════
        # SECCIÓN: GEOFENCE (zona de vuelo permitida)
        # ═══════════════════════════════════════════════════════
        card_gf = tk.Frame(side_panel, bg=BG_CARD, bd=1, relief="solid")
        card_gf.pack(fill="x", padx=4, pady=4)

        tk.Label(card_gf, text="  GEOFENCE (zona segura)", font=("Arial", 9, "bold"),
                 bg=BG_HEADER, fg=FG_HEADER, anchor="w").pack(fill="x", ipady=4)

        content_gf = tk.Frame(card_gf, bg=BG_CARD)
        content_gf.pack(fill="x", padx=8, pady=8)

        # Botón para dibujar en el mapa
        tk.Button(content_gf, text="📍 Dibujar zona (2 clics)", command=self._start_geofence_rect,
                  bg=ACCENT, fg="white", font=("Arial", 8), bd=0,
                  activebackground="#0056b3").pack(fill="x", pady=(0, 6))

        # Fila 1: X1, Y1 (esquina 1)
        row1 = tk.Frame(content_gf, bg=BG_CARD)
        row1.pack(fill="x", pady=1)
        tk.Label(row1, text="Esquina 1:", bg=BG_CARD, font=("Arial", 8)).pack(side="left")
        tk.Entry(row1, textvariable=self.gf_x1_var, width=5, justify="center").pack(side="left", padx=1)
        tk.Entry(row1, textvariable=self.gf_y1_var, width=5, justify="center").pack(side="left", padx=1)

        # Fila 2: X2, Y2 (esquina 2)
        row2 = tk.Frame(content_gf, bg=BG_CARD)
        row2.pack(fill="x", pady=1)
        tk.Label(row2, text="Esquina 2:", bg=BG_CARD, font=("Arial", 8)).pack(side="left")
        tk.Entry(row2, textvariable=self.gf_x2_var, width=5, justify="center").pack(side="left", padx=1)
        tk.Entry(row2, textvariable=self.gf_y2_var, width=5, justify="center").pack(side="left", padx=1)

        # Fila 3: Z min y Z max
        row3 = tk.Frame(content_gf, bg=BG_CARD)
        row3.pack(fill="x", pady=1)
        tk.Label(row3, text="Altura Z:", bg=BG_CARD, font=("Arial", 8)).pack(side="left")
        tk.Entry(row3, textvariable=self.gf_zmin_var, width=4, justify="center").pack(side="left", padx=1)
        tk.Label(row3, text="-", bg=BG_CARD).pack(side="left")
        tk.Entry(row3, textvariable=self.gf_zmax_var, width=4, justify="center").pack(side="left", padx=1)
        tk.Label(row3, text="cm", bg=BG_CARD, font=("Arial", 8)).pack(side="left")

        # Fila 4: Modo
        row4 = tk.Frame(content_gf, bg=BG_CARD)
        row4.pack(fill="x", pady=1)
        tk.Label(row4, text="Modo:", bg=BG_CARD, font=("Arial", 8)).pack(side="left")
        tk.Radiobutton(row4, text="Soft", variable=self.gf_mode_var, value="soft",
                       bg=BG_CARD, activebackground=BG_CARD, font=("Arial", 8)).pack(side="left")
        tk.Radiobutton(row4, text="Hard", variable=self.gf_mode_var, value="hard",
                       bg=BG_CARD, activebackground=BG_CARD, font=("Arial", 8)).pack(side="left")

        # Fila 5: Botones
        row5 = tk.Frame(content_gf, bg=BG_CARD)
        row5.pack(fill="x", pady=(6, 0))
        tk.Button(row5, text="✓ Activar", command=self.on_gf_activate,
                  bg="#28a745", fg="white", font=("Arial", 8), bd=0,
                  activebackground="#218838").pack(side="left", fill="x", expand=True, padx=1)
        tk.Button(row5, text="✗ Desactivar", command=self.on_gf_disable,
                  bg="#6c757d", fg="white", font=("Arial", 8), bd=0,
                  activebackground="#5a6268").pack(side="left", fill="x", expand=True, padx=1)
        tk.Button(row5, text="🗑", command=self._clear_geofence,
                  bg="#dc3545", fg="white", font=("Arial", 8), bd=0, width=3,
                  activebackground="#c82333").pack(side="left", padx=1)

        # ═══════════════════════════════════════════════════════
        # SECCIÓN: CAPAS DE ALTITUD
        # ═══════════════════════════════════════════════════════
        card4 = tk.Frame(side_panel, bg=BG_CARD, bd=1, relief="solid")
        card4.pack(fill="x", padx=4, pady=4)

        tk.Label(card4, text="  CAPAS DE ALTITUD", font=("Arial", 9, "bold"),
                 bg=BG_HEADER, fg=FG_HEADER, anchor="w").pack(fill="x", ipady=4)

        layers_container = tk.Frame(card4, bg=BG_CARD)
        layers_container.pack(fill="x", padx=6, pady=6)

        # Obtener valores actuales de las capas
        if hasattr(self.dron, "get_layers"):
            layers = self.dron.get_layers()
        else:
            layers = [
                {"z_min": 0, "z_max": 60},
                {"z_min": 60, "z_max": 120},
                {"z_min": 120, "z_max": 200},
            ]

        # Usar los mismos colores del tema (ya definidos arriba)

        # Variables para los límites
        self._layer1_max_var = tk.IntVar(value=int(layers[0]["z_max"]))
        self._layer2_max_var = tk.IntVar(value=int(layers[1]["z_max"]))
        self._layer3_max_var = tk.IntVar(value=int(layers[2]["z_max"]))

        # Labels para mostrar los rangos dinámicamente
        self._layer1_range_var = tk.StringVar(value=f"0 - {layers[0]['z_max']} cm")
        self._layer2_range_var = tk.StringVar(value=f"{layers[0]['z_max']} - {layers[1]['z_max']} cm")
        self._layer3_range_var = tk.StringVar(value=f"{layers[1]['z_max']} - {layers[2]['z_max']} cm")

        # Margen mínimo entre capas
        MIN_GAP = 20

        def on_c1_change(*args):
            c1 = self._layer1_max_var.get()
            c2 = self._layer2_max_var.get()
            # Si C1 >= C2, empujar C2 hacia arriba
            if c1 >= c2 - MIN_GAP:
                self._layer2_max_var.set(c1 + MIN_GAP)
            update_ranges()

        def on_c2_change(*args):
            c1 = self._layer1_max_var.get()
            c2 = self._layer2_max_var.get()
            c3 = self._layer3_max_var.get()
            # Si C2 <= C1, empujar C1 hacia abajo
            if c2 <= c1 + MIN_GAP:
                self._layer1_max_var.set(c2 - MIN_GAP)
            # Si C2 >= C3, empujar C3 hacia arriba
            if c2 >= c3 - MIN_GAP:
                self._layer3_max_var.set(c2 + MIN_GAP)
            update_ranges()

        def on_c3_change(*args):
            c2 = self._layer2_max_var.get()
            c3 = self._layer3_max_var.get()
            # Si C3 <= C2, empujar C2 hacia abajo
            if c3 <= c2 + MIN_GAP:
                self._layer2_max_var.set(c3 - MIN_GAP)
            update_ranges()

        def update_ranges():
            c1 = self._layer1_max_var.get()
            c2 = self._layer2_max_var.get()
            c3 = self._layer3_max_var.get()
            self._layer1_range_var.set(f"0 - {c1} cm")
            self._layer2_range_var.set(f"{c1} - {c2} cm")
            self._layer3_range_var.set(f"{c2} - {c3} cm")

            # Actualizar también el indicador de la esquina
            if self._layer_label and self._draw_layer_var:
                layer = self._draw_layer_var.get()
                if layer == "1":
                    zmin, zmax = 0, c1
                elif layer == "2":
                    zmin, zmax = c1, c2
                elif layer == "3":
                    zmin, zmax = c2, c3
                else:  # all
                    zmin, zmax = 0, c3

                if layer == "all":
                    layer_text = "Todas las capas"
                else:
                    layer_text = f"Capa {layer}"

                self._layer_label.config(text=f"{layer_text}\n({zmin}-{zmax} cm)")

        # Capa 3 (arriba - azul) - techo
        layer3_frame = tk.Frame(layers_container, bg=layer_colors[2], bd=0)
        layer3_frame.pack(fill="x", padx=2, pady=1)
        tk.Label(layer3_frame, text="C3", font=("Arial", 9, "bold"),
                 bg=layer_colors[2], fg="white", width=3).pack(side="left", padx=2)
        tk.Scale(layer3_frame, from_=100, to=300, orient="horizontal",
                 variable=self._layer3_max_var, length=90, sliderlength=18,
                 bg=layer_colors[2], fg="white", troughcolor=layer_colors[2],
                 highlightthickness=0, font=("Arial", 7), bd=0,
                 command=lambda v: on_c3_change()).pack(side="left", padx=2)
        tk.Label(layer3_frame, textvariable=self._layer3_range_var, font=("Arial", 7),
                 bg=layer_colors[2], fg="white", width=10).pack(side="right", padx=2)

        # Capa 2 (medio - naranja)
        layer2_frame = tk.Frame(layers_container, bg=layer_colors[1], bd=0)
        layer2_frame.pack(fill="x", padx=2, pady=1)
        tk.Label(layer2_frame, text="C2", font=("Arial", 9, "bold"),
                 bg=layer_colors[1], fg="white", width=3).pack(side="left", padx=2)
        tk.Scale(layer2_frame, from_=50, to=250, orient="horizontal",
                 variable=self._layer2_max_var, length=90, sliderlength=18,
                 bg=layer_colors[1], fg="white", troughcolor=layer_colors[1],
                 highlightthickness=0, font=("Arial", 7), bd=0,
                 command=lambda v: on_c2_change()).pack(side="left", padx=2)
        tk.Label(layer2_frame, textvariable=self._layer2_range_var, font=("Arial", 7),
                 bg=layer_colors[1], fg="white", width=10).pack(side="right", padx=2)

        # Capa 1 (abajo - verde)
        layer1_frame = tk.Frame(layers_container, bg=layer_colors[0], bd=0)
        layer1_frame.pack(fill="x", padx=2, pady=1)
        tk.Label(layer1_frame, text="C1", font=("Arial", 9, "bold"),
                 bg=layer_colors[0], fg="white", width=3).pack(side="left", padx=2)
        tk.Scale(layer1_frame, from_=20, to=150, orient="horizontal",
                 variable=self._layer1_max_var, length=90, sliderlength=18,
                 bg=layer_colors[0], fg="white", troughcolor=layer_colors[0],
                 highlightthickness=0, font=("Arial", 7), bd=0,
                 command=lambda v: on_c1_change()).pack(side="left", padx=2)
        tk.Label(layer1_frame, textvariable=self._layer1_range_var, font=("Arial", 7),
                 bg=layer_colors[0], fg="white", width=10).pack(side="right", padx=2)

        # Suelo - estilo mejorado
        suelo_frame = tk.Frame(layers_container, bg="#8B4513")
        suelo_frame.pack(fill="x", padx=2, pady=(4, 2))
        tk.Label(suelo_frame, text="═══ SUELO ═══", font=("Arial", 8, "bold"),
                 bg="#8B4513", fg="#FFE4C4").pack(fill="x", ipady=2)

        # Botón Aplicar con estilo del tema
        tk.Button(layers_container, text="✓ Aplicar capas", command=self._apply_layers,
                  bg="#28a745", fg="white", font=("Arial", 8, "bold"), bd=0,
                  activebackground="#218838", cursor="hand2").pack(fill="x", padx=2, pady=(6, 4))

        # ═══════════════════════════════════════════════════════
        # SECCIÓN: PLANTILLAS (al final)
        # ═══════════════════════════════════════════════════════
        card_plantillas = tk.Frame(side_panel, bg=BG_CARD, bd=1, relief="solid")
        card_plantillas.pack(fill="x", padx=4, pady=4)

        tk.Label(card_plantillas, text="  PLANTILLAS", font=("Arial", 9, "bold"),
                 bg=BG_HEADER, fg=FG_HEADER, anchor="w").pack(fill="x", ipady=4)

        content_plantillas = tk.Frame(card_plantillas, bg=BG_CARD)
        content_plantillas.pack(fill="x", padx=8, pady=8)

        btn_frame_plantillas = tk.Frame(content_plantillas, bg=BG_CARD)
        btn_frame_plantillas.pack(fill="x")
        tk.Button(btn_frame_plantillas, text="💾 Guardar", command=self._save_template,
                  bg="#28a745", fg="white", font=("Arial", 8), bd=0,
                  activebackground="#218838").pack(side="left", fill="x", expand=True, padx=2)
        tk.Button(btn_frame_plantillas, text="📂 Cargar", command=self._load_template,
                  bg="#17a2b8", fg="white", font=("Arial", 8), bd=0,
                  activebackground="#138496").pack(side="left", fill="x", expand=True, padx=2)

        self.map_canvas.bind("<Button-1>", self._on_map_click)

        # Indicador de capa actual (esquina superior izquierda del canvas)
        self._layer_label = tk.Label(
            self._map_win,
            text="Capa: --",
            font=("Arial", 14, "bold"),
            bg="#333333",
            fg="#ffffff",
            padx=10,
            pady=5
        )
        self._layer_label.place(x=20, y=20)

        # Dibujar
        self._map_static_drawn = False
        self._redraw_map_static()
        self._last_pose_key = None
        self._update_map_drone()

    def _redraw_map_static(self):

        if not self.map_canvas:
            return

        self.map_canvas.delete("all")
        self._map_static_drawn = True

        # Grid
        for i in range(0, MAP_SIZE_PX + 1, int(GRID_STEP_CM * PX_PER_CM)):
            self.map_canvas.create_line(i, 0, i, MAP_SIZE_PX, fill=MAP_AXES_COLOR, width=1)
            self.map_canvas.create_line(0, i, MAP_SIZE_PX, i, fill=MAP_AXES_COLOR, width=1)

        # Ejes
        mid = MAP_SIZE_PX // 2
        self.map_canvas.create_line(mid, 0, mid, MAP_SIZE_PX, fill="#888", width=2)
        self.map_canvas.create_line(0, mid, MAP_SIZE_PX, mid, fill="#888", width=2)

        # Rectángulo de inclusión del backend (geofence activado)
        enabled = bool(getattr(self.dron, "_gf_enabled", False))
        if enabled:
            lim = getattr(self.dron, "_gf_limits", {})
            # Usar coordenadas absolutas si existen
            if "x1" in lim and "x2" in lim:
                gf_x1 = float(lim.get("x1", 0.0))
                gf_y1 = float(lim.get("y1", 0.0))
                gf_x2 = float(lim.get("x2", 0.0))
                gf_y2 = float(lim.get("y2", 0.0))
                p1x, p1y = self._world_to_canvas(gf_x1, gf_y1)
                p2x, p2y = self._world_to_canvas(gf_x2, gf_y2)
                self.map_canvas.create_rectangle(p1x, p1y, p2x, p2y, outline="#00aa00", width=3, tags="inclusion")

        # Rectángulo de inclusión local (definido en mapa pero no activado aún)
        if self._incl_rect:
            self._draw_inclusion_rect(self._incl_rect)

        # Obtener capa actual del dron para colorear exclusiones
        current_layer = self._current_layer if self._current_layer > 0 else 1

        # Colores para exclusiones
        COLOR_IN_LAYER = "#ff0000"      # Rojo - obstáculo en capa actual
        COLOR_OTHER_LAYER = "#ffcccc"   # Rojo claro - obstáculo en otra capa
        WIDTH_IN_LAYER = 3
        WIDTH_OTHER_LAYER = 1

        # Exclusiones (círculos)
        for c in self._excl_circles:
            cx_w, cy_w, r_w = c["cx"], c["cy"], c["r"]
            cx_px, cy_px = self._world_to_canvas(cx_w, cy_w)
            r_px = r_w * PX_PER_CM

            # Determinar si está en la capa actual
            excl_layers = self._get_exclusion_layers(c)
            in_current_layer = current_layer in excl_layers

            color = COLOR_IN_LAYER if in_current_layer else COLOR_OTHER_LAYER
            width = WIDTH_IN_LAYER if in_current_layer else WIDTH_OTHER_LAYER

            self.map_canvas.create_oval(
                cx_px - r_px, cy_px - r_px,
                cx_px + r_px, cy_px + r_px,
                outline=color, width=width, fill="", tags="exclusion"
            )

            # Mostrar en qué capas está (pequeño texto)
            layers_text = ",".join(str(l) for l in excl_layers)
            self.map_canvas.create_text(
                cx_px, cy_px,
                text=f"C{layers_text}",
                font=("Arial", 8),
                fill=color,
                tags="exclusion"
            )

        # Exclusiones (polígonos)
        for p in self._excl_polys:
            pts = p["poly"]
            if len(pts) >= 3:
                canvas_pts = []
                for (px, py) in pts:
                    canvas_pts.extend(self._world_to_canvas(px, py))

                # Determinar si está en la capa actual
                excl_layers = self._get_exclusion_layers(p)
                in_current_layer = current_layer in excl_layers

                color = COLOR_IN_LAYER if in_current_layer else COLOR_OTHER_LAYER
                width = WIDTH_IN_LAYER if in_current_layer else WIDTH_OTHER_LAYER

                self.map_canvas.create_polygon(
                    canvas_pts,
                    outline=color, fill="", width=width, tags="exclusion"
                )

                # Mostrar en qué capas está (en el centroide)
                centroid_x = sum(pt[0] for pt in pts) / len(pts)
                centroid_y = sum(pt[1] for pt in pts) / len(pts)
                cx_text, cy_text = self._world_to_canvas(centroid_x, centroid_y)
                layers_text = ",".join(str(l) for l in excl_layers)
                self.map_canvas.create_text(
                    cx_text, cy_text,
                    text=f"C{layers_text}",
                    font=("Arial", 8),
                    fill=color,
                    tags="exclusion"
                )

        # Redibujar el dron para que no desaparezca
        self._last_pose_key = None  # Forzar redibujado
        self._update_map_drone()

    def _update_map_drone(self):

        try:
            if not self._map_win or not self.map_canvas:
                return
            if not tk.Toplevel.winfo_exists(self._map_win):
                self.map_canvas = None
                self._map_win = None
                return
        except Exception as e:
            print(f"[DEBUG] Error validando ventana: {e}")
            self.map_canvas = None
            self._map_win = None
            return

        pose = getattr(self.dron, "pose", None)
        if not pose:
            return

        x = getattr(pose, "x_cm", None)
        y = getattr(pose, "y_cm", None)
        z = getattr(pose, "z_cm", None)
        yaw = getattr(pose, "yaw_deg", None)

        if x is None or y is None:
            return

        # Actualizar indicador de capa
        self._update_layer_indicator(z)

        key = (round(x, 1), round(y, 1), round(yaw, 1) if yaw else 0)
        if key == self._last_pose_key:
            return
        self._last_pose_key = key

        try:
            if self._map_drone_item:
                self.map_canvas.delete("drone")

            cx_px, cy_px = self._world_to_canvas(x, y)
            rad = 8
            self._map_drone_item = self.map_canvas.create_oval(
                cx_px - rad, cy_px - rad,
                cx_px + rad, cy_px + rad,
                fill=MAP_DRONE_COLOR, outline="black", width=2, tags="drone"
            )

            # Flecha de yaw (rotación) - roja corta - indica hacia dónde apunta el dron
            if yaw is not None:
                th = math.radians(yaw)
                arrow_len = 20
                # Sistema de coordenadas: yaw=0 → forward=+X mundo = arriba en pantalla
                dx_world = arrow_len * math.cos(th)  # X en mundo (forward cuando yaw=0)
                dy_world = arrow_len * math.sin(th)  # Y en mundo (right cuando yaw=0)
                # Conversión a canvas: +X mundo = arriba (-Y canvas), +Y mundo = derecha (+X canvas)
                dx_canvas = dy_world * PX_PER_CM
                dy_canvas = -dx_world * PX_PER_CM

                self.map_canvas.create_line(
                    cx_px, cy_px,
                    cx_px + dx_canvas, cy_px + dy_canvas,
                    fill="red", width=2, arrow=tk.LAST, tags="drone"
                )

            # Flecha de velocidad (dirección de movimiento) - negra larga
            vx = getattr(pose, "vx", None) or getattr(pose, "vel_x", None)
            vy = getattr(pose, "vy", None) or getattr(pose, "vel_y", None)
            if vx is not None and vy is not None:
                speed = math.sqrt(vx**2 + vy**2)
                if speed > 5:  # Solo mostrar si hay movimiento significativo
                    # Normalizar y escalar la flecha
                    arrow_len_vel = min(50, speed * 2)  # Escalar con velocidad
                    # vx, vy están en coordenadas mundo (+X arriba, +Y derecha)
                    dx_vel = (vy / speed) * arrow_len_vel * PX_PER_CM   # Y mundo → X canvas
                    dy_vel = -(vx / speed) * arrow_len_vel * PX_PER_CM  # X mundo → -Y canvas

                    self.map_canvas.create_line(
                        cx_px, cy_px,
                        cx_px + dx_vel, cy_px + dy_vel,
                        fill="black", width=3, arrow=tk.LAST, tags="drone"
                    )
        except Exception as e:
            print(f"[DEBUG] Error dibujando dron: {e}")
            self.map_canvas = None
            self._map_win = None

    def _on_map_click(self, event):

        if not self._tool_var:
            return

        tool = self._tool_var.get()
        wx, wy = self._canvas_to_world(event.x, event.y)

        if tool == "circle":
            self._add_exclusion_circle(wx, wy)
        elif tool == "rectangle":
            self._add_rectangle_point(wx, wy)
        elif tool == "polygon":
            self._add_polygon_point(wx, wy)
        elif tool == "geofence_rect":
            self._add_geofence_point(wx, wy)
        elif tool == "inclusion_rect":
            self._add_inclusion_point(wx, wy)

    def _on_draw_layer_change(self, *args):
        """Actualiza el indicador de capa cuando cambia la selección."""
        if not self._draw_layer_var:
            return

        layer = self._draw_layer_var.get()
        zmin, zmax = self._get_layer_z_range(layer)

        # Actualizar el indicador grande de la esquina
        if self._layer_label:
            # Usar colores consistentes con el tema
            colors = {
                "1": "#28a745",  # Verde - capa 1
                "2": "#fd7e14",  # Naranja - capa 2
                "3": "#007bff",  # Azul - capa 3
                "all": "#6c757d",  # Gris - todas
            }
            bg_color = colors.get(layer, "#333333")

            if layer == "all":
                layer_text = "Todas las capas"
            else:
                layer_text = f"Capa {layer}"

            self._layer_label.config(
                text=f"{layer_text}\n({zmin}-{zmax} cm)",
                bg=bg_color
            )

            # Redibujar mapa para mostrar exclusiones de esa capa
            self._current_layer = int(layer) if layer.isdigit() else 0
            self._redraw_map_static()

    def _get_layer_z_range(self, layer_str):
        """Obtiene el rango Z para una capa seleccionada."""
        # Obtener configuración de capas
        if hasattr(self.dron, "get_layers"):
            layers = self.dron.get_layers()
        else:
            layers = [
                {"z_min": 0, "z_max": 60},
                {"z_min": 60, "z_max": 120},
                {"z_min": 120, "z_max": 200},
            ]

        if layer_str == "all":
            return 0, int(layers[-1]["z_max"]) if layers else 200
        elif layer_str == "1":
            return int(layers[0]["z_min"]), int(layers[0]["z_max"])
        elif layer_str == "2":
            return int(layers[1]["z_min"]), int(layers[1]["z_max"])
        elif layer_str == "3":
            return int(layers[2]["z_min"]), int(layers[2]["z_max"])
        else:
            return 0, 200

    def _add_exclusion_circle(self, wx, wy):

        r = float(self._circle_radius_var.get() or 30.0)

        # Obtener Z según la capa seleccionada
        layer = self._draw_layer_var.get() if self._draw_layer_var else "all"
        zmin, zmax = self._get_layer_z_range(layer)

        self._excl_circles.append({"cx": wx, "cy": wy, "r": r, "zmin": zmin, "zmax": zmax})

        try:
            if hasattr(self.dron, "add_exclusion_circle"):
                self.dron.add_exclusion_circle(
                    cx=wx, cy=wy, r_cm=r,
                    z_min_cm=zmin, z_max_cm=zmax
                )
            else:
                if not hasattr(self.dron, "_gf_excl_circles"):
                    self.dron._gf_excl_circles = []
                self.dron._gf_excl_circles.append({"cx": wx, "cy": wy, "r": r, "zmin": zmin, "zmax": zmax})
        except Exception as e:
            print(f"[WARN] No se pudo enviar círculo al backend: {e}")

        self._redraw_map_static()

    def _add_rectangle_point(self, wx, wy):
        """Añade un punto para dibujar un rectángulo (2 clics: esquinas opuestas)."""
        if not hasattr(self, '_rect_points'):
            self._rect_points = []

        self._rect_points.append((wx, wy))
        cx_px, cy_px = self._world_to_canvas(wx, wy)

        # Mostrar punto temporal
        self.map_canvas.create_oval(
            cx_px - 4, cy_px - 4, cx_px + 4, cy_px + 4,
            fill="purple", outline="black", tags="rect_temp"
        )

        # Si tenemos 2 puntos, crear el rectángulo
        if len(self._rect_points) == 2:
            x1, y1 = self._rect_points[0]
            x2, y2 = self._rect_points[1]

            # Obtener Z según la capa seleccionada
            layer = self._draw_layer_var.get() if self._draw_layer_var else "all"
            zmin, zmax = self._get_layer_z_range(layer)

            # Crear polígono rectangular (4 esquinas)
            rect_poly = [(x1, y1), (x2, y1), (x2, y2), (x1, y2)]
            self._excl_polys.append({"poly": rect_poly, "zmin": zmin, "zmax": zmax})

            try:
                if hasattr(self.dron, "add_exclusion_poly"):
                    self.dron.add_exclusion_poly(
                        points=rect_poly,
                        z_min_cm=zmin, z_max_cm=zmax
                    )
                else:
                    if not hasattr(self.dron, "_gf_excl_polys"):
                        self.dron._gf_excl_polys = []
                    self.dron._gf_excl_polys.append({"poly": rect_poly, "zmin": zmin, "zmax": zmax})
            except Exception as e:
                print(f"[WARN] No se pudo enviar rectángulo al backend: {e}")

            # Limpiar puntos temporales
            self._rect_points.clear()
            self.map_canvas.delete("rect_temp")
            self._redraw_map_static()

    def _add_polygon_point(self, wx, wy):

        self._poly_points.append((wx, wy))
        cx_px, cy_px = self._world_to_canvas(wx, wy)
        self.map_canvas.create_oval(
            cx_px - 4, cy_px - 4, cx_px + 4, cy_px + 4,
            fill="orange", outline="black", tags="poly_temp"
        )

    def _close_polygon(self):

        if len(self._poly_points) < 3:
            messagebox.showwarning("Polígono", "Necesitas al menos 3 puntos.")
            return

        # Obtener Z según la capa seleccionada
        layer = self._draw_layer_var.get() if self._draw_layer_var else "all"
        zmin, zmax = self._get_layer_z_range(layer)

        self._excl_polys.append({"poly": list(self._poly_points), "zmin": zmin, "zmax": zmax})

        try:
            if hasattr(self.dron, "add_exclusion_poly"):
                self.dron.add_exclusion_poly(
                    points=list(self._poly_points),
                    z_min_cm=zmin, z_max_cm=zmax
                )
            else:
                if not hasattr(self.dron, "_gf_excl_polys"):
                    self.dron._gf_excl_polys = []
                self.dron._gf_excl_polys.append({"poly": list(self._poly_points), "zmin": zmin, "zmax": zmax})
        except Exception as e:
            print(f"[WARN] No se pudo enviar polígono al backend: {e}")

        self._poly_points.clear()
        self.map_canvas.delete("poly_temp")
        self._redraw_map_static()

    def _clear_exclusions(self):

        self._excl_circles.clear()
        self._excl_polys.clear()
        self._poly_points.clear()
        if hasattr(self, '_rect_points'):
            self._rect_points.clear()

        if hasattr(self.dron, "_gf_excl_circles"):
            self.dron._gf_excl_circles.clear()
        if hasattr(self.dron, "_gf_excl_polys"):
            self.dron._gf_excl_polys.clear()

        self.map_canvas.delete("poly_temp")
        self.map_canvas.delete("rect_temp")
        self._redraw_map_static()

    def _start_geofence_rect(self):
        """Inicia el dibujo de la zona de geofence en el mapa."""
        if not hasattr(self, '_gf_pts'):
            self._gf_pts = []
        self._gf_pts.clear()
        self._tool_var.set("geofence_rect")
        self._hud_show("Clic en 2 esquinas del geofence", 3.0)

    def _add_geofence_point(self, wx, wy):
        """Añade un punto para definir el rectángulo de geofence."""
        if not hasattr(self, '_gf_pts'):
            self._gf_pts = []

        self._gf_pts.append((wx, wy))
        cx_px, cy_px = self._world_to_canvas(wx, wy)

        # Mostrar punto temporal
        self.map_canvas.create_oval(
            cx_px - 5, cy_px - 5, cx_px + 5, cy_px + 5,
            fill="#00aa00", outline="white", width=2, tags="gf_temp"
        )

        if len(self._gf_pts) == 2:
            x1, y1 = self._gf_pts[0]
            x2, y2 = self._gf_pts[1]

            # Actualizar las variables de coordenadas
            self.gf_x1_var.set(str(int(min(x1, x2))))
            self.gf_y1_var.set(str(int(min(y1, y2))))
            self.gf_x2_var.set(str(int(max(x1, x2))))
            self.gf_y2_var.set(str(int(max(y1, y2))))

            # Guardar para dibujar (coordenadas absolutas)
            gf_x1 = min(x1, x2)
            gf_y1 = min(y1, y2)
            gf_x2 = max(x1, x2)
            gf_y2 = max(y1, y2)
            self._incl_rect = (gf_x1, gf_y1, gf_x2, gf_y2)

            width_x = abs(x2 - x1)
            width_y = abs(y2 - y1)

            # Limpiar
            self._gf_pts.clear()
            self._tool_var.set("none")
            self.map_canvas.delete("gf_temp")

            self._hud_show(f"Zona definida: {width_x:.0f}x{width_y:.0f} cm", 2.0)
            self._redraw_map_static()

    def _start_inclusion_rect(self):

        self._incl_pts.clear()
        self._tool_var.set("inclusion_rect")
        self._hud_show("Clic en 2 esquinas", 2.0)

    def _add_inclusion_point(self, wx, wy):

        self._incl_pts.append((wx, wy))
        if len(self._incl_pts) == 2:
            x1, y1 = self._incl_pts[0]
            x2, y2 = self._incl_pts[1]

            # Coordenadas absolutas ordenadas
            gf_x1 = min(x1, x2)
            gf_y1 = min(y1, y2)
            gf_x2 = max(x1, x2)
            gf_y2 = max(y1, y2)

            # Actualizar variables de geofence
            self.gf_x1_var.set(str(int(gf_x1)))
            self.gf_y1_var.set(str(int(gf_y1)))
            self.gf_x2_var.set(str(int(gf_x2)))
            self.gf_y2_var.set(str(int(gf_y2)))

            # Guardar con coordenadas absolutas
            self._incl_rect = (gf_x1, gf_y1, gf_x2, gf_y2)
            self._incl_pts.clear()
            self._tool_var.set("none")

            width_x = abs(x2 - x1)
            width_y = abs(y2 - y1)
            self._hud_show(f"Zona: {width_x:.0f}x{width_y:.0f} cm", 2.0)
            self._redraw_map_static()

    def _sync_inclusion_to_gf(self):

        if not self._incl_rect:
            messagebox.showwarning("Inclusión", "Define primero el rectángulo.")
            return

        # Ahora _incl_rect es (x1, y1, x2, y2)
        x1, y1, x2, y2 = self._incl_rect
        zmin = self._incl_zmin_var.get()
        zmax = self._incl_zmax_var.get()
        mode = self.gf_mode_var.get()

        # Calcular centro y dimensiones para compatibilidad
        cx = (x1 + x2) / 2
        cy = (y1 + y2) / 2
        width_x = abs(x2 - x1)
        width_y = abs(y2 - y1)

        if hasattr(self.dron, "set_geofence"):
            self.dron.set_geofence(
                max_x_cm=width_x, max_y_cm=width_y,
                max_z_cm=zmax, z_min_cm=zmin, mode=mode
            )
        # Guardar coordenadas absolutas para el dibujo
        setattr(self.dron, "_gf_enabled", True)
        setattr(self.dron, "_gf_center", (cx, cy))
        setattr(self.dron, "_gf_limits", {"x1": x1, "y1": y1, "x2": x2, "y2": y2, "zmin": zmin, "zmax": zmax})
        setattr(self.dron, "_gf_mode", mode)

        self._restart_gf_monitor(force=True)
        self._reapply_exclusions_to_backend()

        self._hud_show(f"Geofence sincronizado", 1.5)
        self._redraw_map_static()

    def _save_template(self):

        path = filedialog.asksaveasfilename(defaultextension=".json", filetypes=[("JSON", "*.json")])
        if not path:
            return

        data = {
            "circles": self._excl_circles,
            "polygons": self._excl_polys,
            "inclusion": self._incl_rect,
            "zmin": self._incl_zmin_var.get(),
            "zmax": self._incl_zmax_var.get(),
            # Configuración de capas
            "layers": {
                "c1_max": self._layer1_max_var.get(),
                "c2_max": self._layer2_max_var.get(),
                "c3_max": self._layer3_max_var.get()
            }
        }

        try:
            with open(path, "w") as f:
                json.dump(data, f, indent=2)
            messagebox.showinfo("Plantilla", "Guardada correctamente.")
        except Exception as e:
            messagebox.showerror("Plantilla", f"Error guardando: {e}")

    def _load_template(self):
        """Carga exclusiones, inclusión y capas desde un archivo JSON."""
        path = filedialog.askopenfilename(filetypes=[("JSON", "*.json")])
        if not path:
            return

        try:
            with open(path, "r") as f:
                data = json.load(f)

            self._excl_circles = data.get("circles", [])
            self._excl_polys = data.get("polygons", [])
            self._incl_rect = data.get("inclusion")
            self._incl_zmin_var.set(data.get("zmin", 0))
            self._incl_zmax_var.set(data.get("zmax", 120))

            if self._incl_rect:
                _, _, max_x, max_y = self._incl_rect
                self.gf_max_x_var.set(str(int(max_x)))
                self.gf_max_y_var.set(str(int(max_y)))

            # Cargar configuración de capas
            layers_config = data.get("layers", {})
            if layers_config:
                self._layer1_max_var.set(layers_config.get("c1_max", 60))
                self._layer2_max_var.set(layers_config.get("c2_max", 120))
                self._layer3_max_var.set(layers_config.get("c3_max", 200))
                # Aplicar las capas al dron
                self._apply_layers()

            self._reapply_exclusions_to_backend()

            self._redraw_map_static()
            messagebox.showinfo("Plantilla", "Cargada correctamente.")

        except Exception as e:
            messagebox.showerror("Plantilla", f"Error cargando: {e}")

    def _world_to_canvas(self, wx, wy):

        mid = MAP_SIZE_PX / 2.0
        cx = mid + (wy * PX_PER_CM)
        cy = mid - (wx * PX_PER_CM)
        return (cx, cy)

    def _canvas_to_world(self, cx, cy):

        mid = MAP_SIZE_PX / 2.0
        wy = (cx - mid) / PX_PER_CM
        wx = (mid - cy) / PX_PER_CM
        return (wx, wy)

    def _draw_inclusion_rect(self, rect_tuple):

        if self.map_canvas is None or not rect_tuple:
            return

        # rect_tuple es (x1, y1, x2, y2) - coordenadas absolutas
        x1, y1, x2, y2 = rect_tuple

        p1x, p1y = self._world_to_canvas(x1, y1)
        p2x, p2y = self._world_to_canvas(x2, y2)

        self.map_canvas.create_rectangle(p1x, p1y, p2x, p2y, outline="#00aa00", width=3, tags=("inclusion",))

    def _update_layer_indicator(self, z_cm):
        """Actualiza el indicador de capa en el mapa."""
        if not self._layer_label:
            return

        # Obtener capa actual usando el método del dron
        if hasattr(self.dron, "get_current_layer"):
            layer = self.dron.get_current_layer(z_cm)
        else:
            # Fallback: calcular localmente
            layer = self._calculate_layer(z_cm)

        if layer != self._current_layer:
            self._last_layer = self._current_layer
            self._current_layer = layer

            # Cambiar color según la capa (colores consistentes con el tema)
            colors = {
                1: "#28a745",  # Verde - capa baja
                2: "#fd7e14",  # Naranja - capa media
                3: "#007bff",  # Azul - capa alta
            }
            bg_color = colors.get(layer, "#333333")

            # Obtener rango de altura de la capa
            if hasattr(self.dron, "get_layers"):
                layers = self.dron.get_layers()
                if 0 < layer <= len(layers):
                    layer_info = layers[layer - 1]
                    z_range = f"({layer_info['z_min']:.0f}-{layer_info['z_max']:.0f}cm)"
                else:
                    z_range = ""
            else:
                z_range = ""

            # Actualizar label
            z_str = f"{z_cm:.0f}" if z_cm is not None else "--"
            self._layer_label.config(
                text=f"Capa {layer} {z_range}\nAltura: {z_str} cm",
                bg=bg_color
            )

            # Redibujar mapa para actualizar colores de obstáculos
            if self._last_layer != 0:  # No redibujar la primera vez
                self._redraw_map_static()

                # Alerta en el HUD del FPV
                direction = "Subiendo" if layer > self._last_layer else "Bajando"
                self._hud_show(f"{direction} a Capa {layer}", 2.0)

    def _calculate_layer(self, z_cm):
        """Calcula la capa localmente (fallback si el dron no tiene el método)."""
        if z_cm is None:
            return 0

        # Rangos por defecto
        if z_cm <= 60:
            return 1
        elif z_cm <= 120:
            return 2
        else:
            return 3

    def _apply_layers(self):
        """Aplica la configuración de capas al dron."""
        try:
            # Los sliders definen el techo de cada capa
            # El suelo de cada capa es el techo de la anterior
            c1_max = self._layer1_max_var.get()
            c2_max = self._layer2_max_var.get()
            c3_max = self._layer3_max_var.get()

            # Validar orden lógico
            if c1_max >= c2_max:
                messagebox.showwarning("Capas", "C1 debe ser menor que C2")
                return
            if c2_max >= c3_max:
                messagebox.showwarning("Capas", "C2 debe ser menor que C3")
                return

            layers = [
                {"name": "Capa 1", "z_min": 0, "z_max": c1_max},
                {"name": "Capa 2", "z_min": c1_max, "z_max": c2_max},
                {"name": "Capa 3", "z_min": c2_max, "z_max": c3_max}
            ]

            # Aplicar al dron
            if hasattr(self.dron, "set_layers"):
                self.dron.set_layers(layers)
                self._hud_show("Capas configuradas", 1.5)

                # Forzar actualización del indicador
                self._current_layer = 0
                pose = getattr(self.dron, "pose", None)
                if pose:
                    z = getattr(pose, "z_cm", 0)
                    self._update_layer_indicator(z)
            else:
                messagebox.showwarning("Capas", "El dron no soporta configuración de capas")

        except Exception as e:
            messagebox.showerror("Capas", f"Error: {e}")

    def _get_exclusion_layers(self, exclusion):
        """Determina qué capas ocupa una exclusión."""
        # Intentar usar el método del dron
        if hasattr(self.dron, "get_exclusion_layers"):
            return self.dron.get_exclusion_layers(exclusion)

        # Fallback: calcular localmente
        excl_zmin = exclusion.get("zmin")
        excl_zmax = exclusion.get("zmax")

        # Si no tiene límites, ocupa todas las capas
        if excl_zmin is None and excl_zmax is None:
            return [1, 2, 3]

        excl_zmin = float(excl_zmin) if excl_zmin is not None else 0.0
        excl_zmax = float(excl_zmax) if excl_zmax is not None else 999.0

        # Rangos por defecto de capas
        layers_ranges = [
            (0, 60),    # Capa 1
            (60, 120),  # Capa 2
            (120, 200)  # Capa 3
        ]

        result = []
        for i, (layer_zmin, layer_zmax) in enumerate(layers_ranges):
            # Hay solapamiento si los rangos se intersectan
            if not (excl_zmax < layer_zmin or excl_zmin > layer_zmax):
                result.append(i + 1)

        return result if result else [1, 2, 3]

    # ═══════════════════════════════════════════════════════════════════════════
    # GALERÍA DE VUELOS
    # ═══════════════════════════════════════════════════════════════════════════

    def open_gallery_window(self):
        """Abre la ventana de galería de vuelos."""
        if hasattr(self, '_gallery_win') and self._gallery_win:
            try:
                if tk.Toplevel.winfo_exists(self._gallery_win):
                    self._gallery_win.lift()
                    return
            except:
                pass

        self._gallery_win = tk.Toplevel(self.root)
        self._gallery_win.title("Galería de Vuelos")
        self._gallery_win.geometry("1000x700")
        self._gallery_win.configure(bg="#f0f0f0")

        # Variables de estado
        self._gallery_selected_session = tk.StringVar(value="")
        self._gallery_filter = tk.StringVar(value="all")
        self._gallery_thumbnails = []
        self._gallery_media_list = []

        # Colores del tema (consistente con mapa)
        BG_CARD = "#f8f9fa"
        BG_HEADER = "#343a40"
        FG_HEADER = "#ffffff"

        # ─────────────────────────────────────────────────────────────────────
        # PANEL IZQUIERDO (Sesiones + Filtros)
        # ─────────────────────────────────────────────────────────────────────
        left_panel = tk.Frame(self._gallery_win, width=250, bg=BG_CARD, bd=1, relief="solid")
        left_panel.pack(side="left", fill="y", padx=10, pady=10)
        left_panel.pack_propagate(False)

        # Sección SESIONES
        sessions_card = tk.Frame(left_panel, bg=BG_CARD)
        sessions_card.pack(fill="x", padx=4, pady=4)

        tk.Label(sessions_card, text="  SESIONES DE VUELO", font=("Arial", 9, "bold"),
                 bg=BG_HEADER, fg=FG_HEADER, anchor="w").pack(fill="x", ipady=4)

        # Lista de sesiones con scroll
        sessions_container = tk.Frame(sessions_card, bg=BG_CARD)
        sessions_container.pack(fill="both", expand=True, padx=4, pady=4)

        sessions_canvas = tk.Canvas(sessions_container, bg=BG_CARD, highlightthickness=0, height=350)
        sessions_scrollbar = tk.Scrollbar(sessions_container, orient="vertical", command=sessions_canvas.yview)
        self._sessions_frame = tk.Frame(sessions_canvas, bg=BG_CARD)

        self._sessions_frame.bind("<Configure>", lambda e: sessions_canvas.configure(scrollregion=sessions_canvas.bbox("all")))
        sessions_canvas.create_window((0, 0), window=self._sessions_frame, anchor="nw", width=220)
        sessions_canvas.configure(yscrollcommand=sessions_scrollbar.set)

        sessions_scrollbar.pack(side="right", fill="y")
        sessions_canvas.pack(side="left", fill="both", expand=True)

        # Sección FILTRAR
        filter_card = tk.Frame(left_panel, bg=BG_CARD, bd=1, relief="solid")
        filter_card.pack(fill="x", padx=4, pady=(10, 4))

        tk.Label(filter_card, text="  FILTRAR", font=("Arial", 9, "bold"),
                 bg=BG_HEADER, fg=FG_HEADER, anchor="w").pack(fill="x", ipady=4)

        filter_content = tk.Frame(filter_card, bg=BG_CARD)
        filter_content.pack(fill="x", padx=8, pady=8)

        tk.Radiobutton(filter_content, text="📁 Todo", variable=self._gallery_filter, value="all",
                       bg=BG_CARD, activebackground=BG_CARD, font=("Arial", 10),
                       command=self._gallery_apply_filter).pack(anchor="w")
        tk.Radiobutton(filter_content, text="📷 Solo fotos", variable=self._gallery_filter, value="photos",
                       bg=BG_CARD, activebackground=BG_CARD, font=("Arial", 10),
                       command=self._gallery_apply_filter).pack(anchor="w")
        tk.Radiobutton(filter_content, text="🎬 Solo vídeos", variable=self._gallery_filter, value="videos",
                       bg=BG_CARD, activebackground=BG_CARD, font=("Arial", 10),
                       command=self._gallery_apply_filter).pack(anchor="w")

        # Botón refrescar
        tk.Button(left_panel, text="🔄 Actualizar", command=self._gallery_refresh,
                  bg="#6c757d", fg="white", font=("Arial", 9), bd=0,
                  activebackground="#5a6268").pack(fill="x", padx=8, pady=8)

        # ─────────────────────────────────────────────────────────────────────
        # PANEL DERECHO (Grid de miniaturas)
        # ─────────────────────────────────────────────────────────────────────
        right_panel = tk.Frame(self._gallery_win, bg="#ffffff", bd=1, relief="solid")
        right_panel.pack(side="right", fill="both", expand=True, padx=(0, 10), pady=10)

        # Header con info de sesión seleccionada
        self._gallery_header = tk.Label(right_panel, text="Selecciona una sesión",
                                         font=("Arial", 11, "bold"), bg="#ffffff", fg="#666", anchor="w")
        self._gallery_header.pack(fill="x", padx=10, pady=(10, 5))

        # Área de miniaturas con scroll
        thumb_container = tk.Frame(right_panel, bg="#ffffff")
        thumb_container.pack(fill="both", expand=True, padx=5, pady=5)

        thumb_canvas = tk.Canvas(thumb_container, bg="#ffffff", highlightthickness=0)
        thumb_scrollbar = tk.Scrollbar(thumb_container, orient="vertical", command=thumb_canvas.yview)
        self._thumb_frame = tk.Frame(thumb_canvas, bg="#ffffff")

        self._thumb_frame.bind("<Configure>", lambda e: thumb_canvas.configure(scrollregion=thumb_canvas.bbox("all")))
        thumb_canvas.create_window((0, 0), window=self._thumb_frame, anchor="nw")
        thumb_canvas.configure(yscrollcommand=thumb_scrollbar.set)

        # Scroll con rueda del ratón
        def _on_mousewheel_gallery(event):
            thumb_canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")
        thumb_canvas.bind_all("<MouseWheel>", _on_mousewheel_gallery)

        thumb_scrollbar.pack(side="right", fill="y")
        thumb_canvas.pack(side="left", fill="both", expand=True)
        self._thumb_canvas = thumb_canvas

        # Barra de estado
        self._gallery_status = tk.Label(right_panel, text="", font=("Arial", 8), bg="#ffffff", fg="#888", anchor="w")
        self._gallery_status.pack(fill="x", padx=10, pady=(0, 5))

        # Cargar sesiones
        self._gallery_refresh()

    def _gallery_refresh(self):
        """Refresca la lista de sesiones."""
        for widget in self._sessions_frame.winfo_children():
            widget.destroy()

        sessions = self._session_manager.list_sessions()

        if not sessions:
            tk.Label(self._sessions_frame, text="No hay sesiones",
                     font=("Arial", 9, "italic"), fg="#888", bg="#f8f9fa").pack(pady=20)
            return

        for session in sessions:
            self._create_session_item(session)

    def _create_session_item(self, session):
        """Crea un item de sesión en la lista."""
        session_id = session["id"]
        photos = session["photos_count"]
        videos = session["videos_count"]

        if session_id == "legacy":
            date_str = "📦 Archivos antiguos"
        else:
            try:
                parts = session_id.split("_")
                date_str = f"📅 {parts[0]}  🕐 {parts[1].replace('-', ':')}"
            except:
                date_str = session_id

        is_selected = self._gallery_selected_session.get() == session_id
        bg_color = "#d4edda" if is_selected else "#f8f9fa"

        item_frame = tk.Frame(self._sessions_frame, bg=bg_color, bd=1, relief="solid", cursor="hand2")
        item_frame.pack(fill="x", pady=2)

        tk.Label(item_frame, text=date_str, font=("Arial", 9, "bold"),
                 bg=bg_color, fg="#333", anchor="w").pack(fill="x", padx=6, pady=(4, 0))
        tk.Label(item_frame, text=f"📷 {photos}  🎬 {videos}",
                 font=("Arial", 8), bg=bg_color, fg="#666", anchor="w").pack(fill="x", padx=6, pady=(0, 4))

        def on_click(e, sid=session_id):
            self._gallery_selected_session.set(sid)
            self._gallery_refresh()
            self._gallery_load_media(sid)

        item_frame.bind("<Button-1>", on_click)
        for child in item_frame.winfo_children():
            child.bind("<Button-1>", on_click)

    def _gallery_load_media(self, session_id):
        """Carga los archivos multimedia de una sesión."""
        filter_type = self._gallery_filter.get()
        self._gallery_media_list = self._session_manager.get_session_media(session_id, filter_type)

        count = len(self._gallery_media_list)
        filter_text = {"all": "archivos", "photos": "fotos", "videos": "vídeos"}[filter_type]
        self._gallery_header.configure(text=f"Sesión: {session_id}  •  {count} {filter_text}")

        self._gallery_thumbnails.clear()
        for widget in self._thumb_frame.winfo_children():
            widget.destroy()

        if not self._gallery_media_list:
            tk.Label(self._thumb_frame, text="No hay archivos en esta sesión",
                     font=("Arial", 10, "italic"), fg="#888", bg="#ffffff").pack(pady=50)
            return

        self._create_thumbnail_grid()

    def _create_thumbnail_grid(self):
        """Crea el grid de miniaturas."""
        THUMB_SIZE = 120
        COLUMNS = 5
        PADDING = 8

        for idx, media in enumerate(self._gallery_media_list):
            row = idx // COLUMNS
            col = idx % COLUMNS

            thumb_frame = tk.Frame(self._thumb_frame, bg="#ffffff", cursor="hand2")
            thumb_frame.grid(row=row, column=col, padx=PADDING, pady=PADDING)

            try:
                if media["type"] == "photo":
                    img = Image.open(media["path"])
                    img.thumbnail((THUMB_SIZE, THUMB_SIZE))
                    photo = ImageTk.PhotoImage(img)
                    label = tk.Label(thumb_frame, image=photo, bg="#eee", bd=2, relief="solid")
                    label.image = photo
                else:
                    label = tk.Label(thumb_frame, text="🎬", font=("Arial", 40),
                                     width=8, height=4, bg="#333", fg="white", bd=2, relief="solid")
            except:
                label = tk.Label(thumb_frame, text="❌", font=("Arial", 30),
                                 width=8, height=4, bg="#fee", bd=2, relief="solid")

            label.pack()

            filename = media["filename"]
            if len(filename) > 15:
                filename = filename[:12] + "..."
            tk.Label(thumb_frame, text=filename, font=("Arial", 7), bg="#ffffff", fg="#666").pack()

            size_mb = media["size_bytes"] / (1024 * 1024)
            tk.Label(thumb_frame, text=f"{size_mb:.1f} MB", font=("Arial", 7), bg="#ffffff", fg="#999").pack()

            def on_click(e, index=idx):
                self._gallery_open_viewer(index)

            label.bind("<Button-1>", on_click)
            thumb_frame.bind("<Button-1>", on_click)
            self._gallery_thumbnails.append(thumb_frame)

        self._thumb_frame.update_idletasks()

    def _gallery_apply_filter(self):
        """Aplica el filtro seleccionado."""
        session_id = self._gallery_selected_session.get()
        if session_id:
            self._gallery_load_media(session_id)

    def _gallery_open_viewer(self, index):
        """Abre el visor ampliado de un archivo."""
        if index < 0 or index >= len(self._gallery_media_list):
            return

        media = self._gallery_media_list[index]

        viewer = tk.Toplevel(self._gallery_win)
        viewer.title("Visor")
        viewer.geometry("900x700")
        viewer.configure(bg="#1a1a1a")
        viewer.transient(self._gallery_win)

        self._viewer_index = index
        self._viewer_win = viewer

        # Variables de reproducción de vídeo
        self._video_playing = False
        self._video_cap = None
        self._video_after_id = None

        # Barra superior
        top_bar = tk.Frame(viewer, bg="#1a1a1a")
        top_bar.pack(fill="x", pady=10)

        def on_close_viewer():
            self._stop_video_playback()
            viewer.destroy()

        tk.Button(top_bar, text="✕ Cerrar", command=on_close_viewer,
                  bg="#dc3545", fg="white", font=("Arial", 10), bd=0,
                  activebackground="#c82333").pack(side="right", padx=20)

        self._viewer_title = tk.Label(top_bar, text="", font=("Arial", 11, "bold"),
                                       bg="#1a1a1a", fg="white")
        self._viewer_title.pack(side="left", padx=20)

        # Área de contenido
        content_frame = tk.Frame(viewer, bg="#1a1a1a")
        content_frame.pack(fill="both", expand=True)

        tk.Button(content_frame, text="◀", font=("Arial", 24), bg="#333", fg="white",
                  bd=0, width=3, command=lambda: self._viewer_navigate(-1, viewer)).pack(side="left", fill="y", padx=5)

        self._viewer_label = tk.Label(content_frame, bg="#1a1a1a")
        self._viewer_label.pack(side="left", fill="both", expand=True)

        tk.Button(content_frame, text="▶", font=("Arial", 24), bg="#333", fg="white",
                  bd=0, width=3, command=lambda: self._viewer_navigate(1, viewer)).pack(side="right", fill="y", padx=5)

        # Controles de vídeo (ocultos por defecto)
        self._video_controls = tk.Frame(viewer, bg="#2a2a2a")

        self._video_play_btn = tk.Button(self._video_controls, text="▶ Play", font=("Arial", 10, "bold"),
                                          bg="#28a745", fg="white", bd=0, width=10,
                                          command=self._toggle_video_playback)
        self._video_play_btn.pack(side="left", padx=10, pady=8)

        self._video_time_label = tk.Label(self._video_controls, text="00:00 / 00:00",
                                           font=("Arial", 9), bg="#2a2a2a", fg="#aaa")
        self._video_time_label.pack(side="left", padx=10)

        self._video_progress = ttk.Scale(self._video_controls, from_=0, to=100, orient="horizontal",
                                          command=self._on_video_seek)
        self._video_progress.pack(side="left", fill="x", expand=True, padx=10)

        # Barra inferior
        bottom_bar = tk.Frame(viewer, bg="#2a2a2a")
        bottom_bar.pack(fill="x", pady=10)

        self._viewer_info = tk.Label(bottom_bar, text="", font=("Arial", 9),
                                      bg="#2a2a2a", fg="#aaa")
        self._viewer_info.pack(side="left", padx=20)

        tk.Button(bottom_bar, text="🗑 Eliminar", command=lambda: self._viewer_delete(viewer),
                  bg="#dc3545", fg="white", font=("Arial", 9), bd=0).pack(side="right", padx=10)

        tk.Button(bottom_bar, text="📂 Abrir carpeta", command=self._viewer_open_folder,
                  bg="#6c757d", fg="white", font=("Arial", 9), bd=0).pack(side="right", padx=10)

        viewer.bind("<Left>", lambda e: self._viewer_navigate(-1, viewer))
        viewer.bind("<Right>", lambda e: self._viewer_navigate(1, viewer))
        viewer.bind("<Escape>", lambda e: on_close_viewer())
        viewer.bind("<space>", lambda e: self._toggle_video_playback())
        viewer.protocol("WM_DELETE_WINDOW", on_close_viewer)

        self._viewer_show(viewer)

    def _viewer_show(self, viewer):
        """Muestra el archivo actual en el visor."""
        # Detener reproducción anterior si existe
        self._stop_video_playback()

        if self._viewer_index < 0 or self._viewer_index >= len(self._gallery_media_list):
            return

        media = self._gallery_media_list[self._viewer_index]
        self._viewer_title.configure(text=f"{media['filename']}  ({self._viewer_index + 1}/{len(self._gallery_media_list)})")

        size_mb = media["size_bytes"] / (1024 * 1024)
        self._viewer_info.configure(text=f"{media['type'].upper()}  •  {size_mb:.2f} MB  •  {media['modified_time'][:10]}")

        if media["type"] == "photo":
            # Ocultar controles de vídeo
            self._video_controls.pack_forget()
            try:
                img = Image.open(media["path"])
                img.thumbnail((750, 500), Image.Resampling.LANCZOS)
                photo = ImageTk.PhotoImage(img)
                self._viewer_label.configure(image=photo, text="")
                self._viewer_label.image = photo
            except Exception as e:
                self._viewer_label.configure(image="", text=f"Error: {e}", fg="red")
        else:
            # Mostrar controles de vídeo
            self._video_controls.pack(fill="x", before=self._viewer_info.master)
            self._current_video_path = media["path"]

            # Abrir vídeo y mostrar primer frame
            try:
                self._video_cap = cv2.VideoCapture(media["path"])
                self._video_fps = self._video_cap.get(cv2.CAP_PROP_FPS) or 30
                self._video_total_frames = int(self._video_cap.get(cv2.CAP_PROP_FRAME_COUNT))
                self._video_duration = self._video_total_frames / self._video_fps

                # Actualizar duración
                duration_str = self._format_time(self._video_duration)
                self._video_time_label.configure(text=f"00:00 / {duration_str}")
                self._video_progress.configure(to=self._video_total_frames)
                self._video_progress.set(0)

                # Mostrar primer frame
                ret, frame = self._video_cap.read()
                if ret:
                    self._display_video_frame(frame)
                self._video_cap.set(cv2.CAP_PROP_POS_FRAMES, 0)

                self._video_play_btn.configure(text="▶ Play", bg="#28a745")
            except Exception as e:
                self._viewer_label.configure(image="", text=f"Error: {e}", fg="red")

    def _display_video_frame(self, frame):
        """Muestra un frame de vídeo en el visor."""
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img = Image.fromarray(frame_rgb)
        img.thumbnail((750, 500), Image.Resampling.LANCZOS)
        photo = ImageTk.PhotoImage(img)
        self._viewer_label.configure(image=photo, text="")
        self._viewer_label.image = photo

    def _format_time(self, seconds):
        """Formatea segundos a MM:SS."""
        mins = int(seconds // 60)
        secs = int(seconds % 60)
        return f"{mins:02d}:{secs:02d}"

    def _toggle_video_playback(self):
        """Alterna play/pause del vídeo."""
        if self._video_playing:
            self._pause_video()
        else:
            self._play_video()

    def _play_video(self):
        """Inicia la reproducción del vídeo."""
        if not self._video_cap or not self._video_cap.isOpened():
            return

        self._video_playing = True
        self._video_play_btn.configure(text="⏸ Pausa", bg="#ffc107")
        self._video_update_frame()

    def _pause_video(self):
        """Pausa la reproducción del vídeo."""
        self._video_playing = False
        self._video_play_btn.configure(text="▶ Play", bg="#28a745")
        if self._video_after_id:
            try:
                self._viewer_win.after_cancel(self._video_after_id)
            except:
                pass
            self._video_after_id = None

    def _stop_video_playback(self):
        """Detiene completamente la reproducción."""
        self._video_playing = False
        if self._video_after_id:
            try:
                self._viewer_win.after_cancel(self._video_after_id)
            except:
                pass
            self._video_after_id = None
        if self._video_cap:
            try:
                self._video_cap.release()
            except:
                pass
            self._video_cap = None

    def _video_update_frame(self):
        """Actualiza el frame del vídeo durante la reproducción."""
        if not self._video_playing or not self._video_cap:
            return

        ret, frame = self._video_cap.read()
        if ret:
            self._display_video_frame(frame)

            # Actualizar progreso
            current_frame = int(self._video_cap.get(cv2.CAP_PROP_POS_FRAMES))
            current_time = current_frame / self._video_fps
            duration_str = self._format_time(self._video_duration)
            current_str = self._format_time(current_time)
            self._video_time_label.configure(text=f"{current_str} / {duration_str}")
            self._video_progress.set(current_frame)

            # Siguiente frame
            delay = int(1000 / self._video_fps)
            self._video_after_id = self._viewer_win.after(delay, self._video_update_frame)
        else:
            # Fin del vídeo - reiniciar
            self._video_cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            self._pause_video()
            self._video_progress.set(0)
            self._video_time_label.configure(text=f"00:00 / {self._format_time(self._video_duration)}")

    def _on_video_seek(self, value):
        """Maneja el seek en la barra de progreso."""
        if not self._video_cap:
            return

        frame_num = int(float(value))
        self._video_cap.set(cv2.CAP_PROP_POS_FRAMES, frame_num)

        # Mostrar frame actual
        ret, frame = self._video_cap.read()
        if ret:
            self._display_video_frame(frame)
            current_time = frame_num / self._video_fps
            self._video_time_label.configure(
                text=f"{self._format_time(current_time)} / {self._format_time(self._video_duration)}"
            )
        # Retroceder un frame para que play continúe desde aquí
        self._video_cap.set(cv2.CAP_PROP_POS_FRAMES, frame_num)

    def _viewer_navigate(self, direction, viewer):
        """Navega al archivo anterior/siguiente."""
        self._stop_video_playback()
        new_index = self._viewer_index + direction
        if 0 <= new_index < len(self._gallery_media_list):
            self._viewer_index = new_index
            self._viewer_show(viewer)

    def _viewer_delete(self, viewer):
        """Elimina el archivo actual."""
        if not messagebox.askyesno("Eliminar", "¿Eliminar este archivo?"):
            return

        media = self._gallery_media_list[self._viewer_index]
        if self._session_manager.delete_media(media["path"]):
            self._gallery_media_list.pop(self._viewer_index)
            if not self._gallery_media_list:
                viewer.destroy()
                self._gallery_apply_filter()
                return
            if self._viewer_index >= len(self._gallery_media_list):
                self._viewer_index = len(self._gallery_media_list) - 1
            self._viewer_show(viewer)
            self._gallery_apply_filter()

    def _viewer_open_folder(self):
        """Abre la carpeta del archivo en el explorador."""
        import subprocess
        import platform
        media = self._gallery_media_list[self._viewer_index]
        folder = os.path.dirname(media["path"])
        try:
            if platform.system() == "Windows":
                os.startfile(folder)
            elif platform.system() == "Darwin":
                subprocess.run(["open", folder])
            else:
                subprocess.run(["xdg-open", folder])
        except Exception as e:
            messagebox.showerror("Error", f"No se pudo abrir: {e}")

    def _open_external(self, path):
        """Abre archivo con aplicación del sistema."""
        import subprocess
        import platform
        try:
            if platform.system() == "Windows":
                os.startfile(path)
            elif platform.system() == "Darwin":
                subprocess.run(["open", path])
            else:
                subprocess.run(["xdg-open", path])
        except Exception as e:
            messagebox.showerror("Error", f"No se pudo abrir: {e}")

    # ═══════════════════════════════════════════════════════════════════════════
    # EDITOR DE MISIONES
    # ═══════════════════════════════════════════════════════════════════════════

    def open_mission_window(self):
        """Abre el editor de misiones."""
        if hasattr(self, '_mission_win') and self._mission_win:
            try:
                if tk.Toplevel.winfo_exists(self._mission_win):
                    self._mission_win.lift()
                    return
            except:
                pass

        self._mission_win = tk.Toplevel(self.root)
        self._mission_win.title("Editor de Misiones")
        self._mission_win.geometry(f"{MAP_SIZE_PX + 320}x{MAP_SIZE_PX + 50}")

        # Estado de la misión
        self._mission_waypoints = []  # Lista de waypoints con acciones
        self._mission_tool = tk.StringVar(value="waypoint")
        self._mission_exclusions = []  # Círculos: {'type': 'circle', 'cx', 'cy', 'r'}
        self._mission_rect_points = []  # Puntos temporales para rectángulo
        self._mission_poly_points = []  # Puntos temporales para polígono
        self._mission_geofence = None  # Geofence propio
        self._mission_running = False
        self._wp_selected_idx = None
        self._selected_obs_idx = None  # Índice del obstáculo seleccionado

        # Canvas del mapa
        self._mission_canvas = tk.Canvas(
            self._mission_win,
            width=MAP_SIZE_PX,
            height=MAP_SIZE_PX,
            bg=MAP_BG_COLOR,
            highlightthickness=0
        )
        self._mission_canvas.pack(side="left", padx=10, pady=10)
        self._mission_canvas.bind("<Button-1>", self._on_mission_canvas_click)

        # Panel lateral con scroll
        side_container = tk.Frame(self._mission_win, bd=1, relief="groove")
        side_container.pack(side="right", fill="y", padx=10, pady=10)

        side_canvas = tk.Canvas(side_container, width=280, highlightthickness=0)
        scrollbar = tk.Scrollbar(side_container, orient="vertical", command=side_canvas.yview)
        side_panel = tk.Frame(side_canvas)

        side_panel.bind("<Configure>", lambda e: side_canvas.configure(scrollregion=side_canvas.bbox("all")))
        side_canvas.create_window((0, 0), window=side_panel, anchor="nw")
        side_canvas.configure(yscrollcommand=scrollbar.set)

        def _on_mousewheel_mission(event):
            side_canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")
        side_canvas.bind_all("<MouseWheel>", _on_mousewheel_mission)

        scrollbar.pack(side="right", fill="y")
        side_canvas.pack(side="left", fill="both", expand=True)

        # Colores del tema
        BG_CARD = "#f8f9fa"
        BG_HEADER = "#343a40"
        FG_HEADER = "#ffffff"
        ACCENT = "#17a2b8"

        # ═══════════════════════════════════════════════════════════════════
        # SECCIÓN: HERRAMIENTAS
        # ═══════════════════════════════════════════════════════════════════
        card_tools = tk.Frame(side_panel, bg=BG_CARD, bd=1, relief="solid")
        card_tools.pack(fill="x", padx=4, pady=4)

        tk.Label(card_tools, text="  HERRAMIENTAS", font=("Arial", 9, "bold"),
                 bg=BG_HEADER, fg=FG_HEADER, anchor="w").pack(fill="x", ipady=4)

        tools_content = tk.Frame(card_tools, bg=BG_CARD)
        tools_content.pack(fill="x", padx=8, pady=8)

        tool_row = tk.Frame(tools_content, bg=BG_CARD)
        tool_row.pack(fill="x", pady=2)

        tk.Radiobutton(tool_row, text="📍 Waypoint", variable=self._mission_tool, value="waypoint",
                       bg=BG_CARD, activebackground=BG_CARD, font=("Arial", 9)).pack(side="left")

        # Fila de obstáculos
        obs_tool_row = tk.Frame(tools_content, bg=BG_CARD)
        obs_tool_row.pack(fill="x", pady=2)
        tk.Label(obs_tool_row, text="Obstáculos:", bg=BG_CARD, font=("Arial", 8)).pack(side="left")
        tk.Radiobutton(obs_tool_row, text="⭕", variable=self._mission_tool, value="obstacle",
                       bg=BG_CARD, activebackground=BG_CARD, font=("Arial", 11)).pack(side="left")
        tk.Radiobutton(obs_tool_row, text="⬜", variable=self._mission_tool, value="obs_rect",
                       bg=BG_CARD, activebackground=BG_CARD, font=("Arial", 11)).pack(side="left")
        tk.Radiobutton(obs_tool_row, text="⬡", variable=self._mission_tool, value="obs_poly",
                       bg=BG_CARD, activebackground=BG_CARD, font=("Arial", 11)).pack(side="left")
        tk.Radiobutton(obs_tool_row, text="✋", variable=self._mission_tool, value="select_obs",
                       bg=BG_CARD, activebackground=BG_CARD, font=("Arial", 11)).pack(side="left")

        # Radio para obstáculos circulares
        obs_row = tk.Frame(tools_content, bg=BG_CARD)
        obs_row.pack(fill="x", pady=2)
        tk.Label(obs_row, text="Radio ⭕:", bg=BG_CARD, font=("Arial", 8)).pack(side="left")
        self._mission_obs_radius = tk.IntVar(value=30)
        tk.Entry(obs_row, textvariable=self._mission_obs_radius, width=5).pack(side="left", padx=4)
        tk.Label(obs_row, text="cm", bg=BG_CARD, font=("Arial", 8)).pack(side="left")

        # Botones para polígonos y eliminar
        obs_btn_row = tk.Frame(tools_content, bg=BG_CARD)
        obs_btn_row.pack(fill="x", pady=2)
        tk.Button(obs_btn_row, text="✓ Cerrar ⬡", command=self._close_mission_polygon,
                  bg="#6c757d", fg="white", font=("Arial", 8), bd=0).pack(side="left", padx=2)
        tk.Button(obs_btn_row, text="🗑 Último obs", command=self._delete_last_obstacle,
                  bg="#dc3545", fg="white", font=("Arial", 8), bd=0).pack(side="left", padx=2)

        # ═══════════════════════════════════════════════════════════════════
        # SECCIÓN: EDITAR OBSTÁCULO SELECCIONADO
        # ═══════════════════════════════════════════════════════════════════
        card_obs_edit = tk.Frame(side_panel, bg=BG_CARD, bd=1, relief="solid")
        card_obs_edit.pack(fill="x", padx=4, pady=4)

        tk.Label(card_obs_edit, text="  EDITAR OBSTÁCULO", font=("Arial", 9, "bold"),
                 bg=BG_HEADER, fg=FG_HEADER, anchor="w").pack(fill="x", ipady=4)

        obs_edit_content = tk.Frame(card_obs_edit, bg=BG_CARD)
        obs_edit_content.pack(fill="x", padx=8, pady=8)

        self._obs_edit_label = tk.Label(obs_edit_content, text="Clic en ✋ y luego en obstáculo",
                                         bg=BG_CARD, font=("Arial", 8, "italic"), fg="#666")
        self._obs_edit_label.pack(anchor="w")

        # Campos de edición (para círculo: cx, cy, r)
        obs_edit_row1 = tk.Frame(obs_edit_content, bg=BG_CARD)
        obs_edit_row1.pack(fill="x", pady=(4, 2))
        tk.Label(obs_edit_row1, text="X:", bg=BG_CARD, font=("Arial", 8)).pack(side="left")
        self._obs_edit_x = tk.Entry(obs_edit_row1, width=5)
        self._obs_edit_x.pack(side="left", padx=(2, 8))
        tk.Label(obs_edit_row1, text="Y:", bg=BG_CARD, font=("Arial", 8)).pack(side="left")
        self._obs_edit_y = tk.Entry(obs_edit_row1, width=5)
        self._obs_edit_y.pack(side="left", padx=(2, 8))

        obs_edit_row2 = tk.Frame(obs_edit_content, bg=BG_CARD)
        obs_edit_row2.pack(fill="x", pady=2)
        tk.Label(obs_edit_row2, text="Radio/Tamaño:", bg=BG_CARD, font=("Arial", 8)).pack(side="left")
        self._obs_edit_size = tk.Entry(obs_edit_row2, width=5)
        self._obs_edit_size.pack(side="left", padx=(2, 8))

        obs_edit_btns = tk.Frame(obs_edit_content, bg=BG_CARD)
        obs_edit_btns.pack(fill="x", pady=(4, 0))
        tk.Button(obs_edit_btns, text="✓ Aplicar", command=self._apply_obs_edit,
                  bg="#28a745", fg="white", font=("Arial", 8, "bold"), bd=0, padx=8).pack(side="left", padx=2)
        tk.Button(obs_edit_btns, text="🗑 Eliminar", command=self._delete_selected_obs,
                  bg="#dc3545", fg="white", font=("Arial", 8, "bold"), bd=0, padx=8).pack(side="left", padx=2)

        # ═══════════════════════════════════════════════════════════════════
        # SECCIÓN: GEOFENCE
        # ═══════════════════════════════════════════════════════════════════
        card_gf = tk.Frame(side_panel, bg=BG_CARD, bd=1, relief="solid")
        card_gf.pack(fill="x", padx=4, pady=4)

        tk.Label(card_gf, text="  GEOFENCE (zona segura)", font=("Arial", 9, "bold"),
                 bg=BG_HEADER, fg=FG_HEADER, anchor="w").pack(fill="x", ipady=4)

        gf_content = tk.Frame(card_gf, bg=BG_CARD)
        gf_content.pack(fill="x", padx=8, pady=8)

        # Coordenadas del geofence
        self._mgf_x1 = tk.IntVar(value=-200)
        self._mgf_y1 = tk.IntVar(value=-200)
        self._mgf_x2 = tk.IntVar(value=200)
        self._mgf_y2 = tk.IntVar(value=200)
        self._mgf_zmin = tk.IntVar(value=20)
        self._mgf_zmax = tk.IntVar(value=150)

        r1 = tk.Frame(gf_content, bg=BG_CARD)
        r1.pack(fill="x", pady=1)
        tk.Label(r1, text="X:", bg=BG_CARD, font=("Arial", 8), width=3).pack(side="left")
        tk.Entry(r1, textvariable=self._mgf_x1, width=5).pack(side="left", padx=2)
        tk.Label(r1, text="a", bg=BG_CARD, font=("Arial", 8)).pack(side="left")
        tk.Entry(r1, textvariable=self._mgf_x2, width=5).pack(side="left", padx=2)
        tk.Label(r1, text="cm", bg=BG_CARD, font=("Arial", 8)).pack(side="left")

        r2 = tk.Frame(gf_content, bg=BG_CARD)
        r2.pack(fill="x", pady=1)
        tk.Label(r2, text="Y:", bg=BG_CARD, font=("Arial", 8), width=3).pack(side="left")
        tk.Entry(r2, textvariable=self._mgf_y1, width=5).pack(side="left", padx=2)
        tk.Label(r2, text="a", bg=BG_CARD, font=("Arial", 8)).pack(side="left")
        tk.Entry(r2, textvariable=self._mgf_y2, width=5).pack(side="left", padx=2)
        tk.Label(r2, text="cm", bg=BG_CARD, font=("Arial", 8)).pack(side="left")

        r3 = tk.Frame(gf_content, bg=BG_CARD)
        r3.pack(fill="x", pady=1)
        tk.Label(r3, text="Z:", bg=BG_CARD, font=("Arial", 8), width=3).pack(side="left")
        tk.Entry(r3, textvariable=self._mgf_zmin, width=5).pack(side="left", padx=2)
        tk.Label(r3, text="a", bg=BG_CARD, font=("Arial", 8)).pack(side="left")
        tk.Entry(r3, textvariable=self._mgf_zmax, width=5).pack(side="left", padx=2)
        tk.Label(r3, text="cm", bg=BG_CARD, font=("Arial", 8)).pack(side="left")

        tk.Button(gf_content, text="✓ Aplicar Geofence", command=self._mission_apply_geofence,
                  bg="#28a745", fg="white", font=("Arial", 8), bd=0).pack(fill="x", pady=4)

        # ═══════════════════════════════════════════════════════════════════
        # SECCIÓN: PLAN DE VUELO
        # ═══════════════════════════════════════════════════════════════════
        card_wp = tk.Frame(side_panel, bg=BG_CARD, bd=1, relief="solid")
        card_wp.pack(fill="x", padx=4, pady=4)

        tk.Label(card_wp, text="  PLAN DE VUELO", font=("Arial", 9, "bold"),
                 bg=BG_HEADER, fg=FG_HEADER, anchor="w").pack(fill="x", ipady=4)

        wp_content = tk.Frame(card_wp, bg=BG_CARD)
        wp_content.pack(fill="x", padx=8, pady=8)

        # Altura por defecto para nuevos waypoints
        hz = tk.Frame(wp_content, bg=BG_CARD)
        hz.pack(fill="x", pady=2)
        tk.Label(hz, text="Altura WP:", bg=BG_CARD, font=("Arial", 8)).pack(side="left")
        self._wp_default_z = tk.IntVar(value=80)
        tk.Entry(hz, textvariable=self._wp_default_z, width=5).pack(side="left", padx=4)
        tk.Label(hz, text="cm", bg=BG_CARD, font=("Arial", 8)).pack(side="left")

        # Lista de waypoints
        tk.Label(wp_content, text="Waypoints:", bg=BG_CARD, font=("Arial", 8, "bold")).pack(anchor="w", pady=(8, 2))

        wp_list_frame = tk.Frame(wp_content, bg=BG_CARD)
        wp_list_frame.pack(fill="x")

        self._wp_listbox = tk.Listbox(wp_list_frame, height=6, font=("Arial", 8), selectmode="single")
        self._wp_listbox.pack(side="left", fill="x", expand=True)
        self._wp_listbox.bind("<<ListboxSelect>>", self._on_wp_select)

        wp_scroll = tk.Scrollbar(wp_list_frame, orient="vertical", command=self._wp_listbox.yview)
        wp_scroll.pack(side="right", fill="y")
        self._wp_listbox.configure(yscrollcommand=wp_scroll.set)

        # Botones de waypoint
        wp_btns = tk.Frame(wp_content, bg=BG_CARD)
        wp_btns.pack(fill="x", pady=4)
        tk.Button(wp_btns, text="🗑", command=self._delete_selected_wp,
                  bg="#dc3545", fg="white", font=("Arial", 8), bd=0, width=3).pack(side="left", padx=2)
        tk.Button(wp_btns, text="⬆", command=lambda: self._move_wp(-1),
                  bg="#6c757d", fg="white", font=("Arial", 8), bd=0, width=3).pack(side="left", padx=2)
        tk.Button(wp_btns, text="⬇", command=lambda: self._move_wp(1),
                  bg="#6c757d", fg="white", font=("Arial", 8), bd=0, width=3).pack(side="left", padx=2)
        tk.Button(wp_btns, text="🗑 Todo", command=self._clear_all_waypoints,
                  bg="#dc3545", fg="white", font=("Arial", 8), bd=0).pack(side="right", padx=2)

        # ═══════════════════════════════════════════════════════════════════
        # SECCIÓN: ACCIONES EN WAYPOINT
        # ═══════════════════════════════════════════════════════════════════
        card_actions = tk.Frame(side_panel, bg=BG_CARD, bd=1, relief="solid")
        card_actions.pack(fill="x", padx=4, pady=4)

        tk.Label(card_actions, text="  ACCIONES EN WAYPOINT", font=("Arial", 9, "bold"),
                 bg=BG_HEADER, fg=FG_HEADER, anchor="w").pack(fill="x", ipady=4)

        actions_content = tk.Frame(card_actions, bg=BG_CARD)
        actions_content.pack(fill="x", padx=8, pady=8)

        self._wp_action_label = tk.Label(actions_content, text="Selecciona un waypoint",
                                          bg=BG_CARD, font=("Arial", 8, "italic"), fg="#666")
        self._wp_action_label.pack(anchor="w")

        # Coordenadas editables
        coords_frame = tk.Frame(actions_content, bg=BG_CARD)
        coords_frame.pack(fill="x", pady=(4, 8))

        tk.Label(coords_frame, text="X:", bg=BG_CARD, font=("Arial", 8)).pack(side="left")
        self._wp_edit_x_entry = tk.Entry(coords_frame, width=5)
        self._wp_edit_x_entry.pack(side="left", padx=(2, 8))
        self._wp_edit_x_entry.insert(0, "0")

        tk.Label(coords_frame, text="Y:", bg=BG_CARD, font=("Arial", 8)).pack(side="left")
        self._wp_edit_y_entry = tk.Entry(coords_frame, width=5)
        self._wp_edit_y_entry.pack(side="left", padx=(2, 8))
        self._wp_edit_y_entry.insert(0, "0")

        tk.Label(coords_frame, text="Z:", bg=BG_CARD, font=("Arial", 8)).pack(side="left")
        self._wp_edit_z_entry = tk.Entry(coords_frame, width=5)
        self._wp_edit_z_entry.pack(side="left", padx=(2, 8))
        self._wp_edit_z_entry.insert(0, "100")

        tk.Button(coords_frame, text="✓", command=self._apply_wp_coords, font=("Arial", 8, "bold"),
                  bg="#28a745", fg="white", bd=0, padx=6).pack(side="left", padx=4)

        # Checkboxes de acciones
        self._action_photo = tk.BooleanVar(value=False)
        self._action_video = tk.BooleanVar(value=False)
        self._action_video_duration = tk.IntVar(value=5)
        self._action_rotate = tk.BooleanVar(value=False)
        self._action_rotate_deg = tk.IntVar(value=90)
        self._action_wait = tk.BooleanVar(value=False)
        self._action_wait_sec = tk.IntVar(value=2)

        # Foto
        af1 = tk.Frame(actions_content, bg=BG_CARD)
        af1.pack(fill="x", pady=1)
        tk.Checkbutton(af1, text="📷 Tomar foto", variable=self._action_photo,
                       bg=BG_CARD, activebackground=BG_CARD, font=("Arial", 8),
                       command=self._update_wp_actions).pack(side="left")

        # Video
        af2 = tk.Frame(actions_content, bg=BG_CARD)
        af2.pack(fill="x", pady=1)
        tk.Checkbutton(af2, text="🎬 Grabar", variable=self._action_video,
                       bg=BG_CARD, activebackground=BG_CARD, font=("Arial", 8),
                       command=self._update_wp_actions).pack(side="left")
        tk.Entry(af2, textvariable=self._action_video_duration, width=3).pack(side="left", padx=2)
        tk.Label(af2, text="seg", bg=BG_CARD, font=("Arial", 8)).pack(side="left")

        # Girar
        af3 = tk.Frame(actions_content, bg=BG_CARD)
        af3.pack(fill="x", pady=1)
        tk.Checkbutton(af3, text="🔄 Girar", variable=self._action_rotate,
                       bg=BG_CARD, activebackground=BG_CARD, font=("Arial", 8),
                       command=self._update_wp_actions).pack(side="left")
        tk.Entry(af3, textvariable=self._action_rotate_deg, width=4).pack(side="left", padx=2)
        tk.Label(af3, text="°", bg=BG_CARD, font=("Arial", 8)).pack(side="left")

        # Esperar
        af4 = tk.Frame(actions_content, bg=BG_CARD)
        af4.pack(fill="x", pady=1)
        tk.Checkbutton(af4, text="⏱ Esperar", variable=self._action_wait,
                       bg=BG_CARD, activebackground=BG_CARD, font=("Arial", 8),
                       command=self._update_wp_actions).pack(side="left")
        tk.Entry(af4, textvariable=self._action_wait_sec, width=3).pack(side="left", padx=2)
        tk.Label(af4, text="seg", bg=BG_CARD, font=("Arial", 8)).pack(side="left")

        # ═══════════════════════════════════════════════════════════════════
        # SECCIÓN: EJECUTAR
        # ═══════════════════════════════════════════════════════════════════
        card_exec = tk.Frame(side_panel, bg=BG_CARD, bd=1, relief="solid")
        card_exec.pack(fill="x", padx=4, pady=4)

        tk.Label(card_exec, text="  EJECUTAR MISIÓN", font=("Arial", 9, "bold"),
                 bg=BG_HEADER, fg=FG_HEADER, anchor="w").pack(fill="x", ipady=4)

        exec_content = tk.Frame(card_exec, bg=BG_CARD)
        exec_content.pack(fill="x", padx=8, pady=8)

        # Opciones de misión
        self._mission_return_home_var = tk.BooleanVar(value=True)
        tk.Checkbutton(exec_content, text="🏠 Volver a casa (0,0) al final",
                       variable=self._mission_return_home_var, bg=BG_CARD,
                       font=("Arial", 8), activebackground=BG_CARD).pack(anchor="w", pady=2)

        self._mission_auto_avoid_var = tk.BooleanVar(value=True)
        tk.Checkbutton(exec_content, text="🚧 Evitar obstáculos automáticamente",
                       variable=self._mission_auto_avoid_var, bg=BG_CARD,
                       font=("Arial", 8), activebackground=BG_CARD).pack(anchor="w", pady=2)

        self._mission_status_label = tk.Label(exec_content, text="Estado: Listo",
                                               bg=BG_CARD, font=("Arial", 8), fg="#333")
        self._mission_status_label.pack(anchor="w", pady=2)

        exec_btns = tk.Frame(exec_content, bg=BG_CARD)
        exec_btns.pack(fill="x", pady=4)

        tk.Button(exec_btns, text="🔍 Validar", command=self._validate_mission_only,
                  bg="#17a2b8", fg="white", font=("Arial", 10, "bold"), bd=0).pack(side="left", padx=2)

        self._mission_exec_btn = tk.Button(exec_btns, text="▶ Ejecutar", command=self._execute_mission,
                                            bg="#28a745", fg="white", font=("Arial", 10, "bold"), bd=0)
        self._mission_exec_btn.pack(side="left", fill="x", expand=True, padx=2)

        tk.Button(exec_btns, text="⏹ Abortar", command=self._abort_mission,
                  bg="#dc3545", fg="white", font=("Arial", 10, "bold"), bd=0).pack(side="left", fill="x", expand=True, padx=2)

        # Dibujar mapa inicial
        self._draw_mission_map()

    def _mission_world_to_canvas(self, wx, wy):
        """Convierte coordenadas mundo a canvas (misión).
        Sistema: +X mundo = arriba en pantalla (forward), +Y mundo = derecha (right)
        """
        mid = MAP_SIZE_PX / 2.0
        cx = mid + (wy * PX_PER_CM)   # Y mundo → X canvas (derecha)
        cy = mid - (wx * PX_PER_CM)   # X mundo → Y canvas (arriba)
        return cx, cy

    def _mission_canvas_to_world(self, cx, cy):
        """Convierte coordenadas canvas a mundo (misión).
        Sistema: arriba en pantalla = +X mundo, derecha = +Y mundo
        """
        mid = MAP_SIZE_PX / 2.0
        wy = (cx - mid) / PX_PER_CM   # X canvas → Y mundo
        wx = (mid - cy) / PX_PER_CM   # Y canvas → X mundo
        return wx, wy

    def _draw_mission_map(self):
        """Dibuja el mapa de misiones."""
        if not hasattr(self, '_mission_canvas') or not self._mission_canvas:
            return

        self._mission_canvas.delete("all")

        # Grid (líneas cada GRID_STEP_CM)
        center = MAP_SIZE_PX // 2
        for i in range(-400, 401, GRID_STEP_CM):
            # Líneas verticales (X mundo constante = línea horizontal en canvas debido al swap)
            _, cy = self._mission_world_to_canvas(i, 0)
            self._mission_canvas.create_line(0, cy, MAP_SIZE_PX, cy, fill=MAP_AXES_COLOR, width=1)
            # Líneas horizontales (Y mundo constante = línea vertical en canvas debido al swap)
            cx, _ = self._mission_world_to_canvas(0, i)
            self._mission_canvas.create_line(cx, 0, cx, MAP_SIZE_PX, fill=MAP_AXES_COLOR, width=1)

        # Ejes principales
        self._mission_canvas.create_line(center, 0, center, MAP_SIZE_PX, fill="#999", width=2)
        self._mission_canvas.create_line(0, center, MAP_SIZE_PX, center, fill="#999", width=2)

        # Origen
        self._mission_canvas.create_oval(center - 5, center - 5, center + 5, center + 5,
                                          fill="#333", outline="white", width=2)
        self._mission_canvas.create_text(center + 15, center + 15, text="(0,0)", font=("Arial", 8), fill="#666")

        # Dibujar geofence
        if self._mission_geofence:
            gf = self._mission_geofence
            x1, y1 = self._mission_world_to_canvas(gf['x1'], gf['y1'])
            x2, y2 = self._mission_world_to_canvas(gf['x2'], gf['y2'])
            self._mission_canvas.create_rectangle(x1, y1, x2, y2,
                                                   outline="#28a745", fill="", width=3, dash=(10, 5))

        # Dibujar obstáculos
        for i, obs in enumerate(self._mission_exclusions):
            obs_type = obs.get('type', 'circle')
            # Color diferente si está seleccionado
            is_selected = (i == self._selected_obs_idx)
            outline_color = "#ffc107" if is_selected else "#dc3545"  # Amarillo si seleccionado
            fill_color = "#fff3cd" if is_selected else "#ffcccc"
            line_width = 4 if is_selected else 2

            if obs_type == 'circle':
                cx, cy = self._mission_world_to_canvas(obs['cx'], obs['cy'])
                r = obs['r'] * PX_PER_CM
                self._mission_canvas.create_oval(cx - r, cy - r, cx + r, cy + r,
                                                  outline=outline_color, fill=fill_color, width=line_width)
            elif obs_type == 'rect':
                x1, y1 = self._mission_world_to_canvas(obs['x1'], obs['y1'])
                x2, y2 = self._mission_world_to_canvas(obs['x2'], obs['y2'])
                self._mission_canvas.create_rectangle(x1, y1, x2, y2,
                                                       outline=outline_color, fill=fill_color, width=line_width)
            elif obs_type == 'poly':
                points = []
                for px, py in obs['points']:
                    cx, cy = self._mission_world_to_canvas(px, py)
                    points.extend([cx, cy])
                if len(points) >= 6:
                    self._mission_canvas.create_polygon(points, outline=outline_color,
                                                         fill=fill_color, width=line_width)

        # Dibujar puntos temporales de rectángulo
        for px, py in self._mission_rect_points:
            cx, cy = self._mission_world_to_canvas(px, py)
            self._mission_canvas.create_oval(cx - 5, cy - 5, cx + 5, cy + 5,
                                              fill="#ff6600", outline="#cc5500", width=2)

        # Dibujar puntos temporales de polígono
        if self._mission_poly_points:
            points = []
            for px, py in self._mission_poly_points:
                cx, cy = self._mission_world_to_canvas(px, py)
                points.extend([cx, cy])
                self._mission_canvas.create_oval(cx - 4, cy - 4, cx + 4, cy + 4,
                                                  fill="#ff6600", outline="#cc5500", width=2)
            if len(points) >= 4:
                self._mission_canvas.create_line(points, fill="#ff6600", width=2, dash=(4, 4))

        # Dibujar waypoints y ruta
        if len(self._mission_waypoints) > 0:
            # Líneas de ruta
            points = [(MAP_SIZE_PX // 2, MAP_SIZE_PX // 2)]  # Origen
            for wp in self._mission_waypoints:
                cx, cy = self._mission_world_to_canvas(wp['x'], wp['y'])
                points.append((cx, cy))

            for i in range(len(points) - 1):
                self._mission_canvas.create_line(points[i][0], points[i][1],
                                                  points[i + 1][0], points[i + 1][1],
                                                  fill="#17a2b8", width=2, arrow=tk.LAST)

            # Waypoints
            for i, wp in enumerate(self._mission_waypoints):
                cx, cy = self._mission_world_to_canvas(wp['x'], wp['y'])
                color = "#ffc107" if i == self._wp_selected_idx else "#17a2b8"
                self._mission_canvas.create_oval(cx - 10, cy - 10, cx + 10, cy + 10,
                                                  fill=color, outline="white", width=2)
                self._mission_canvas.create_text(cx, cy, text=str(i + 1),
                                                  font=("Arial", 9, "bold"), fill="white")

                # Iconos de acciones
                icons = []
                if wp.get('photo'):
                    icons.append("📷")
                if wp.get('video'):
                    icons.append("🎬")
                if wp.get('rotate'):
                    icons.append("🔄")
                if wp.get('wait'):
                    icons.append("⏱")
                if icons:
                    self._mission_canvas.create_text(cx, cy + 18, text="".join(icons),
                                                      font=("Arial", 7))

        # Dibujar dron (se actualiza en tiempo real)
        self._draw_mission_drone_marker()

    def _draw_mission_drone_marker(self):
        """Dibuja el marcador del dron en el mapa de misiones."""
        if not hasattr(self, '_mission_canvas') or not self._mission_canvas:
            return

        # Eliminar dron anterior
        self._mission_canvas.delete("drone")

        pose = getattr(self.dron, "pose", None)
        if not pose:
            return

        x = getattr(pose, "x_cm", None)
        y = getattr(pose, "y_cm", None)
        yaw = getattr(pose, "yaw_deg", None)

        if x is None or y is None:
            return

        cx_px, cy_px = self._mission_world_to_canvas(x, y)
        rad = 10

        # Círculo del dron
        self._mission_canvas.create_oval(
            cx_px - rad, cy_px - rad,
            cx_px + rad, cy_px + rad,
            fill=MAP_DRONE_COLOR, outline="white", width=2, tags="drone"
        )

        # Flecha de yaw (roja) - indica hacia dónde apunta el dron
        if yaw is not None:
            th = math.radians(yaw)
            arrow_len = 25
            # Sistema de coordenadas: yaw=0 → forward=+X mundo = arriba en pantalla
            dx_world = arrow_len * math.cos(th)  # X en mundo (forward cuando yaw=0)
            dy_world = arrow_len * math.sin(th)  # Y en mundo (right cuando yaw=0)
            # Conversión a canvas: +X mundo = arriba (-Y canvas), +Y mundo = derecha (+X canvas)
            dx_canvas = dy_world * PX_PER_CM
            dy_canvas = -dx_world * PX_PER_CM

            self._mission_canvas.create_line(
                cx_px, cy_px,
                cx_px + dx_canvas, cy_px + dy_canvas,
                fill="red", width=3, arrow=tk.LAST, tags="drone"
            )

        # Flecha de velocidad/movimiento (negra) - indica hacia dónde se mueve el dron
        vx = getattr(pose, "vx", None) or getattr(self.dron, "vx_cm_s", None)
        vy = getattr(pose, "vy", None) or getattr(self.dron, "vy_cm_s", None)
        if vx is not None and vy is not None:
            speed = math.sqrt(vx**2 + vy**2)
            if speed > 5:
                arrow_len_vel = 40
                # vx, vy están en coordenadas mundo (+X arriba, +Y derecha)
                dx_vel = (vy / speed) * arrow_len_vel * PX_PER_CM   # Y mundo → X canvas
                dy_vel = -(vx / speed) * arrow_len_vel * PX_PER_CM  # X mundo → -Y canvas

                self._mission_canvas.create_line(
                    cx_px, cy_px,
                    cx_px + dx_vel, cy_px + dy_vel,
                    fill="black", width=3, arrow=tk.LAST, tags="drone"
                )

    def _update_mission_drone(self):
        """Actualiza la posición del dron en el mapa de misiones."""
        try:
            if not hasattr(self, '_mission_win') or not self._mission_win:
                return
            if not hasattr(self, '_mission_canvas') or not self._mission_canvas:
                return
            if not tk.Toplevel.winfo_exists(self._mission_win):
                return

            self._draw_mission_drone_marker()
        except Exception:
            pass

    def _on_mission_canvas_click(self, event):
        """Maneja clicks en el canvas de misiones."""
        wx, wy = self._mission_canvas_to_world(event.x, event.y)
        tool = self._mission_tool.get()

        if tool == "waypoint":
            # Validar que está dentro del geofence
            if self._mission_geofence:
                gf = self._mission_geofence
                if not (gf['x1'] <= wx <= gf['x2'] and gf['y1'] <= wy <= gf['y2']):
                    messagebox.showwarning("Fuera de zona", "El waypoint está fuera del geofence")
                    return

            # Validar que no está en un obstáculo (usa módulo tello_geometry)
            for obs in self._mission_exclusions:
                if point_in_obstacle(wx, wy, obs):
                    messagebox.showwarning("Obstáculo", "El waypoint está dentro de un obstáculo")
                    return

            # Añadir waypoint
            z = self._wp_default_z.get()
            wp = {
                'x': round(wx, 1),
                'y': round(wy, 1),
                'z': z,
                'photo': False,
                'video': False,
                'video_duration': 5,
                'rotate': False,
                'rotate_deg': 0,
                'wait': False,
                'wait_sec': 0
            }
            self._mission_waypoints.append(wp)
            self._update_wp_listbox()
            self._draw_mission_map()

        elif tool == "obstacle":
            r = self._mission_obs_radius.get()
            obs = {'type': 'circle', 'cx': round(wx, 1), 'cy': round(wy, 1), 'r': r}
            self._mission_exclusions.append(obs)
            self._draw_mission_map()

        elif tool == "obs_rect":
            self._mission_rect_points.append((round(wx, 1), round(wy, 1)))
            if len(self._mission_rect_points) == 2:
                p1, p2 = self._mission_rect_points
                obs = {
                    'type': 'rect',
                    'x1': min(p1[0], p2[0]), 'y1': min(p1[1], p2[1]),
                    'x2': max(p1[0], p2[0]), 'y2': max(p1[1], p2[1])
                }
                self._mission_exclusions.append(obs)
                self._mission_rect_points.clear()
            self._draw_mission_map()

        elif tool == "obs_poly":
            self._mission_poly_points.append((round(wx, 1), round(wy, 1)))
            self._draw_mission_map()

        elif tool == "select_obs":
            # Buscar si el clic está dentro de algún obstáculo
            for i, obs in enumerate(self._mission_exclusions):
                if point_in_obstacle(wx, wy, obs):
                    self._selected_obs_idx = i
                    self._load_obs_to_edit(obs, i)
                    self._draw_mission_map()
                    return
            # No se encontró obstáculo
            self._selected_obs_idx = None
            self._obs_edit_label.configure(text="Clic en ✋ y luego en obstáculo")
            self._draw_mission_map()

    def _load_obs_to_edit(self, obs, idx):
        """Carga los datos del obstáculo en los campos de edición."""
        obs_type = obs.get('type', 'circle')

        self._obs_edit_x.delete(0, tk.END)
        self._obs_edit_y.delete(0, tk.END)
        self._obs_edit_size.delete(0, tk.END)

        if obs_type == 'circle':
            self._obs_edit_x.insert(0, str(int(obs['cx'])))
            self._obs_edit_y.insert(0, str(int(obs['cy'])))
            self._obs_edit_size.insert(0, str(int(obs['r'])))
            self._obs_edit_label.configure(text=f"Editando ⭕ #{idx+1} (cx, cy, radio)")
        elif obs_type == 'rect':
            # Para rectángulo, mostramos centro y ancho
            cx = (obs['x1'] + obs['x2']) / 2
            cy = (obs['y1'] + obs['y2']) / 2
            w = abs(obs['x2'] - obs['x1'])
            self._obs_edit_x.insert(0, str(int(cx)))
            self._obs_edit_y.insert(0, str(int(cy)))
            self._obs_edit_size.insert(0, str(int(w)))
            self._obs_edit_label.configure(text=f"Editando ⬜ #{idx+1} (cx, cy, ancho)")
        elif obs_type == 'poly':
            # Para polígono, solo mostramos posición del centroide
            pts = obs.get('points', [])
            if pts:
                cx = sum(p[0] for p in pts) / len(pts)
                cy = sum(p[1] for p in pts) / len(pts)
                self._obs_edit_x.insert(0, str(int(cx)))
                self._obs_edit_y.insert(0, str(int(cy)))
                self._obs_edit_size.insert(0, "—")
            self._obs_edit_label.configure(text=f"Editando ⬡ #{idx+1} (solo mover)")

    def _apply_obs_edit(self):
        """Aplica los cambios al obstáculo seleccionado."""
        if self._selected_obs_idx is None:
            return

        idx = self._selected_obs_idx
        if idx >= len(self._mission_exclusions):
            return

        obs = self._mission_exclusions[idx]
        obs_type = obs.get('type', 'circle')

        try:
            new_x = int(self._obs_edit_x.get())
            new_y = int(self._obs_edit_y.get())
            size_str = self._obs_edit_size.get()
            new_size = int(size_str) if size_str != "—" else None
        except ValueError:
            return

        if obs_type == 'circle':
            obs['cx'] = new_x
            obs['cy'] = new_y
            if new_size is not None:
                obs['r'] = new_size
        elif obs_type == 'rect':
            # Mover el rectángulo manteniendo su tamaño
            old_cx = (obs['x1'] + obs['x2']) / 2
            old_cy = (obs['y1'] + obs['y2']) / 2
            dx = new_x - old_cx
            dy = new_y - old_cy
            obs['x1'] += dx
            obs['x2'] += dx
            obs['y1'] += dy
            obs['y2'] += dy
            # Cambiar tamaño si se especificó
            if new_size is not None:
                half = new_size / 2
                obs['x1'] = new_x - half
                obs['x2'] = new_x + half
                h = abs(obs['y2'] - obs['y1']) / 2
                obs['y1'] = new_y - h
                obs['y2'] = new_y + h
        elif obs_type == 'poly':
            # Mover todos los puntos del polígono
            pts = obs.get('points', [])
            if pts:
                old_cx = sum(p[0] for p in pts) / len(pts)
                old_cy = sum(p[1] for p in pts) / len(pts)
                dx = new_x - old_cx
                dy = new_y - old_cy
                obs['points'] = [(p[0] + dx, p[1] + dy) for p in pts]

        self._draw_mission_map()
        print(f"[OBS] Obstáculo #{idx+1} actualizado")

    def _delete_selected_obs(self):
        """Elimina el obstáculo seleccionado."""
        if self._selected_obs_idx is None:
            return

        idx = self._selected_obs_idx
        if idx < len(self._mission_exclusions):
            del self._mission_exclusions[idx]
            self._selected_obs_idx = None
            self._obs_edit_label.configure(text="Clic en ✋ y luego en obstáculo")
            self._obs_edit_x.delete(0, tk.END)
            self._obs_edit_y.delete(0, tk.END)
            self._obs_edit_size.delete(0, tk.END)
            self._draw_mission_map()
            print(f"[OBS] Obstáculo #{idx+1} eliminado")

    def _close_mission_polygon(self):
        """Cierra el polígono de obstáculo actual."""
        if len(self._mission_poly_points) >= 3:
            obs = {'type': 'poly', 'points': list(self._mission_poly_points)}
            self._mission_exclusions.append(obs)
        self._mission_poly_points.clear()
        self._draw_mission_map()

    def _delete_last_obstacle(self):
        """Elimina el último obstáculo añadido."""
        if self._mission_exclusions:
            self._mission_exclusions.pop()
            self._draw_mission_map()

    def _mission_apply_geofence(self):
        """Aplica el geofence configurado."""
        self._mission_geofence = {
            'x1': min(self._mgf_x1.get(), self._mgf_x2.get()),
            'y1': min(self._mgf_y1.get(), self._mgf_y2.get()),
            'x2': max(self._mgf_x1.get(), self._mgf_x2.get()),
            'y2': max(self._mgf_y1.get(), self._mgf_y2.get()),
            'zmin': self._mgf_zmin.get(),
            'zmax': self._mgf_zmax.get()
        }
        self._draw_mission_map()

    def _update_wp_listbox(self):
        """Actualiza la lista de waypoints."""
        self._wp_listbox.delete(0, tk.END)
        for i, wp in enumerate(self._mission_waypoints):
            actions = []
            if wp.get('photo'):
                actions.append("📷")
            if wp.get('video'):
                actions.append("🎬")
            if wp.get('rotate'):
                actions.append(f"🔄{wp.get('rotate_deg', 0)}°")
            if wp.get('wait'):
                actions.append(f"⏱{wp.get('wait_sec', 0)}s")
            action_str = " ".join(actions) if actions else ""
            self._wp_listbox.insert(tk.END, f"WP{i + 1}: ({wp['x']}, {wp['y']}, {wp['z']}) {action_str}")

    def _on_wp_select(self, event):
        """Maneja selección de waypoint en la lista."""
        selection = self._wp_listbox.curselection()
        if not selection:
            # NO resetear _wp_selected_idx aquí, puede ser pérdida temporal de foco
            return

        idx = selection[0]
        self._wp_selected_idx = idx
        wp = self._mission_waypoints[idx]

        # Actualizar coordenadas editables
        self._wp_edit_x_entry.delete(0, tk.END)
        self._wp_edit_x_entry.insert(0, str(int(wp.get('x', 0))))
        self._wp_edit_y_entry.delete(0, tk.END)
        self._wp_edit_y_entry.insert(0, str(int(wp.get('y', 0))))
        self._wp_edit_z_entry.delete(0, tk.END)
        self._wp_edit_z_entry.insert(0, str(int(wp.get('z', 100))))

        # Actualizar checkboxes
        self._action_photo.set(wp.get('photo', False))
        self._action_video.set(wp.get('video', False))
        self._action_video_duration.set(wp.get('video_duration', 5))
        self._action_rotate.set(wp.get('rotate', False))
        self._action_rotate_deg.set(wp.get('rotate_deg', 90))
        self._action_wait.set(wp.get('wait', False))
        self._action_wait_sec.set(wp.get('wait_sec', 2))

        self._wp_action_label.configure(text=f"Editando WP{idx + 1}")
        self._draw_mission_map()

    def _update_wp_actions(self):
        """Actualiza las acciones del waypoint seleccionado."""
        if self._wp_selected_idx is None:
            return

        wp = self._mission_waypoints[self._wp_selected_idx]
        wp['photo'] = self._action_photo.get()
        wp['video'] = self._action_video.get()
        wp['video_duration'] = self._action_video_duration.get()
        wp['rotate'] = self._action_rotate.get()
        wp['rotate_deg'] = self._action_rotate_deg.get()
        wp['wait'] = self._action_wait.get()
        wp['wait_sec'] = self._action_wait_sec.get()

        self._update_wp_listbox()
        self._draw_mission_map()

    def _apply_wp_coords(self):
        """Aplica las coordenadas editadas al waypoint seleccionado."""
        if self._wp_selected_idx is None:
            print("[WP] No hay waypoint seleccionado")
            return

        idx = self._wp_selected_idx

        try:
            new_x = int(self._wp_edit_x_entry.get())
            new_y = int(self._wp_edit_y_entry.get())
            new_z = int(self._wp_edit_z_entry.get())
        except ValueError as e:
            print(f"[WP] Error convirtiendo valores: {e}")
            return

        if idx >= len(self._mission_waypoints):
            print(f"[WP] Índice {idx} fuera de rango")
            return

        wp = self._mission_waypoints[idx]
        print(f"[WP] Actualizando WP{idx+1}: ({wp['x']},{wp['y']},{wp['z']}) -> ({new_x},{new_y},{new_z})")
        wp['x'] = new_x
        wp['y'] = new_y
        wp['z'] = new_z

        self._update_wp_listbox()
        self._draw_mission_map()
        self._wp_selected_idx = idx
        self._wp_listbox.selection_set(idx)
        print(f"[WP] Guardado OK")

    def _delete_selected_wp(self):
        """Elimina el waypoint seleccionado."""
        if self._wp_selected_idx is not None:
            del self._mission_waypoints[self._wp_selected_idx]
            self._wp_selected_idx = None
            self._update_wp_listbox()
            self._draw_mission_map()

    def _move_wp(self, direction):
        """Mueve el waypoint seleccionado arriba/abajo."""
        if self._wp_selected_idx is None:
            return
        idx = self._wp_selected_idx
        new_idx = idx + direction
        if 0 <= new_idx < len(self._mission_waypoints):
            self._mission_waypoints[idx], self._mission_waypoints[new_idx] = \
                self._mission_waypoints[new_idx], self._mission_waypoints[idx]
            self._wp_selected_idx = new_idx
            self._update_wp_listbox()
            self._wp_listbox.selection_set(new_idx)
            self._draw_mission_map()

    def _clear_all_waypoints(self):
        """Elimina todos los waypoints."""
        if messagebox.askyesno("Confirmar", "¿Eliminar todos los waypoints?"):
            self._mission_waypoints.clear()
            self._wp_selected_idx = None
            self._update_wp_listbox()
            self._draw_mission_map()

    def _validate_mission_only(self):
        """Valida la misión sin ejecutarla (no requiere dron conectado)."""
        if not self._mission_waypoints:
            messagebox.showwarning("Sin waypoints", "Añade al menos un waypoint", parent=self._mission_win)
            return

        return_home = self._mission_return_home_var.get() if hasattr(self, '_mission_return_home_var') else False
        auto_avoid = self._mission_auto_avoid_var.get() if hasattr(self, '_mission_auto_avoid_var') else True

        # Validar rutas contra obstáculos
        try:
            valid, error = validate_mission_paths(self._mission_waypoints, self._mission_exclusions, return_home)
        except Exception as e:
            self._mission_status_label.configure(text=f"❌ Error: {e}", fg="#dc3545")
            return

        if valid:
            # Ruta válida sin obstáculos en el camino
            msg = f"✅ Ruta válida ({len(self._mission_waypoints)} WPs)"
            self._mission_status_label.configure(text=msg, fg="#28a745")
            self._mission_win.after(50, lambda: messagebox.showinfo("Validación", msg, parent=self._mission_win))
        elif auto_avoid and self._mission_exclusions:
            # Ruta inválida pero auto_avoid está activado - path planning
            try:
                planned_wps = plan_mission_with_avoidance(
                    self._mission_waypoints, self._mission_exclusions, return_home
                )
                n_original = len(self._mission_waypoints)
                n_total = len(planned_wps)
                n_intermediate = sum(1 for wp in planned_wps if wp.get('_intermediate', False))

                msg = f"🔄 {n_original} WPs + {n_intermediate} intermedios = {n_total}"
                self._mission_status_label.configure(text=msg, fg="#17a2b8")
                detail = f"Ruta atraviesa obstáculos.\n\nWPs originales: {n_original}\nWPs intermedios: {n_intermediate}\nTotal: {n_total}"
                self._mission_win.after(50, lambda d=detail: messagebox.showinfo("Path Planning", d, parent=self._mission_win))
            except Exception as e:
                msg = f"❌ Error: {e}"
                self._mission_status_label.configure(text=msg, fg="#dc3545")
        else:
            # Ruta inválida y auto_avoid desactivado
            msg = f"❌ {error}"
            self._mission_status_label.configure(text=msg, fg="#dc3545")
            detail = f"{error}\n\nActiva 'Evitar obstáculos' o mueve waypoints"
            self._mission_win.after(50, lambda d=detail: messagebox.showerror("Ruta Inválida", d, parent=self._mission_win))

    def _execute_mission(self):
        """Ejecuta la misión planificada."""
        if not self._mission_waypoints:
            messagebox.showwarning("Sin waypoints", "Añade al menos un waypoint")
            return

        if self.dron.state != "connected" and self.dron.state != "flying":
            messagebox.showwarning("No conectado", "Conecta el dron primero")
            return

        if self._mission_running:
            messagebox.showinfo("En ejecución", "Ya hay una misión en ejecución")
            return

        # Obtener opciones
        return_home = self._mission_return_home_var.get() if hasattr(self, '_mission_return_home_var') else False
        auto_avoid = self._mission_auto_avoid_var.get() if hasattr(self, '_mission_auto_avoid_var') else True

        # Validar rutas contra obstáculos (usa módulo tello_geometry)
        valid, error = validate_mission_paths(self._mission_waypoints, self._mission_exclusions, return_home)

        # Si hay obstáculos y auto_avoid está activado, planificar ruta alternativa
        if not valid and auto_avoid and self._mission_exclusions:
            # Usar path planning para evitar obstáculos
            planned_wps = plan_mission_with_avoidance(
                self._mission_waypoints, self._mission_exclusions, return_home
            )
            # Contar waypoints intermedios añadidos
            n_intermediate = sum(1 for wp in planned_wps if wp.get('_intermediate', False))
            if n_intermediate > 0:
                messagebox.showinfo("Path Planning",
                    f"Se han añadido {n_intermediate} waypoints intermedios\n"
                    f"para evitar obstáculos automáticamente.")
            waypoints_to_execute = planned_wps
        elif not valid:
            messagebox.showerror("Ruta inválida",
                f"⚠️ {error}\n\nReorganiza los waypoints o activa 'Evitar obstáculos automáticamente'.")
            return
        else:
            waypoints_to_execute = self._mission_waypoints

        self._mission_running = True
        self._mission_status_label.configure(text="Estado: Ejecutando...", fg="#ffc107")
        self._mission_exec_btn.configure(state="disabled")

        total_wps = len(waypoints_to_execute)

        # Callbacks solo para actualizar UI (la lógica está en el módulo)
        def on_wp_arrived(idx, wp_data):
            """Callback al llegar a cada waypoint (idx 0-based)."""
            self._mission_win.after(0, lambda i=idx: self._mission_status_label.configure(
                text=f"Estado: WP{i + 1}/{total_wps}", fg="#17a2b8"))

        def on_action(idx, action_name):
            """Callback al ejecutar una acción."""
            action_texts = {'rotate': '🔄 Rotando', 'photo': '📷 Foto', 'video': '🎥 Video', 'wait': '⏳ Esperando'}
            text = action_texts.get(action_name, action_name)
            self._mission_win.after(0, lambda: self._mission_status_label.configure(
                text=f"WP{idx + 1}: {text}", fg="#17a2b8"))
            # Manejar video con la grabación de la app
            if action_name == 'video':
                wp = self._mission_waypoints[idx]
                duration = wp.get('video_duration', 0)
                if duration > 0:
                    self._start_recording()
                    # El módulo ya hace el sleep, al terminar paramos
                    self._mission_win.after(int(duration * 1000), self._stop_recording)

        def on_finish():
            """Callback al terminar la misión."""
            self._mission_running = False
            self._mission_win.after(0, lambda: self._mission_status_label.configure(
                text="Estado: Completada ✓", fg="#28a745"))
            self._mission_win.after(0, lambda: self._mission_exec_btn.configure(state="normal"))

        # Timer para actualizar el mapa en tiempo real durante la misión
        def update_map_timer():
            if self._mission_running and hasattr(self, '_mission_win') and self._mission_win:
                try:
                    self._update_mission_drone()
                    self._mission_win.after(100, update_map_timer)  # Actualizar cada 100ms
                except:
                    pass

        # Iniciar el timer de actualización del mapa
        self._mission_win.after(100, update_map_timer)

        # Si usamos path planning, return_home ya está incluido en los waypoints
        use_return_home = return_home if waypoints_to_execute is self._mission_waypoints else False

        # Ejecutar misión (el módulo maneja todo: navegación, acciones, return_home)
        def run_in_thread():
            try:
                self.dron.run_mission(
                    waypoints=waypoints_to_execute,
                    do_land=True,
                    return_home=use_return_home,
                    blocking=True,
                    on_wp_arrived=on_wp_arrived,
                    on_action=on_action,
                    on_finish=on_finish
                )
            except Exception as e:
                self._mission_running = False
                self._mission_win.after(0, lambda: self._mission_status_label.configure(
                    text=f"Estado: Error - {e}", fg="#dc3545"))
                self._mission_win.after(0, lambda: self._mission_exec_btn.configure(state="normal"))

        threading.Thread(target=run_in_thread, daemon=True).start()

    def _abort_mission(self):
        """Aborta la misión en ejecución."""
        if self._mission_running:
            self.dron.abort_mission()
            self._mission_running = False
            self._mission_status_label.configure(text="Estado: Abortada", fg="#dc3545")
            self._mission_exec_btn.configure(state="normal")


#MAIN
if __name__ == "__main__":
    root = tk.Tk()
    app = MiniRemoteApp(root)
    root.mainloop()