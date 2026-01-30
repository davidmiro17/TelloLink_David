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
from tkinter import messagebox, filedialog, ttk, simpledialog
import threading
import time
import sys
import math
import json
import os
import datetime
import shutil

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
from TelloLink.modules.tello_scenario import ScenarioManager, get_scenario_manager
from TelloLink.modules.tello_geometry import (
    point_in_obstacle, validate_mission_paths
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

# Límites de seguridad para misiones
MISSION_MAX_HEIGHT_CM = 300    # Altura máxima permitida (3 metros)
MISSION_MIN_HEIGHT_CM = 30     # Altura mínima permitida (30 cm)
MISSION_MAX_DISTANCE_CM = 1000 # Distancia máxima desde origen (10 metros)


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
        self.temp_var = tk.StringVar(value="—")

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
        self._mission_exclusions = []  # Sincronizado con _excl_circles y _excl_polys
        self._draw_layer_var = None

        # Sistema de capas
        self._current_layer = 0
        self._last_layer = 0
        self._layer_label = None
        self._mission_layer_label = None
        self._mission_draw_layer_var = None
        self._layer1_min_var = None
        self._layer1_max_var = None
        self._layer2_min_var = None
        self._layer2_max_var = None
        self._layer3_min_var = None
        self._layer3_max_var = None

        # Sistema de escenarios
        base_dir = os.path.dirname(os.path.abspath(__file__))
        repo_root = os.path.dirname(base_dir)
        escenarios_dir = os.path.join(base_dir, "escenarios")
        self._scenario_manager = get_scenario_manager(escenarios_dir)
        self._current_scenario_id = None
        self._scenario_combo = None
        self._migrate_scenarios(os.path.join(repo_root, "escenarios"), escenarios_dir)

        # FPV
        self.fpv_label = None
        self._fpv_running = False
        self._fpv_thread = None
        self._frame_lock = threading.Lock()
        self._cv_cap_lock = threading.Lock()
        self._last_bgr = None
        self._last_frame_time = 0.0
        self._want_stream_on = False
        self._rec_running = False
        self._rec_writer = None
        self._rec_path = None
        self._rec_fps = 30
        self._rec_size = (640, 480)
        self._rec_frame_count = 0
        self._rec_expected_frames = None
        self._rec_last_good_frame = None
        self._rec_force_direct = False
        self._fpv_record_enabled = False
        # FPV externo (ventana OpenCV)
        self._fpv_ext_running = False
        self._fpv_ext_thread = None
        self._fpv_ext_cap = None
        # Directorios legacy (para migración)
        self._legacy_shots_dir = os.path.join(base_dir, "captures")
        self._legacy_recs_dir = os.path.join(base_dir, "videos")

        # Gestor de sesiones
        self._sessions_dir = os.path.join(base_dir, "sesiones")
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
        tk.Label(top, text="Temp:").grid(row=0, column=8, sticky="e")
        tk.Label(top, textvariable=self.temp_var, width=6).grid(row=0, column=9, sticky="w")

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
        self._btn_takeoff = tk.Button(flight, text="Despegar (Enter)", width=16, command=self.on_takeoff,
                  bg="#90ee90", font=("Arial", 10, "bold"))
        self._btn_takeoff.grid(row=0, column=0, **pad)
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
        tk.Button(row4, text="🎮", command=self._reconnect_joystick,
                  bg="#1a1a1a", fg="#6c757d", font=("Arial", 9), bd=0,
                  activebackground="#333", padx=4, pady=0).pack(side="right")

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
        tk.Button(btn_container, text="🗑 Reset Total", command=self._emergency_reset,
                  bg="#dc3545", fg="white", font=("Arial", 10, "bold"), padx=15, pady=5).pack(side="left", padx=4)

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
            self.temp_var.set("—")
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

        # Callback que se ejecuta cuando el despegue termina
        def on_takeoff_complete(success):
            # Usamos root.after para actualizar la UI desde el hilo principal
            def update_ui():
                if success:
                    self._btn_takeoff.configure(bg="#28a745")  # Verde: despegue completado
                    self._ensure_pose_origin()
                    self._restart_gf_monitor(force=True)
                    try:
                        self._reapply_exclusions_to_backend()
                    except Exception:
                        pass
                    self._hud_show("Despegado", 1.5)
                else:
                    self._btn_takeoff.configure(bg="#90ee90")  # Restaurar color original
                    messagebox.showerror("TakeOff", "No se pudo despegar.")
                self._resume_keepalive()
            self.root.after(0, update_ui)

        try:
            self._pause_keepalive()
            # Cambiar botón a amarillo mientras despega
            self._btn_takeoff.configure(bg="#ffcc00")
            # Llamada no bloqueante con callback
            self.dron.takeOff(0.5, blocking=False, callback=on_takeoff_complete)
        except Exception as e:
            self._btn_takeoff.configure(bg="#90ee90")  # Restaurar color
            messagebox.showerror("TakeOff", str(e))
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
                self._hud_show("Ya en suelo", 1.0)
                self.root.after(500, lambda: setattr(self, "_ui_landing", False))
                return
            if st not in ("flying", "hovering"):
                self._hud_show(f"Estado: {st}", 1.0)
                self.root.after(500, lambda: setattr(self, "_ui_landing", False))
                return
            self._pause_keepalive()
            self.dron.Land(blocking=False)
            self._hud_show("Aterrizando", 1.5)
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

    def _emergency_reset(self):
        """Borra todos los medios, planes de vuelo y escenarios (reset de emergencia)."""
        # Primera confirmación
        resp1 = messagebox.askyesno(
            "Reset Total",
            "¿Estás seguro de que quieres BORRAR TODO?\n\n"
            "- Todas las fotos y videos\n"
            "- Todos los planes de vuelo\n"
            "- Todos los escenarios\n\n"
            "Esta acción NO se puede deshacer.",
            icon="warning"
        )
        if not resp1:
            return

        # Segunda confirmación (doble seguridad)
        resp2 = messagebox.askyesno(
            "Confirmar Reset Total",
            "ÚLTIMA ADVERTENCIA\n\n"
            "Todos los datos serán eliminados permanentemente.\n\n"
            "¿Confirmas el borrado total?",
            icon="warning"
        )
        if not resp2:
            return

        deleted_sessions = 0
        deleted_scenarios = 0
        errors = []

        # Borrar sesiones (fotos y videos)
        try:
            if os.path.exists(self._sessions_dir):
                for item in os.listdir(self._sessions_dir):
                    item_path = os.path.join(self._sessions_dir, item)
                    try:
                        if os.path.isdir(item_path):
                            shutil.rmtree(item_path)
                        else:
                            os.remove(item_path)
                        deleted_sessions += 1
                    except Exception as e:
                        errors.append(f"Sesión {item}: {e}")
        except Exception as e:
            errors.append(f"Error accediendo sesiones: {e}")

        # Borrar escenarios (incluye planes de vuelo)
        try:
            escenarios_dir = self._scenario_manager.base_dir
            if os.path.exists(escenarios_dir):
                for item in os.listdir(escenarios_dir):
                    if item.endswith('.json'):
                        item_path = os.path.join(escenarios_dir, item)
                        try:
                            os.remove(item_path)
                            deleted_scenarios += 1
                        except Exception as e:
                            errors.append(f"Escenario {item}: {e}")
        except Exception as e:
            errors.append(f"Error accediendo escenarios: {e}")

        # Mostrar resultado
        if errors:
            messagebox.showwarning(
                "Reset Parcial",
                f"Reset completado con errores:\n"
                f"- Sesiones borradas: {deleted_sessions}\n"
                f"- Escenarios borrados: {deleted_scenarios}\n\n"
                f"Errores:\n" + "\n".join(errors[:5])
            )
        else:
            messagebox.showinfo(
                "Reset Completado",
                f"Todos los datos han sido eliminados:\n"
                f"- Sesiones borradas: {deleted_sessions}\n"
                f"- Escenarios borrados: {deleted_scenarios}"
            )

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

                    if getattr(self, "_mission_running", False):
                        time.sleep(0.1)
                        continue

                    if getattr(self.dron, "_goto_in_progress", False):
                        time.sleep(0.1)
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
            snr = getattr(self.dron, "wifi", None)
            temp = getattr(self.dron, "temp_c", None)
            st = getattr(self.dron, "state", "disconnected")
            self.state_var.set(st)
            self.bat_var.set(f"{bat}%" if isinstance(bat, int) else "—")
            self.h_var.set(f"{h} cm" if isinstance(h, (int, float)) else "0 cm")
            self.wifi_var.set(f"{snr}" if isinstance(snr, int) else "—")
            self.temp_var.set(f"{int(temp)}°C" if temp is not None else "—")

            pose = getattr(self.dron, "pose", None)
            if (pose is not None and h is not None and isinstance(h, (int, float))
                    and not getattr(self, "_mission_running", False)
                    and not getattr(self.dron, "_goto_in_progress", False)):
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

            # Activar geofence con formato nuevo (coordenadas absolutas)
            setattr(self.dron, "_gf_enabled", True)
            setattr(self.dron, "_gf_center", (cx, cy))
            setattr(self.dron, "_gf_limits", {"x1": x1, "y1": y1, "x2": x2, "y2": y2, "zmin": zmin, "zmax": zmax})
            setattr(self.dron, "_gf_mode", mode)

            # Guardar el rectángulo de inclusión para dibujarlo (coordenadas absolutas)
            self._incl_rect = (x1, y1, x2, y2)

            # Sincronizar también con _mission_geofence para el Editor de Misiones
            self._mission_geofence = {
                'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2,
                'zmin': zmin, 'zmax': zmax
            }

            self._reapply_exclusions_to_backend()
            self._hud_show(f"Geofence {mode} activado", 1.5)
            if self._map_win and tk.Toplevel.winfo_exists(self._map_win):
                self._redraw_map_static()
            # Redibujar también el editor de misiones si está abierto
            if hasattr(self, '_mission_canvas') and self._mission_canvas and self._mission_canvas.winfo_exists():
                self._draw_mission_map()
        except Exception as e:
            messagebox.showerror("Geofence", f"Error: {e}")

    def on_gf_disable(self):
        try:
            if hasattr(self.dron, "disable_geofence"):
                self.dron.disable_geofence()
            setattr(self.dron, "_gf_enabled", False)
            self._hud_show("Geofence desactivado", 1.5)
            if self._map_win and tk.Toplevel.winfo_exists(self._map_win):
                self._redraw_map_static()
            if hasattr(self, '_mission_canvas') and self._mission_canvas and self._mission_canvas.winfo_exists():
                self._draw_mission_map()
        except Exception as e:
            messagebox.showerror("Geofence", f"Error: {e}")

    def _clear_geofence(self):
        """Limpia la zona de geofence dibujada."""
        try:
            # Limpiar variables
            self.gf_x1_var.set("-100")
            self.gf_y1_var.set("-100")
            self.gf_x2_var.set("100")
            self.gf_y2_var.set("100")
            self._incl_rect = None
            self._mission_geofence = None

            # Desactivar geofence
            if hasattr(self.dron, "disable_geofence"):
                self.dron.disable_geofence()
            else:
                setattr(self.dron, "_gf_enabled", False)
                setattr(self.dron, "_gf_limits", {})

            # Limpiar puntos temporales
            if hasattr(self, '_gf_pts'):
                self._gf_pts.clear()
            if hasattr(self, '_mission_gf_pts'):
                self._mission_gf_pts.clear()

            # Limpiar canvas del mapa si existe
            if hasattr(self, 'map_canvas') and self.map_canvas:
                try:
                    self.map_canvas.delete("gf_temp")
                    self.map_canvas.delete("inclusion")
                except Exception:
                    pass

            # Limpiar canvas del editor si existe
            if hasattr(self, '_mission_canvas') and self._mission_canvas:
                try:
                    self._mission_canvas.delete("gf_temp")
                except Exception:
                    pass

            self._hud_show("Geofence limpiado", 1.5)

            # Redibujar mapa si está abierto
            try:
                if self._map_win and self._map_win.winfo_exists():
                    self._redraw_map_static()
            except Exception:
                pass

            # Redibujar editor si está abierto
            try:
                if hasattr(self, '_mission_canvas') and self._mission_canvas and self._mission_canvas.winfo_exists():
                    self._draw_mission_map()
            except Exception:
                pass
        except Exception as e:
            print(f"[clear_geofence] Error: {e}")

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

    def _migrate_scenarios(self, source_dir, target_dir):
        """Migra escenarios desde otra carpeta al directorio activo."""
        try:
            if not os.path.isdir(source_dir) or not os.path.isdir(target_dir):
                return
            for filename in os.listdir(source_dir):
                if not filename.endswith(".json"):
                    continue
                src = os.path.join(source_dir, filename)
                dst = os.path.join(target_dir, filename)
                if not os.path.exists(dst):
                    try:
                        shutil.copy2(src, dst)
                    except Exception:
                        pass
        except Exception:
            pass

    def _ensure_stream_ready(self, retries=3, wait_s=5.0):
        """Asegura que el stream de vídeo entrega frames válidos."""
        for attempt in range(1, retries + 1):
            try:
                if hasattr(self.dron, "_tello"):
                    self.dron._tello.streamoff()
                    time.sleep(0.3)
                    self.dron._tello.streamon()
                    time.sleep(0.5)

                if self._cv_cap is not None:
                    try:
                        self._cv_cap.release()
                    except Exception:
                        pass

                # Configurar VideoCapture con timeout reducido (5 segundos en vez de 30)
                # El timeout de FFmpeg se controla con stimeout (microsegundos)
                stream_url = "udp://0.0.0.0:11111?timeout=5000000"  # 5 segundos
                self._cv_cap = cv2.VideoCapture(stream_url, cv2.CAP_FFMPEG)
                self._cv_cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

                print(f"[stream] Intento {attempt}/{retries}: esperando frames...")

            except Exception as e:
                print(f"[stream] Error en intento {attempt}: {e}")

            # Esperar frames válidos (ignorar los primeros corruptos)
            t0 = time.time()
            frames_received = 0
            while time.time() - t0 < wait_s:
                if self._cv_cap is not None and self._cv_cap.isOpened():
                    ret, frame = self._cv_cap.read()
                    if ret and frame is not None:
                        frames_received += 1
                        # Aceptar después de recibir algunos frames buenos (ignora los primeros corruptos)
                        if frames_received >= 3:
                            with self._frame_lock:
                                self._last_bgr = frame
                                self._last_frame_time = time.monotonic()
                            print(f"[stream] Stream listo después de {frames_received} frames ({time.time()-t0:.1f}s)")
                            return True
                time.sleep(0.05)

            print(f"[stream] Intento {attempt}/{retries} sin frames válidos (recibidos: {frames_received})")
        return False

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
                self._ensure_stream_ready(retries=3, wait_s=3.0)
        except Exception:
            pass
        self._fpv_running = True
        self._fpv_thread = threading.Thread(target=self._fpv_loop, daemon=True)
        self._fpv_thread.start()
        self._hud_show("FPV iniciado", 1.0)

    def stop_fpv(self):
        self._fpv_running = False
        self._want_stream_on = False

        # Esperar a que el thread de FPV termine (máx 1 segundo)
        fpv_th = getattr(self, "_fpv_thread", None)
        if fpv_th is not None and fpv_th.is_alive():
            fpv_th.join(timeout=1.0)

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

        # Liberar VideoCapture en un thread separado con timeout
        # para evitar bloqueo si FFmpeg se cuelga
        cap_to_release = self._cv_cap
        self._cv_cap = None
        if cap_to_release is not None:
            def _release_cap():
                try:
                    cap_to_release.release()
                except Exception:
                    pass
            release_thread = threading.Thread(target=_release_cap, daemon=True)
            release_thread.start()
            release_thread.join(timeout=1.0)  # Máx 1 segundo para liberar
            # Si no terminó, el thread daemon morirá con el proceso

        self._hud_show("FPV detenido", 1.0)

    def _read_frame_generic(self):
        try:
            if self._cv_cap is not None and self._cv_cap.isOpened():
                # Leer frame actual sin vaciar agresivamente el buffer (evita saltos/congelación)
                with self._cv_cap_lock:
                    ok, frame = self._cv_cap.read()
                if ok and isinstance(frame, np.ndarray) and frame.size > 0:
                    return frame
        except Exception:
            pass
        return None

    def _fpv_loop(self):
        last_badge_toggle = 0.0
        rec_on = False
        last_rec_time = 0.0  # Para limitar grabación a 30fps
        try:
            while self._fpv_running:
                if self._rec_running and getattr(self, "_rec_force_direct", False):
                    time.sleep(0.03)
                    continue
                frame_bgr = self._read_frame_generic()
                if frame_bgr is None:
                    self._set_fpv_text("(esperando…)")
                    time.sleep(0.04)
                    continue
                with self._frame_lock:
                    self._last_bgr = frame_bgr
                    self._last_frame_time = time.monotonic()
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
                if self._rec_running and self._fpv_record_enabled:
                    if self._rec_writer is None:
                        print(f"[_fpv_loop] Writer es None, creando con tamaño {target_w}x{target_h}")
                        self._start_writer((target_w, target_h))
                    # Grabar solo a 30fps para que duración real = duración video
                    now_rec = time.time()
                    if now_rec - last_rec_time >= 0.033:  # ~30fps
                        try:
                            if self._rec_writer is not None:
                                self._rec_writer.write(canvas_base)
                                self._rec_frame_count += 1
                                last_rec_time = now_rec
                        except Exception as e:
                            print(f"[_fpv_loop] ERROR escribiendo frame: {e}")
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
            frame_age = time.monotonic() - self._last_frame_time if self._last_frame_time > 0 else float('inf')

        # Verificar que el frame existe y es reciente (< 3 segundos)
        if frame is None or frame_age > 3.0:
            print(f"[take_snapshot] Frame no disponible o antiguo ({frame_age:.1f}s), capturando del stream...")

            # Intentar leer directamente del stream (con lock para evitar race condition)
            if hasattr(self, '_cv_cap') and self._cv_cap is not None and self._cv_cap.isOpened():
                with self._cv_cap_lock:
                    # Flush frames antiguos
                    for _ in range(15):
                        try:
                            ret, _ = self._cv_cap.read()
                            if not ret:
                                break
                        except:
                            break

                    # Capturar frame actual
                    for attempt in range(10):
                        try:
                            ret, new_frame = self._cv_cap.read()
                            if ret and new_frame is not None:
                                frame = new_frame
                                print(f"[take_snapshot] Frame capturado del stream (intento {attempt+1})")
                                break
                        except Exception as e:
                            print(f"[take_snapshot] Error: {e}")
                        time.sleep(0.05)
                    else:
                        print("[take_snapshot] ERROR: No se pudo capturar frame")
                        self._hud_show("Error: Sin video", 1.5)
                        return False
            else:
                print("[take_snapshot] ERROR: Stream no disponible")
                self._hud_show("Error: Sin stream", 1.5)
                return False

        # Guardar foto
        path = self._session_manager.get_photo_path()
        try:
            cv2.imwrite(path, frame)
            self._hud_show("Foto guardada", 1.8)
            print(f"[take_snapshot] Foto guardada: {path}")
            return True
        except Exception as e:
            print(f"[take_snapshot] ERROR al guardar: {e}")
            self._hud_show("Error", 1.5)
            return False

    def toggle_recording(self):
        if self._rec_running:
            self._stop_recording()
        else:
            self._start_recording()

    def _start_writer(self, size_wh):
        """Crea el VideoWriter. Retorna True si tuvo éxito, False si falló."""
        self._rec_size = size_wh
        ext = os.path.splitext(self._rec_path)[1].lower()
        if ext == ".avi":
            fourcc = cv2.VideoWriter_fourcc(*"MJPG")
        else:
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        self._rec_writer = cv2.VideoWriter(self._rec_path, fourcc, self._rec_fps, self._rec_size)
        if not self._rec_writer.isOpened() and ext != ".avi":
            alt_path = os.path.splitext(self._rec_path)[0] + ".avi"
            alt_fourcc = cv2.VideoWriter_fourcc(*"MJPG")
            self._rec_writer = cv2.VideoWriter(alt_path, alt_fourcc, self._rec_fps, self._rec_size)
            if self._rec_writer.isOpened():
                self._rec_path = alt_path

        # Validar que el writer se abrió correctamente
        if not self._rec_writer.isOpened():
            print(f"[_start_writer] ERROR: No se pudo abrir VideoWriter")
            self._rec_writer = None
            return False

        print(f"[_start_writer] VideoWriter creado: path={self._rec_path}, size={size_wh}, fps={self._rec_fps}")
        return True

    def _recording_thread_func(self):
        """Hilo dedicado para grabación de video, independiente del FPV visual."""
        print("[_recording_thread] Iniciando hilo de grabación")

        print("[_recording_thread] Modo directo: leyendo del stream")
        self._recording_from_stream()

    def _recording_from_fpv_buffer(self):
        """Graba usando frames del buffer _last_bgr (cuando FPV está activo)."""
        frames_sin_datos = 0
        last_log_time = time.time()
        last_written_time = 0
        first_frame_logged = False

        # Variables para medir FPS reales del stream
        fps_measure_frames = []  # Lista de timestamps de frames recibidos
        fps_measured = False
        measured_fps = 25.0  # Valor por defecto más realista para Tello
        recording_start_time = time.monotonic()

        while self._rec_running:
            try:
                frame = None
                frame_time = 0

                # Obtener frame del buffer compartido con FPV
                with self._frame_lock:
                    if self._last_bgr is not None and self._last_frame_time > last_written_time:
                        frame = self._last_bgr.copy()
                        frame_time = self._last_frame_time

                if frame is not None:
                    frames_sin_datos = 0
                    last_written_time = frame_time

                    if not first_frame_logged:
                        print(f"[_recording_thread] Primer frame del buffer FPV: {frame.shape}")
                        first_frame_logged = True
                        recording_start_time = time.monotonic()

                    # Medir FPS reales durante el primer segundo
                    if not fps_measured:
                        fps_measure_frames.append(time.monotonic())
                        elapsed_measure = fps_measure_frames[-1] - fps_measure_frames[0]
                        if elapsed_measure >= 1.0 and len(fps_measure_frames) >= 5:
                            # Calcular FPS reales
                            measured_fps = (len(fps_measure_frames) - 1) / elapsed_measure
                            # Limitar a rango razonable (15-35 FPS)
                            measured_fps = max(15.0, min(35.0, measured_fps))
                            self._rec_fps = measured_fps
                            print(f"[_recording_thread] FPS medidos del buffer FPV: {measured_fps:.1f}")
                            fps_measured = True
                            fps_measure_frames = []  # Liberar memoria

                    # Crear writer si no existe (después de medir FPS o tras 1.5s)
                    elapsed_total = time.monotonic() - recording_start_time
                    if self._rec_writer is None and (fps_measured or elapsed_total > 1.5):
                        if not fps_measured:
                            # Timeout, usar valor por defecto
                            print(f"[_recording_thread] Timeout midiendo FPS, usando {measured_fps:.1f}")
                            self._rec_fps = measured_fps
                            fps_measured = True
                        h, w = frame.shape[:2]
                        print(f"[_recording_thread] Creando writer con tamaño {w}x{h}, FPS={self._rec_fps:.1f}")
                        self._start_writer((w, h))

                    # Escribir cada frame nuevo que llega (sin limitación artificial)
                    if self._rec_writer is not None:
                        self._rec_writer.write(frame)
                        self._rec_frame_count += 1

                        # Log cada ~1 segundo
                        if time.time() - last_log_time >= 1.0:
                            actual_fps = self._rec_frame_count / max(0.1, time.monotonic() - recording_start_time)
                            print(f"[_recording_thread] Frames: {self._rec_frame_count}, FPS actual: {actual_fps:.1f}")
                            last_log_time = time.time()
                else:
                    frames_sin_datos += 1
                    if frames_sin_datos == 50:
                        print("[_recording_thread] WARNING: 50 lecturas sin frames nuevos del buffer FPV")

                time.sleep(0.015)  # ~66 checks por segundo

            except Exception as e:
                print(f"[_recording_thread] Error: {e}")
                import traceback
                traceback.print_exc()
                time.sleep(0.05)

        print(f"[_recording_thread] Hilo terminado, frames grabados: {self._rec_frame_count}")

    def _recording_from_stream(self):
        """Graba leyendo directamente del stream (cuando FPV NO está activo)."""
        # Diagnóstico inicial del stream
        cap_exists = hasattr(self, '_cv_cap') and self._cv_cap is not None
        cap_opened = cap_exists and self._cv_cap.isOpened()
        print(f"[_recording_thread] Estado stream: cap_exists={cap_exists}, cap_opened={cap_opened}")

        if not cap_opened:
            print("[_recording_thread] ERROR: Stream no disponible, intentando reiniciar...")
            if not self._ensure_stream_ready(retries=2, wait_s=2.0):
                print("[_recording_thread] ERROR: No se pudo iniciar el stream, abortando grabación")
                return

        # Flush del buffer UDP - descartar frames antiguos
        print("[_recording_thread] Flushing buffer de frames antiguos...")
        flush_count = 0
        flush_start = time.monotonic()
        if hasattr(self, '_cv_cap') and self._cv_cap is not None and self._cv_cap.isOpened():
            while time.monotonic() - flush_start < 0.5:
                with self._cv_cap_lock:
                    ret, _ = self._cv_cap.read()
                if ret:
                    flush_count += 1
                else:
                    time.sleep(0.01)
        print(f"[_recording_thread] Flushed {flush_count} frames antiguos del buffer")

        frames_sin_datos = 0
        last_log_time = time.time()
        last_good_frame = None
        first_frame_logged = False

        # Variables para medir FPS reales del stream
        fps_measure_frames = []  # Lista de timestamps de frames recibidos
        fps_measured = False
        measured_fps = 25.0  # Valor por defecto más realista para Tello
        recording_start_time = time.monotonic()

        while self._rec_running:
            try:
                frame = None

                # Leer frame del stream
                if hasattr(self, '_cv_cap') and self._cv_cap is not None and self._cv_cap.isOpened():
                    with self._cv_cap_lock:
                        ret, direct_frame = self._cv_cap.read()
                    if ret and direct_frame is not None:
                        frame = direct_frame
                        frame_time = time.monotonic()
                        with self._frame_lock:
                            self._last_bgr = frame
                            self._rec_last_good_frame = frame
                            self._last_frame_time = frame_time
                        if not first_frame_logged:
                            print(f"[_recording_thread] Primer frame recibido: {frame.shape}")
                            first_frame_logged = True
                            recording_start_time = frame_time
                    else:
                        if frames_sin_datos == 0:
                            print(f"[_recording_thread] cv_cap.read() falló: ret={ret}")

                if frame is not None:
                    frames_sin_datos = 0
                    last_good_frame = frame
                    self._rec_last_good_frame = frame

                    # Medir FPS reales durante el primer segundo
                    if not fps_measured:
                        fps_measure_frames.append(time.monotonic())
                        elapsed_measure = fps_measure_frames[-1] - fps_measure_frames[0]
                        if elapsed_measure >= 1.0 and len(fps_measure_frames) >= 5:
                            # Calcular FPS reales
                            measured_fps = (len(fps_measure_frames) - 1) / elapsed_measure
                            # Limitar a rango razonable (15-35 FPS)
                            measured_fps = max(15.0, min(35.0, measured_fps))
                            self._rec_fps = measured_fps
                            print(f"[_recording_thread] FPS medidos del stream: {measured_fps:.1f}")
                            fps_measured = True
                            fps_measure_frames = []  # Liberar memoria

                    # Crear writer si no existe (después de medir FPS o tras 1.5s)
                    elapsed_total = time.monotonic() - recording_start_time
                    if self._rec_writer is None and (fps_measured or elapsed_total > 1.5):
                        if not fps_measured:
                            # Timeout, usar valor por defecto
                            print(f"[_recording_thread] Timeout midiendo FPS, usando {measured_fps:.1f}")
                            self._rec_fps = measured_fps
                            fps_measured = True
                        h, w = frame.shape[:2]
                        print(f"[_recording_thread] Creando writer con tamaño {w}x{h}, FPS={self._rec_fps:.1f}")
                        self._start_writer((w, h))

                    # Escribir cada frame que llega (sin limitación artificial)
                    # El stream ya viene a la velocidad del drone
                    if self._rec_writer is not None:
                        self._rec_writer.write(frame)
                        self._rec_frame_count += 1

                        # Log cada ~1 segundo
                        if time.time() - last_log_time >= 1.0:
                            actual_fps = self._rec_frame_count / max(0.1, time.monotonic() - recording_start_time)
                            print(f"[_recording_thread] Frames: {self._rec_frame_count}, FPS actual: {actual_fps:.1f}")
                            last_log_time = time.time()
                else:
                    frames_sin_datos += 1
                    if frames_sin_datos == 30:
                        print("[_recording_thread] WARNING: 30 lecturas sin frames")
                    elif frames_sin_datos == 100:
                        print("[_recording_thread] ERROR: 100 lecturas sin frames, stream probablemente muerto")
                    time.sleep(0.01)

            except Exception as e:
                print(f"[_recording_thread] Error: {e}")
                import traceback
                traceback.print_exc()
                time.sleep(0.05)

        print(f"[_recording_thread] Hilo terminado, frames grabados: {self._rec_frame_count}")

    def _start_recording(self, force_dedicated_thread=False, expected_duration=None):
        # Usar gestor de sesiones para obtener ruta
        self._rec_path = self._session_manager.get_video_path()
        self._rec_writer = None
        self._rec_frame_count = 0
        self._rec_thread = None
        self._rec_last_good_frame = None
        self._rec_force_direct = False
        self._fpv_record_enabled = bool(self._fpv_running)
        self._rec_fps = 30
        self._rec_start_mono = time.monotonic()
        if expected_duration is not None:
            try:
                self._rec_expected_frames = max(0, int(round(float(expected_duration) * self._rec_fps)))
            except Exception:
                self._rec_expected_frames = None
        else:
            self._rec_expected_frames = None

        # Asegurar que hay un stream de video activo
        # Si el FPV está corriendo, reutilizarlo y no reiniciar el stream
        need_stream = False
        if not self._fpv_running and not self._fpv_ext_running:
            if not hasattr(self, '_cv_cap') or self._cv_cap is None:
                need_stream = True
            elif not self._cv_cap.isOpened():
                need_stream = True

        if need_stream:
            print("[_start_recording] Iniciando stream de video...")
            if not self._ensure_stream_ready(retries=3, wait_s=3.0):
                print("[_start_recording] ERROR: No se pudo estabilizar el stream, abortando grabación")
                self._hud_show("Error: Sin stream", 2.0)
                return False

        # Esperar a que haya frames disponibles (timeout 5 segundos)
        print("[_start_recording] Esperando frames del stream...")
        max_wait = 50  # 5 segundos máximo
        frame_found = False
        for i in range(max_wait):
            if hasattr(self, '_cv_cap') and self._cv_cap is not None and self._cv_cap.isOpened():
                with self._cv_cap_lock:
                    ret, new_frame = self._cv_cap.read()
                if ret and new_frame is not None:
                    with self._frame_lock:
                        self._last_bgr = new_frame
                        self._last_frame_time = time.monotonic()
                    print(f"[_start_recording] Frame disponible después de {i*0.1:.1f}s")
                    frame_found = True
                    break
            time.sleep(0.1)

        if not frame_found:
            print("[_start_recording] ERROR: Timeout esperando frames del stream, abortando grabación")
            self._hud_show("Error: Sin frames", 2.0)
            return False

        # Iniciar el flag de grabación
        self._rec_running = True

        # Usar hilo dedicado siempre para mantener cadencia estable
        use_dedicated = True
        if use_dedicated:
            self._fpv_record_enabled = False
            self._rec_force_direct = True
            # NO cerrar FPV - dejar que siga mostrando video mientras graba

        if use_dedicated:
            self._rec_thread = threading.Thread(target=self._recording_thread_func, daemon=True)
            self._rec_thread.start()
            print("[_start_recording] Hilo de grabación dedicado iniciado")
        else:
            print("[_start_recording] Grabación delegada al loop de FPV")

        self._hud_show("Grabando...", 1.5)
        return True

    def _stop_recording(self):
        if self._rec_running:
            self._rec_running = False

            # Esperar a que el hilo de grabación termine
            if hasattr(self, '_rec_thread') and self._rec_thread is not None:
                print("[_stop_recording] Esperando a que termine el hilo de grabación...")
                self._rec_thread.join(timeout=2.0)  # Máximo 2 segundos
                if self._rec_thread.is_alive():
                    print("[_stop_recording] WARNING: El hilo no terminó a tiempo")

            video_path = self._rec_path
            try:
                if self._rec_writer is not None and self._rec_writer.isOpened():
                    frames_recorded = self._rec_frame_count
                    elapsed = max(0.1, time.monotonic() - (self._rec_start_mono or time.monotonic()))
                    actual_fps = frames_recorded / elapsed
                    print(f"[_stop_recording] Liberando writer, guardando video en {video_path}")
                    print(f"[_stop_recording] Total frames: {frames_recorded}, Duración: {elapsed:.1f}s, FPS real: {actual_fps:.1f}")
                    self._rec_writer.release()

                    # Verificar que el video no está vacío
                    import os
                    if os.path.exists(video_path):
                        file_size = os.path.getsize(video_path)
                        if file_size < 1000:  # Menos de 1KB = video vacío/corrupto
                            print(f"[_stop_recording] ⚠️ VIDEO VACÍO detectado ({file_size} bytes)")
                            print(f"[_stop_recording] Eliminando archivo vacío: {video_path}")
                            try:
                                os.remove(video_path)
                            except Exception:
                                pass
                            self._hud_show("Error: Video vacío", 2.0)
                        else:
                            print(f"[_stop_recording] Video guardado: {file_size/1024:.1f} KB, {frames_recorded} frames")
                            self._hud_show("Video guardado", 1.5)
                    else:
                        print(f"[_stop_recording] WARNING: El archivo de video no existe")
                        self._hud_show("Error grabación", 1.5)
                else:
                    print("[_stop_recording] WARNING: No había writer para liberar")
                    # Eliminar archivo vacío si existe
                    import os
                    if video_path and os.path.exists(video_path):
                        try:
                            os.remove(video_path)
                            print(f"[_stop_recording] Archivo vacío eliminado: {video_path}")
                        except Exception:
                            pass
                    self._hud_show("Error: Sin grabación", 1.5)
            except Exception as e:
                print(f"[_stop_recording] ERROR liberando writer: {e}")
                self._hud_show("Error grabación", 1.5)

            self._rec_writer = None
            self._rec_force_direct = False

    # FPV EXTERNO (cv2.imshow con VideoCapture compartido)
    def toggle_fpv_external(self):
        """Alterna la ventana FPV externa."""
        if self._fpv_ext_running:
            self.stop_fpv_external()
        else:
            self.start_fpv_external()

    def start_fpv_external(self):
        """Abre ventana FPV externa con cv2.imshow (baja latencia)."""
        if self._fpv_ext_running:
            return

        # Asegurar que el stream está activo y _cv_cap existe
        try:
            if self._cv_cap is None or not self._cv_cap.isOpened():
                self._want_stream_on = True
                if hasattr(self.dron, "_tello"):
                    self.dron._tello.streamon()
                    time.sleep(0.3)
                # Timeout de 5 segundos en vez de 30
                self._cv_cap = cv2.VideoCapture("udp://0.0.0.0:11111?timeout=5000000", cv2.CAP_FFMPEG)
                self._cv_cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass

        self._fpv_ext_running = True
        self._fpv_ext_thread = threading.Thread(target=self._fpv_ext_cv_loop, daemon=True)
        self._fpv_ext_thread.start()

    def stop_fpv_external(self):
        """Cierra la ventana FPV externa."""
        self._fpv_ext_running = False

        # Esperar a que el thread de FPV externo termine (máx 1 segundo)
        ext_th = getattr(self, "_fpv_ext_thread", None)
        if ext_th is not None and ext_th.is_alive():
            ext_th.join(timeout=1.0)

        # NO cerramos _cv_cap - se queda abierto para reutilizar
        try:
            cv2.destroyWindow("Tello FPV")
        except Exception:
            pass

    def _fpv_ext_cv_loop(self):
        """Loop de FPV externo usando cv2.imshow (rápido)."""
        window_name = "Tello FPV"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, 640, 480)
        last_rec_time = 0.0  # Para limitar grabación a 30fps

        while self._fpv_ext_running:
            try:
                # Detectar si cerraron la ventana con X
                if cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) < 1:
                    break
            except Exception:
                break

            try:
                if self._cv_cap is not None and self._cv_cap.isOpened():
                    with self._cv_cap_lock:
                        ok, frame = self._cv_cap.read()
                    if ok and frame is not None:
                        # Guardar para snapshots
                        with self._frame_lock:
                            self._last_bgr = frame.copy()

                        # Grabación (solo si no hay hilo dedicado activo)
                        if self._rec_running and self._rec_thread is None:
                            if self._rec_writer is None:
                                h, w = frame.shape[:2]
                                self._start_writer((w, h))
                            # Grabar solo a 30fps para que duración real = duración video
                            now_rec = time.time()
                            if now_rec - last_rec_time >= 0.033:  # ~30fps
                                if self._rec_writer is not None:
                                    try:
                                        self._rec_writer.write(frame)
                                        self._rec_frame_count += 1
                                        last_rec_time = now_rec
                                    except Exception:
                                        pass

                        # Dibujar overlays
                        self._draw_overlays(frame)

                        cv2.imshow(window_name, frame)

                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or key == 27:
                    break
            except Exception:
                pass

        self._fpv_ext_running = False
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
    def _init_joystick(self, full_reinit: bool = False):
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

            if not controller.connect(full_reinit=full_reinit):
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

                            # Marcar RC activo para que telemetría no duplique integración de pose
                            self.dron._rc_active = True

                            try:
                                vx_gf, vy_gf, vz_gf, yaw_gf = self.dron.aplicar_geofence_rc(_vx, _vy, _vz, _yaw)
                                self.dron.rc(int(vx_gf), int(vy_gf), int(vz_gf), int(yaw_gf))
                                self._last_rc_sent = time.time()
                            except Exception:
                                self.dron.rc(int(_vx), int(_vy), int(_vz), int(_yaw))

                            if hasattr(self.dron, "pose") and self.dron.pose:
                                self.dron.pose.update_from_rc(vy_gf, vx_gf, vz_gf, yaw_gf, dt_sec=0.05)
                        else:
                            # Si no está volando, desactivar flag RC
                            self.dron._rc_active = False

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
        # Desactivar flag RC cuando se para el joystick
        if hasattr(self, "dron") and self.dron:
            self.dron._rc_active = False

    def _reconnect_joystick(self):
        """Detiene el joystick actual e intenta reconectar."""
        print("[joy] Intentando reconectar joystick...")
        self._joy_label_var.set("🔄 Buscando...")

        def _reconnect_thread():
            # Parar el hilo actual y esperar a que termine
            self._joy_running = False
            time.sleep(0.5)

            # Llamar _init_joystick con full_reinit=True desde el hilo principal de Tk
            self.root.after(0, lambda: self._init_joystick(full_reinit=True))

        threading.Thread(target=_reconnect_thread, daemon=True).start()

    #MAPA
    def open_map_window(self):

        if self._map_win and tk.Toplevel.winfo_exists(self._map_win):
            self._map_win.lift()
            return

        self._map_win = tk.Toplevel(self.root)
        self._map_win.title("Mapa Geofence")
        self._map_win.geometry(f"{MAP_SIZE_PX + 320}x{MAP_SIZE_PX + 50}")

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
        side_canvas = tk.Canvas(side_container, width=280, highlightthickness=0)
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
        tk.Radiobutton(tool_frame, text="✋", variable=self._tool_var, value="select",
                       bg=BG_CARD, activebackground=BG_CARD, font=("Arial", 12)).pack(side="left")

        # Radio
        radio_frame = tk.Frame(content1, bg=BG_CARD)
        radio_frame.pack(fill="x", pady=4)
        tk.Label(radio_frame, text="Radio:", bg=BG_CARD, font=("Arial", 8)).pack(side="left")
        self._circle_radius_var = tk.IntVar(value=30)
        tk.Entry(radio_frame, textvariable=self._circle_radius_var, width=5,
                 justify="center").pack(side="left", padx=4)
        tk.Label(radio_frame, text="cm", bg=BG_CARD, font=("Arial", 8)).pack(side="left")

        # Selector de capas con checkboxes (permite combinaciones libres)
        self._draw_layer_c1 = tk.BooleanVar(value=True)
        self._draw_layer_c2 = tk.BooleanVar(value=False)
        self._draw_layer_c3 = tk.BooleanVar(value=False)

        tk.Label(content1, text="Capas del obstáculo:", bg=BG_CARD, font=("Arial", 8)).pack(anchor="w")
        layer_btn_frame = tk.Frame(content1, bg=BG_CARD)
        layer_btn_frame.pack(fill="x", pady=2)

        tk.Checkbutton(layer_btn_frame, text="C1", variable=self._draw_layer_c1,
                       bg=layer_colors[0], fg="white", selectcolor=layer_colors[0],
                       activebackground=layer_colors[0], font=("Arial", 8, "bold"),
                       indicatoron=0, width=3, command=self._on_draw_layer_change).pack(side="left", padx=1)
        tk.Checkbutton(layer_btn_frame, text="C2", variable=self._draw_layer_c2,
                       bg=layer_colors[1], fg="white", selectcolor=layer_colors[1],
                       activebackground=layer_colors[1], font=("Arial", 8, "bold"),
                       indicatoron=0, width=3, command=self._on_draw_layer_change).pack(side="left", padx=1)
        tk.Checkbutton(layer_btn_frame, text="C3", variable=self._draw_layer_c3,
                       bg=layer_colors[2], fg="white", selectcolor=layer_colors[2],
                       activebackground=layer_colors[2], font=("Arial", 8, "bold"),
                       indicatoron=0, width=3, command=self._on_draw_layer_change).pack(side="left", padx=1)

        # Botón "Todas" para marcar/desmarcar todas
        def toggle_all_layers():
            all_on = self._draw_layer_c1.get() and self._draw_layer_c2.get() and self._draw_layer_c3.get()
            self._draw_layer_c1.set(not all_on)
            self._draw_layer_c2.set(not all_on)
            self._draw_layer_c3.set(not all_on)
            self._on_draw_layer_change()  # Actualizar indicador

        tk.Button(layer_btn_frame, text="ALL", command=toggle_all_layers,
                  bg="#6c757d", fg="white", font=("Arial", 8, "bold"), bd=0, width=4).pack(side="left", padx=1)

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
        # SECCIÓN: ESCENARIO
        # ═══════════════════════════════════════════════════════
        card_scenario_map = tk.Frame(side_panel, bg=BG_CARD, bd=1, relief="solid")
        card_scenario_map.pack(fill="x", padx=4, pady=4)

        tk.Label(card_scenario_map, text="  ESCENARIO", font=("Arial", 9, "bold"),
                 bg=BG_HEADER, fg=FG_HEADER, anchor="w").pack(fill="x", ipady=4)

        scenario_content_map = tk.Frame(card_scenario_map, bg=BG_CARD)
        scenario_content_map.pack(fill="x", padx=8, pady=8)

        # Combo para seleccionar escenario
        scenario_row_map = tk.Frame(scenario_content_map, bg=BG_CARD)
        scenario_row_map.pack(fill="x", pady=2)
        tk.Label(scenario_row_map, text="Escenario:", bg=BG_CARD, font=("Arial", 8)).pack(side="left")

        self._map_scenario_combo_var = tk.StringVar(value="")
        self._map_scenario_combo = ttk.Combobox(scenario_row_map, textvariable=self._map_scenario_combo_var,
                                                 width=15, state="readonly")
        self._map_scenario_combo.pack(side="left", padx=4)

        # Botón refrescar
        tk.Button(scenario_row_map, text="↻", command=self._refresh_map_scenario_list,
                  bg="#6c757d", fg="white", font=("Arial", 8), bd=0, width=2).pack(side="left", padx=2)

        # Botones de escenario: Cargar y Guardar
        scenario_btns_map = tk.Frame(scenario_content_map, bg=BG_CARD)
        scenario_btns_map.pack(fill="x", pady=2)
        tk.Button(scenario_btns_map, text="📂 Cargar", command=self._load_scenario_from_file_to_map,
                  bg="#17a2b8", fg="white", font=("Arial", 8), bd=0).pack(side="left", fill="x", expand=True, padx=2)
        tk.Button(scenario_btns_map, text="💾 Guardar", command=self._save_scenario_from_map,
                  bg="#28a745", fg="white", font=("Arial", 8), bd=0).pack(side="left", fill="x", expand=True, padx=2)

        # Nombre del escenario
        self._map_scenario_name_var = tk.StringVar(value="(ninguno)")
        tk.Label(scenario_content_map, textvariable=self._map_scenario_name_var, bg=BG_CARD,
                 font=("Arial", 8, "italic"), fg="#666").pack(anchor="w", pady=2)

        # Refrescar lista
        self._refresh_map_scenario_list()

        # ═══════════════════════════════════════════════════════
        # SECCIÓN: EDITAR OBSTÁCULO SELECCIONADO
        # ═══════════════════════════════════════════════════════
        card_obs_edit = tk.Frame(side_panel, bg=BG_CARD, bd=1, relief="solid")
        card_obs_edit.pack(fill="x", padx=4, pady=4)

        tk.Label(card_obs_edit, text="  EDITAR OBSTÁCULO", font=("Arial", 9, "bold"),
                 bg=BG_HEADER, fg=FG_HEADER, anchor="w").pack(fill="x", ipady=4)

        obs_edit_content = tk.Frame(card_obs_edit, bg=BG_CARD)
        obs_edit_content.pack(fill="x", padx=8, pady=8)

        # Botón para activar modo selección
        tk.Button(obs_edit_content, text="✋ Seleccionar obstáculo", command=self._map_start_select_obs,
                  bg=ACCENT, fg="white", font=("Arial", 8), bd=0,
                  activebackground="#0056b3").pack(fill="x", pady=(0, 6))

        self._map_obs_edit_label = tk.Label(obs_edit_content, text="Clic en ✋ y luego en obstáculo",
                                             bg=BG_CARD, font=("Arial", 8, "italic"), fg="#666")
        self._map_obs_edit_label.pack(anchor="w")

        # Campos de edición (para círculo: cx, cy, r)
        obs_edit_row1 = tk.Frame(obs_edit_content, bg=BG_CARD)
        obs_edit_row1.pack(fill="x", pady=(4, 2))
        tk.Label(obs_edit_row1, text="X:", bg=BG_CARD, font=("Arial", 8)).pack(side="left")
        self._map_obs_edit_x = tk.Entry(obs_edit_row1, width=5)
        self._map_obs_edit_x.pack(side="left", padx=(2, 8))
        tk.Label(obs_edit_row1, text="Y:", bg=BG_CARD, font=("Arial", 8)).pack(side="left")
        self._map_obs_edit_y = tk.Entry(obs_edit_row1, width=5)
        self._map_obs_edit_y.pack(side="left", padx=(2, 8))

        obs_edit_row2 = tk.Frame(obs_edit_content, bg=BG_CARD)
        obs_edit_row2.pack(fill="x", pady=2)
        tk.Label(obs_edit_row2, text="Radio/Tamaño:", bg=BG_CARD, font=("Arial", 8)).pack(side="left")
        self._map_obs_edit_size = tk.Entry(obs_edit_row2, width=5)
        self._map_obs_edit_size.pack(side="left", padx=(2, 8))

        obs_edit_btns = tk.Frame(obs_edit_content, bg=BG_CARD)
        obs_edit_btns.pack(fill="x", pady=(4, 0))
        tk.Button(obs_edit_btns, text="✓ Aplicar", command=self._map_apply_obs_edit,
                  bg="#28a745", fg="white", font=("Arial", 8, "bold"), bd=0, padx=8).pack(side="left", padx=2)
        tk.Button(obs_edit_btns, text="🗑 Eliminar", command=self._map_delete_selected_obs,
                  bg="#dc3545", fg="white", font=("Arial", 8, "bold"), bd=0, padx=8).pack(side="left", padx=2)

        # Botón duplicar a capa (usa los checkboxes de capa seleccionados)
        obs_edit_btns2 = tk.Frame(obs_edit_content, bg=BG_CARD)
        obs_edit_btns2.pack(fill="x", pady=(4, 0))
        tk.Button(obs_edit_btns2, text="📋 Duplicar a capa seleccionada", command=self._map_duplicate_obs_to_layer,
                  bg="#9b59b6", fg="white", font=("Arial", 8, "bold"), bd=0).pack(fill="x")

        # Variable para obstáculo seleccionado en mapa
        self._map_selected_obs_idx = None
        self._map_selected_obs_type = None  # 'circle', 'poly' o 'rect'

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

            # Actualizar el indicador de la esquina (usa checkboxes)
            if self._layer_label:
                self._on_draw_layer_change()

            # Actualizar también el indicador de la ventana de misiones (usa checkboxes)
            if self._mission_layer_label:
                self._on_mission_layer_change()

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

        # Inicializar indicador con la capa de trabajo seleccionada
        self._on_draw_layer_change()

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

        # Obtener capa de trabajo para colorear exclusiones
        # Si current_layer es 0, significa "ALL" - mostrar todos en color fuerte
        current_layer = self._current_layer
        show_all = (current_layer == 0)

        # Colores para exclusiones
        COLOR_IN_LAYER = "#ff0000"      # Rojo - obstáculo en capa actual
        COLOR_OTHER_LAYER = "#ffcccc"   # Rojo claro - obstáculo en otra capa
        WIDTH_IN_LAYER = 3
        WIDTH_OTHER_LAYER = 1

        # Exclusiones (círculos)
        for i, c in enumerate(self._excl_circles):
            cx_w, cy_w, r_w = c["cx"], c["cy"], c["r"]
            cx_px, cy_px = self._world_to_canvas(cx_w, cy_w)
            r_px = r_w * PX_PER_CM

            # Determinar si está en la capa actual (o ALL = todos)
            excl_layers = self._get_exclusion_layers(c)
            in_current_layer = show_all or (current_layer in excl_layers)

            # Verificar si está seleccionado
            is_selected = (getattr(self, '_map_selected_obs_idx', None) == i and
                          getattr(self, '_map_selected_obs_type', None) == 'circle')

            if is_selected:
                color = "#00ff00"  # Verde para seleccionado
                width = 4
            else:
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
        for i, p in enumerate(self._excl_polys):
            pts = p.get("poly")
            if not pts and "x1" in p and "y1" in p and "x2" in p and "y2" in p:
                pts = [
                    (p["x1"], p["y1"]),
                    (p["x2"], p["y1"]),
                    (p["x2"], p["y2"]),
                    (p["x1"], p["y2"]),
                ]
            if pts and len(pts) >= 3:
                canvas_pts = []
                for (px, py) in pts:
                    canvas_pts.extend(self._world_to_canvas(px, py))

                # Determinar si está en la capa actual (o ALL = todos)
                excl_layers = self._get_exclusion_layers(p)
                in_current_layer = show_all or (current_layer in excl_layers)

                # Verificar si está seleccionado
                is_selected = (getattr(self, '_map_selected_obs_idx', None) == i and
                              getattr(self, '_map_selected_obs_type', None) == 'poly')

                if is_selected:
                    color = "#00ff00"  # Verde para seleccionado
                    width = 4
                else:
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
        elif tool == "select":
            self._map_select_obstacle_at(wx, wy)

    def _on_draw_layer_change(self, *args):
        """Actualiza el indicador de capa cuando cambia la selección (checkboxes)."""
        # Obtener capas seleccionadas desde checkboxes
        c1 = getattr(self, '_draw_layer_c1', None)
        c2 = getattr(self, '_draw_layer_c2', None)
        c3 = getattr(self, '_draw_layer_c3', None)

        selected = []
        if c1 and c1.get():
            selected.append(1)
        if c2 and c2.get():
            selected.append(2)
        if c3 and c3.get():
            selected.append(3)

        zmin, zmax = self._get_layer_z_range()

        # Actualizar el indicador grande de la esquina
        if self._layer_label:
            # Determinar color según capas seleccionadas
            if len(selected) == 0:
                bg_color = "#6c757d"  # Gris - ninguna (usará todas)
                layer_text = "Todas las capas"
            elif len(selected) == 3:
                bg_color = "#6c757d"  # Gris - todas
                layer_text = "Todas las capas"
            elif len(selected) == 1:
                colors = {1: "#28a745", 2: "#fd7e14", 3: "#007bff"}
                bg_color = colors.get(selected[0], "#333333")
                layer_text = f"Capa {selected[0]}"
            else:
                # Combinación de 2 capas
                bg_color = "#9b59b6"  # Púrpura para combinaciones
                layer_text = "Capas " + "+".join(str(s) for s in selected)

            self._layer_label.config(
                text=f"{layer_text}\n({zmin}-{zmax} cm)",
                bg=bg_color
            )

            # Redibujar mapa para mostrar exclusiones
            self._current_layer = selected[0] if len(selected) == 1 else 0
            self._redraw_map_static()

    def _get_layer_z_range(self, layer_str=None):
        """Obtiene el rango Z para las capas seleccionadas (checkboxes)."""
        # Leer de los sliders de la UI si existen
        c1_max = getattr(self, '_layer1_max_var', None)
        c2_max = getattr(self, '_layer2_max_var', None)
        c3_max = getattr(self, '_layer3_max_var', None)

        if c1_max and c2_max and c3_max:
            # Usar valores de los sliders
            layers = [
                {"z_min": 0, "z_max": c1_max.get()},
                {"z_min": c1_max.get(), "z_max": c2_max.get()},
                {"z_min": c2_max.get(), "z_max": c3_max.get()},
            ]
        elif hasattr(self.dron, "get_layers"):
            layers = self.dron.get_layers()
        else:
            layers = [
                {"z_min": 0, "z_max": 60},
                {"z_min": 60, "z_max": 120},
                {"z_min": 120, "z_max": 200},
            ]

        # Si se pasa un string (compatibilidad), usar lógica antigua
        if layer_str is not None:
            if layer_str == "all":
                return 0, int(layers[-1]["z_max"]) if layers else 200
            elif layer_str == "1":
                return int(layers[0]["z_min"]), int(layers[0]["z_max"])
            elif layer_str == "2":
                return int(layers[1]["z_min"]), int(layers[1]["z_max"])
            elif layer_str == "3":
                return int(layers[2]["z_min"]), int(layers[2]["z_max"])
            elif layer_str == "1+2":
                return int(layers[0]["z_min"]), int(layers[1]["z_max"])
            elif layer_str == "2+3":
                return int(layers[1]["z_min"]), int(layers[2]["z_max"])
            elif layer_str == "1+3":
                return int(layers[0]["z_min"]), int(layers[2]["z_max"])
            else:
                return 0, 200

        # Usar checkboxes para determinar capas seleccionadas
        c1 = getattr(self, '_draw_layer_c1', None)
        c2 = getattr(self, '_draw_layer_c2', None)
        c3 = getattr(self, '_draw_layer_c3', None)

        selected = []
        if c1 and c1.get():
            selected.append(0)
        if c2 and c2.get():
            selected.append(1)
        if c3 and c3.get():
            selected.append(2)

        if not selected:
            # Si no hay ninguna seleccionada, usar todas
            return 0, int(layers[-1]["z_max"]) if layers else 200

        # Calcular zmin y zmax basándose en las capas seleccionadas
        zmin = int(layers[min(selected)]["z_min"])
        zmax = int(layers[max(selected)]["z_max"])
        return zmin, zmax

    def _get_selected_layers(self):
        """Devuelve la lista de capas seleccionadas (1, 2, 3) desde los checkboxes."""
        c1 = getattr(self, '_draw_layer_c1', None)
        c2 = getattr(self, '_draw_layer_c2', None)
        c3 = getattr(self, '_draw_layer_c3', None)

        selected = []
        if c1 and c1.get():
            selected.append(1)
        if c2 and c2.get():
            selected.append(2)
        if c3 and c3.get():
            selected.append(3)

        return selected if selected else [1, 2, 3]  # Si ninguna, todas

    def _add_exclusion_circle(self, wx, wy):

        r = float(self._circle_radius_var.get() or 30.0)

        # Obtener Z según las capas seleccionadas (checkboxes)
        zmin, zmax = self._get_layer_z_range()
        selected_layers = self._get_selected_layers()

        # Añadir a AMBAS listas para mantener sincronización
        self._excl_circles.append({"cx": wx, "cy": wy, "r": r, "zmin": zmin, "zmax": zmax, "layers": selected_layers})
        self._mission_exclusions.append({
            'type': 'circle',
            'cx': wx, 'cy': wy, 'r': r,
            'zmin': zmin, 'zmax': zmax,
            'layers': selected_layers
        })

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
        print(f"[DEBUG] _add_rectangle_point llamado: ({wx}, {wy})")
        if not hasattr(self, '_rect_points'):
            self._rect_points = []

        self._rect_points.append((wx, wy))
        print(f"[DEBUG] _rect_points ahora tiene {len(self._rect_points)} puntos")
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

            # Obtener Z según las capas seleccionadas (checkboxes)
            zmin, zmax = self._get_layer_z_range()
            selected_layers = self._get_selected_layers()

            # Crear polígono rectangular (4 esquinas)
            rect_poly = [(x1, y1), (x2, y1), (x2, y2), (x1, y2)]

            # Añadir a AMBAS listas para mantener sincronización
            self._excl_polys.append({"poly": rect_poly, "zmin": zmin, "zmax": zmax, "layers": selected_layers})
            self._mission_exclusions.append({
                'type': 'poly',
                'points': rect_poly,
                'zmin': zmin, 'zmax': zmax,
                'layers': selected_layers
            })
            print(f"[DEBUG] Rectángulo añadido. _excl_polys: {len(self._excl_polys)}, _mission_exclusions: {len(self._mission_exclusions)}")

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
        print(f"[DEBUG] _close_polygon llamado. Puntos: {len(self._poly_points)}")

        if len(self._poly_points) < 3:
            messagebox.showwarning("Polígono", "Necesitas al menos 3 puntos.")
            return

        # Obtener Z según las capas seleccionadas (checkboxes)
        zmin, zmax = self._get_layer_z_range()
        selected_layers = self._get_selected_layers()

        poly_points_copy = list(self._poly_points)

        # Añadir a AMBAS listas para mantener sincronización
        self._excl_polys.append({"poly": poly_points_copy, "zmin": zmin, "zmax": zmax, "layers": selected_layers})
        self._mission_exclusions.append({
            'type': 'poly',
            'points': poly_points_copy,
            'zmin': zmin, 'zmax': zmax,
            'layers': selected_layers
        })
        print(f"[DEBUG] Polígono añadido. _excl_polys: {len(self._excl_polys)}, _mission_exclusions: {len(self._mission_exclusions)}")

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

        # Limpiar selección de obstáculo
        self._map_selected_obs_idx = None
        self._map_selected_obs_type = None
        if hasattr(self, '_map_obs_edit_label'):
            self._map_obs_edit_label.configure(text="Clic en ✋ y luego en obstáculo")
        if hasattr(self, '_map_obs_edit_x'):
            self._map_obs_edit_x.delete(0, tk.END)
            self._map_obs_edit_y.delete(0, tk.END)
            self._map_obs_edit_size.delete(0, tk.END)
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

    def _map_start_select_obs(self):
        """Activa el modo de selección de obstáculos en el mapa."""
        self._tool_var.set("select")
        self._hud_show("Clic en un obstáculo para seleccionarlo", 3.0)

    def _map_select_obstacle_at(self, wx, wy):
        """Busca y selecciona un obstáculo en las coordenadas dadas."""
        import math

        # Buscar en círculos
        for i, c in enumerate(self._excl_circles):
            cx, cy, r = c.get("cx", 0), c.get("cy", 0), c.get("r", 30)
            dist = math.sqrt((wx - cx)**2 + (wy - cy)**2)
            if dist <= r:
                self._map_selected_obs_idx = i
                self._map_selected_obs_type = 'circle'
                self._map_fill_obs_edit_fields(c, 'circle')
                self._redraw_map_static()
                return

        # Buscar en polígonos
        for i, p in enumerate(self._excl_polys):
            pts = p.get("poly", [])
            if pts and self._point_in_polygon(wx, wy, pts):
                self._map_selected_obs_idx = i
                self._map_selected_obs_type = 'poly'
                self._map_fill_obs_edit_fields(p, 'poly')
                self._redraw_map_static()
                return

        # No se encontró nada
        self._map_selected_obs_idx = None
        self._map_selected_obs_type = None
        if hasattr(self, '_map_obs_edit_label'):
            self._map_obs_edit_label.configure(text="No se encontró obstáculo")
        self._redraw_map_static()

    def _point_in_polygon(self, x, y, poly):
        """Verifica si un punto está dentro de un polígono (ray casting)."""
        n = len(poly)
        inside = False
        j = n - 1
        for i in range(n):
            xi, yi = poly[i]
            xj, yj = poly[j]
            if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi) + xi):
                inside = not inside
            j = i
        return inside

    def _map_fill_obs_edit_fields(self, obs, obs_type):
        """Rellena los campos de edición con los datos del obstáculo."""
        if not hasattr(self, '_map_obs_edit_x'):
            return

        self._map_obs_edit_x.delete(0, tk.END)
        self._map_obs_edit_y.delete(0, tk.END)
        self._map_obs_edit_size.delete(0, tk.END)

        if obs_type == 'circle':
            cx = obs.get('cx', 0)
            cy = obs.get('cy', 0)
            r = obs.get('r', 30)
            self._map_obs_edit_x.insert(0, str(int(cx)))
            self._map_obs_edit_y.insert(0, str(int(cy)))
            self._map_obs_edit_size.insert(0, str(int(r)))
            self._map_obs_edit_label.configure(text=f"Editando ⭕ #{self._map_selected_obs_idx+1}")
        elif obs_type == 'poly':
            pts = obs.get('poly', [])
            if pts:
                cx = sum(p[0] for p in pts) / len(pts)
                cy = sum(p[1] for p in pts) / len(pts)
                self._map_obs_edit_x.insert(0, str(int(cx)))
                self._map_obs_edit_y.insert(0, str(int(cy)))
                self._map_obs_edit_size.insert(0, "—")
            self._map_obs_edit_label.configure(text=f"Editando ⬡ #{self._map_selected_obs_idx+1} (solo mover)")

    def _map_apply_obs_edit(self):
        """Aplica los cambios al obstáculo seleccionado en el mapa."""
        if self._map_selected_obs_idx is None or self._map_selected_obs_type is None:
            return

        idx = self._map_selected_obs_idx
        obs_type = self._map_selected_obs_type

        try:
            new_x = int(self._map_obs_edit_x.get())
            new_y = int(self._map_obs_edit_y.get())
            size_str = self._map_obs_edit_size.get()
            new_size = int(size_str) if size_str != "—" else None
        except ValueError:
            return

        if obs_type == 'circle':
            if idx < len(self._excl_circles):
                self._excl_circles[idx]['cx'] = new_x
                self._excl_circles[idx]['cy'] = new_y
                if new_size is not None:
                    self._excl_circles[idx]['r'] = new_size
                print(f"[MAP] Círculo #{idx+1} actualizado")
        elif obs_type == 'poly':
            if idx < len(self._excl_polys):
                pts = self._excl_polys[idx].get('poly', [])
                if pts:
                    old_cx = sum(p[0] for p in pts) / len(pts)
                    old_cy = sum(p[1] for p in pts) / len(pts)
                    dx = new_x - old_cx
                    dy = new_y - old_cy
                    self._excl_polys[idx]['poly'] = [(p[0] + dx, p[1] + dy) for p in pts]
                    print(f"[MAP] Polígono #{idx+1} movido")

        # Sincronizar con backend
        self._reapply_exclusions_to_backend()
        self._redraw_map_static()

    def _map_delete_selected_obs(self):
        """Elimina el obstáculo seleccionado del mapa."""
        if self._map_selected_obs_idx is None or self._map_selected_obs_type is None:
            return

        idx = self._map_selected_obs_idx
        obs_type = self._map_selected_obs_type

        if obs_type == 'circle':
            if idx < len(self._excl_circles):
                del self._excl_circles[idx]
                print(f"[MAP] Círculo #{idx+1} eliminado")
        elif obs_type == 'poly':
            if idx < len(self._excl_polys):
                del self._excl_polys[idx]
                print(f"[MAP] Polígono #{idx+1} eliminado")

        self._map_selected_obs_idx = None
        self._map_selected_obs_type = None
        if hasattr(self, '_map_obs_edit_label'):
            self._map_obs_edit_label.configure(text="Clic en ✋ y luego en obstáculo")
        if hasattr(self, '_map_obs_edit_x'):
            self._map_obs_edit_x.delete(0, tk.END)
            self._map_obs_edit_y.delete(0, tk.END)
            self._map_obs_edit_size.delete(0, tk.END)

        # Sincronizar con backend
        self._reapply_exclusions_to_backend()
        self._redraw_map_static()

    def _map_duplicate_obs_to_layer(self):
        """Duplica el obstáculo seleccionado a las capas marcadas en los checkboxes."""
        if self._map_selected_obs_idx is None or self._map_selected_obs_type is None:
            messagebox.showinfo("Sin selección", "Primero selecciona un obstáculo con ✋")
            return

        # Obtener zmin/zmax y lista de capas de los checkboxes seleccionados
        zmin, zmax = self._get_layer_z_range()
        selected_layers = self._get_selected_layers()

        idx = self._map_selected_obs_idx
        obs_type = self._map_selected_obs_type

        if obs_type == 'circle':
            if idx < len(self._excl_circles):
                orig = self._excl_circles[idx]
                new_obs = {
                    'cx': orig['cx'],
                    'cy': orig['cy'],
                    'r': orig['r'],
                    'zmin': zmin,
                    'zmax': zmax,
                    'layers': selected_layers,
                    'nombre': orig.get('nombre', '')
                }
                self._excl_circles.append(new_obs)
                print(f"[MAP] Círculo duplicado a capas {selected_layers}")

                # Sincronizar con backend
                if hasattr(self.dron, "add_exclusion_circle"):
                    self.dron.add_exclusion_circle(
                        cx=new_obs['cx'], cy=new_obs['cy'], r_cm=new_obs['r'],
                        z_min_cm=zmin, z_max_cm=zmax
                    )

        elif obs_type == 'poly':
            if idx < len(self._excl_polys):
                orig = self._excl_polys[idx]
                new_obs = {
                    'poly': list(orig['poly']),  # Copia de los puntos
                    'zmin': zmin,
                    'zmax': zmax,
                    'layers': selected_layers,
                    'nombre': orig.get('nombre', '')
                }
                self._excl_polys.append(new_obs)
                print(f"[MAP] Polígono duplicado a capas {selected_layers}")

                # Sincronizar con backend
                if hasattr(self.dron, "add_exclusion_polygon"):
                    self.dron.add_exclusion_polygon(
                        points=new_obs['poly'],
                        z_min_cm=zmin, z_max_cm=zmax
                    )

        self._redraw_map_static()
        self._hud_show(f"Duplicado a {zmin}-{zmax}cm", 1.5)

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

            # Sincronizar también con _mission_geofence para el Editor de Misiones
            self._mission_geofence = {
                'x1': gf_x1, 'y1': gf_y1,
                'x2': gf_x2, 'y2': gf_y2,
                'zmin': int(self.gf_zmin_var.get() or 0),
                'zmax': int(self.gf_zmax_var.get() or 200)
            }

            width_x = abs(x2 - x1)
            width_y = abs(y2 - y1)

            # Limpiar
            self._gf_pts.clear()
            self._tool_var.set("none")
            self.map_canvas.delete("gf_temp")

            self._hud_show(f"Zona definida: {width_x:.0f}x{width_y:.0f} cm", 2.0)
            self._redraw_map_static()

            # Redibujar también el editor de misiones si está abierto
            if hasattr(self, '_mission_canvas') and self._mission_canvas and self._mission_canvas.winfo_exists():
                self._draw_mission_map()

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

        print(f"[DEBUG _save_template] Círculos: {len(self._excl_circles)}")
        print(f"[DEBUG _save_template] Polígonos: {len(self._excl_polys)}")
        for i, p in enumerate(self._excl_polys):
            print(f"  Poly {i}: {p}")

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
            print(f"[DEBUG _load_template] Círculos cargados: {len(self._excl_circles)}")
            print(f"[DEBUG _load_template] Polígonos cargados: {len(self._excl_polys)}")
            for i, p in enumerate(self._excl_polys):
                print(f"  Poly {i}: {p}")

            try:
                self._incl_rect = data.get("inclusion")
                if hasattr(self, '_incl_zmin_var'):
                    self._incl_zmin_var.set(data.get("zmin", 0))
                if hasattr(self, '_incl_zmax_var'):
                    self._incl_zmax_var.set(data.get("zmax", 120))

                if self._incl_rect:
                    x1, y1, x2, y2 = self._incl_rect
                    if hasattr(self, 'gf_x1_var'):
                        self.gf_x1_var.set(str(int(x1)))
                    if hasattr(self, 'gf_y1_var'):
                        self.gf_y1_var.set(str(int(y1)))
                    if hasattr(self, 'gf_x2_var'):
                        self.gf_x2_var.set(str(int(x2)))
                    if hasattr(self, 'gf_y2_var'):
                        self.gf_y2_var.set(str(int(y2)))
            except:
                pass

            try:
                layers_config = data.get("layers", {})
                if layers_config and hasattr(self, '_layer1_max_var'):
                    self._layer1_max_var.set(layers_config.get("c1_max", 60))
                    self._layer2_max_var.set(layers_config.get("c2_max", 120))
                    self._layer3_max_var.set(layers_config.get("c3_max", 200))
                    self._apply_layers()
            except:
                pass

            try:
                self._reapply_exclusions_to_backend()
            except:
                pass

            # Redibujar ambos mapas si existen
            try:
                if hasattr(self, 'map_canvas') and self.map_canvas and self.map_canvas.winfo_exists():
                    self._redraw_map_static()
                    self.map_canvas.update_idletasks()
            except:
                pass
            try:
                if hasattr(self, '_mission_canvas') and self._mission_canvas and self._mission_canvas.winfo_exists():
                    self._draw_mission_map()
                    self._mission_canvas.update_idletasks()
            except:
                pass

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

        # Comprobar si geofence está activo
        gf_enabled = getattr(self.dron, "_gf_enabled", False)

        if gf_enabled:
            # Activo: verde fuerte, línea gruesa
            self.map_canvas.create_rectangle(p1x, p1y, p2x, p2y, outline="#00aa00", width=3, tags=("inclusion",))
        else:
            # Desactivado (standby): verde tenue, línea fina punteada
            self.map_canvas.create_rectangle(p1x, p1y, p2x, p2y, outline="#88cc88", width=1, dash=(4, 4), tags=("inclusion",))

    def _update_layer_indicator(self, z_cm):
        """Actualiza el indicador de capa en el mapa."""
        if not self._layer_label:
            return

        # Si el dron no está volando (altura <= 5cm), no sobrescribir el indicador
        # de capa de trabajo seleccionada
        if z_cm is None or z_cm <= 5:
            return

        # Obtener capa actual usando el método del dron
        if hasattr(self.dron, "get_current_layer"):
            layer = self.dron.get_current_layer(z_cm)
        else:
            # Fallback: calcular localmente
            layer = self._calculate_layer(z_cm)

        # Obtener rango de altura de la capa actual
        if hasattr(self.dron, "get_layers"):
            layers = self.dron.get_layers()
            if 0 < layer <= len(layers):
                layer_info = layers[layer - 1]
                z_range = f"({layer_info['z_min']:.0f}-{layer_info['z_max']:.0f}cm)"
            else:
                z_range = ""
        else:
            z_range = ""

        # Color según la capa
        colors = {
            1: "#28a745",  # Verde - capa baja
            2: "#fd7e14",  # Naranja - capa media
            3: "#007bff",  # Azul - capa alta
        }
        bg_color = colors.get(layer, "#333333")

        # Siempre actualizar el texto con la Z actual
        z_str = f"{z_cm:.0f}" if z_cm is not None else "--"
        self._layer_label.config(
            text=f"Capa {layer} {z_range}\nAltura: {z_str} cm",
            bg=bg_color
        )

        # Solo redibujar mapa y mostrar HUD cuando cambia de capa
        if layer != self._current_layer:
            self._last_layer = self._current_layer
            self._current_layer = layer

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
        # Si el obstáculo tiene el campo 'layers' explícito, usarlo directamente
        if 'layers' in exclusion and exclusion['layers']:
            return list(exclusion['layers'])

        # Intentar usar el método del dron
        if hasattr(self.dron, "get_exclusion_layers"):
            return self.dron.get_exclusion_layers(exclusion)

        # Fallback: calcular localmente desde zmin/zmax
        excl_zmin = exclusion.get("zmin")
        excl_zmax = exclusion.get("zmax")

        # Si no tiene límites, ocupa todas las capas
        if excl_zmin is None and excl_zmax is None:
            return [1, 2, 3]

        excl_zmin = float(excl_zmin) if excl_zmin is not None else 0.0
        excl_zmax = float(excl_zmax) if excl_zmax is not None else 999.0

        # Leer rangos de los sliders si existen
        c1_max = getattr(self, '_layer1_max_var', None)
        c2_max = getattr(self, '_layer2_max_var', None)
        c3_max = getattr(self, '_layer3_max_var', None)

        if c1_max and c2_max and c3_max:
            layers_ranges = [
                (0, c1_max.get()),           # Capa 1
                (c1_max.get(), c2_max.get()), # Capa 2
                (c2_max.get(), c3_max.get())  # Capa 3
            ]
        else:
            # Fallback a valores por defecto
            layers_ranges = [
                (0, 60),    # Capa 1
                (60, 120),  # Capa 2
                (120, 200)  # Capa 3
            ]

        result = []
        for i, (layer_zmin, layer_zmax) in enumerate(layers_ranges):
            # Hay solapamiento si los rangos se intersectan (límites exclusivos)
            # Un obstáculo en la frontera exacta (ej: zmax=60 con layer2 zmin=60) NO solapa
            if excl_zmin < layer_zmax and excl_zmax > layer_zmin:
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
        self._gallery_media_lock = threading.Lock()  # Lock para acceso seguro

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

        # Sección NAVEGACIÓN JERÁRQUICA
        nav_card = tk.Frame(left_panel, bg=BG_CARD)
        nav_card.pack(fill="both", expand=True, padx=4, pady=4)

        tk.Label(nav_card, text="  NAVEGACIÓN", font=("Arial", 9, "bold"),
                 bg=BG_HEADER, fg=FG_HEADER, anchor="w").pack(fill="x", ipady=4)

        # Treeview jerárquico: Escenario → Plan/Manual → Sesiones
        tree_container = tk.Frame(nav_card, bg=BG_CARD)
        tree_container.pack(fill="both", expand=True, padx=4, pady=4)

        # Estilo para el Treeview
        style = ttk.Style()
        style.configure("Gallery.Treeview", rowheight=24, font=("Arial", 9))
        style.configure("Gallery.Treeview.Heading", font=("Arial", 9, "bold"))

        tree_scroll = tk.Scrollbar(tree_container, orient="vertical")
        self._gallery_tree = ttk.Treeview(tree_container, style="Gallery.Treeview",
                                           yscrollcommand=tree_scroll.set, selectmode="browse")
        tree_scroll.configure(command=self._gallery_tree.yview)

        self._gallery_tree.heading("#0", text="Estructura", anchor="w")
        self._gallery_tree.column("#0", width=220, stretch=True)

        tree_scroll.pack(side="right", fill="y")
        self._gallery_tree.pack(side="left", fill="both", expand=True)

        # Evento de selección
        self._gallery_tree.bind("<<TreeviewSelect>>", self._gallery_on_tree_select)

        # Sección FILTRAR CONTENIDO
        filter_card = tk.Frame(left_panel, bg=BG_CARD, bd=1, relief="solid")
        filter_card.pack(fill="x", padx=4, pady=(10, 4))

        tk.Label(filter_card, text="  FILTRAR CONTENIDO", font=("Arial", 9, "bold"),
                 bg=BG_HEADER, fg=FG_HEADER, anchor="w").pack(fill="x", ipady=4)

        filter_content = tk.Frame(filter_card, bg=BG_CARD)
        filter_content.pack(fill="x", padx=8, pady=8)

        # Filtro por tipo de media
        tk.Label(filter_content, text="Mostrar:", bg=BG_CARD, font=("Arial", 8)).pack(anchor="w", pady=(4, 0))
        tk.Radiobutton(filter_content, text="📁 Todo", variable=self._gallery_filter, value="all",
                       bg=BG_CARD, activebackground=BG_CARD, font=("Arial", 9),
                       command=self._gallery_apply_filter).pack(anchor="w")
        tk.Radiobutton(filter_content, text="📷 Solo fotos", variable=self._gallery_filter, value="photos",
                       bg=BG_CARD, activebackground=BG_CARD, font=("Arial", 9),
                       command=self._gallery_apply_filter).pack(anchor="w")
        tk.Radiobutton(filter_content, text="🎬 Solo vídeos", variable=self._gallery_filter, value="videos",
                       bg=BG_CARD, activebackground=BG_CARD, font=("Arial", 9),
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
        """Refresca el árbol jerárquico de sesiones."""
        # Limpiar árbol existente
        for item in self._gallery_tree.get_children():
            self._gallery_tree.delete(item)

        # Obtener datos
        scenarios = self._scenario_manager.list_scenarios()
        sessions = self._session_manager.list_sessions()

        # Mapear escenarios por ID
        scenario_map = {s['id']: s for s in scenarios}
        self._gallery_scenarios_data = {s['id']: s for s in scenarios}

        # Estructura: {escenario_id: {plan_id/None: [sessions]}}
        hierarchy = {}

        for session in sessions:
            esc_id = session.get("escenario_id") or "_sin_escenario_"
            plan_id = session.get("plan_id")
            tipo = session.get("tipo", "manual")

            if esc_id not in hierarchy:
                hierarchy[esc_id] = {"planes": {}, "manuales": []}

            if tipo == "plan" and plan_id:
                if plan_id not in hierarchy[esc_id]["planes"]:
                    hierarchy[esc_id]["planes"][plan_id] = []
                hierarchy[esc_id]["planes"][plan_id].append(session)
            else:
                hierarchy[esc_id]["manuales"].append(session)

        # Poblar el Treeview
        self._gallery_session_map = {}  # Para mapear iid a session_id

        for esc_id, data in hierarchy.items():
            # Nodo de escenario
            if esc_id == "_sin_escenario_":
                esc_name = "(Sin escenario)"
                esc_icon = "📦"
            else:
                esc_info = scenario_map.get(esc_id, {})
                esc_name = esc_info.get("nombre", esc_id)
                esc_icon = "📁"

            total_sessions = len(data["manuales"]) + sum(len(v) for v in data["planes"].values())
            esc_text = f"{esc_icon} {esc_name} ({total_sessions})"
            esc_node = self._gallery_tree.insert("", "end", text=esc_text, open=True,
                                                  tags=("escenario",))

            # Nodos de planes de vuelo
            for plan_id, plan_sessions in data["planes"].items():
                plan_text = f"📋 Plan: {plan_id} ({len(plan_sessions)})"
                plan_node = self._gallery_tree.insert(esc_node, "end", text=plan_text, open=False,
                                                       tags=("plan",))
                # Sesiones del plan
                for session in plan_sessions:
                    session_text = self._format_session_text(session)
                    session_iid = self._gallery_tree.insert(plan_node, "end", text=session_text,
                                                             tags=("session",))
                    self._gallery_session_map[session_iid] = session["id"]

            # Nodo de vuelos manuales
            if data["manuales"]:
                manual_text = f"🎮 Vuelos Manuales ({len(data['manuales'])})"
                manual_node = self._gallery_tree.insert(esc_node, "end", text=manual_text, open=False,
                                                         tags=("manual",))
                for session in data["manuales"]:
                    session_text = self._format_session_text(session)
                    session_iid = self._gallery_tree.insert(manual_node, "end", text=session_text,
                                                             tags=("session",))
                    self._gallery_session_map[session_iid] = session["id"]

    def _format_session_text(self, session):
        """Formatea el texto de una sesión para el árbol."""
        session_id = session["id"]
        photos = session.get("photos_count", 0)
        videos = session.get("videos_count", 0)

        if session_id == "legacy":
            return f"📦 Archivos antiguos ({photos}📷 {videos}🎬)"

        try:
            parts = session_id.split("_")
            date_str = parts[0]
            time_str = parts[1].replace("-", ":") if len(parts) > 1 else ""
            return f"📅 {date_str} {time_str} ({photos}📷 {videos}🎬)"
        except:
            return f"📅 {session_id} ({photos}📷 {videos}🎬)"

    def _gallery_on_tree_select(self, event):
        """Maneja la selección en el árbol jerárquico."""
        selection = self._gallery_tree.selection()
        if not selection:
            return

        selected_iid = selection[0]
        tags = self._gallery_tree.item(selected_iid, "tags")

        if "session" in tags:
            # Es una sesión - cargar media
            session_id = self._gallery_session_map.get(selected_iid)
            if session_id:
                self._gallery_selected_session.set(session_id)
                self._gallery_load_media(session_id)
        elif "plan" in tags or "manual" in tags:
            # Es un nodo de plan/manual - mostrar todas las sesiones de ese grupo
            self._gallery_load_group_media(selected_iid)
        elif "escenario" in tags:
            # Es un escenario - mostrar todas las sesiones del escenario
            self._gallery_load_scenario_media(selected_iid)

    def _gallery_load_group_media(self, group_iid):
        """Carga media de todas las sesiones de un grupo (plan o manual)."""
        all_media = []
        children = self._gallery_tree.get_children(group_iid)

        for child_iid in children:
            session_id = self._gallery_session_map.get(child_iid)
            if session_id:
                filter_type = self._gallery_filter.get()
                media = self._session_manager.get_session_media(session_id, filter_type)
                all_media.extend(media)

        # Ordenar por fecha
        all_media.sort(key=lambda x: x.get("modified_time", ""), reverse=True)

        group_text = self._gallery_tree.item(group_iid, "text")
        self._gallery_header.configure(text=f"{group_text}  •  {len(all_media)} archivos")

        with self._gallery_media_lock:
            self._gallery_media_list = all_media

        self._gallery_thumbnails.clear()
        for widget in self._thumb_frame.winfo_children():
            widget.destroy()

        if not all_media:
            tk.Label(self._thumb_frame, text="No hay archivos en este grupo",
                     font=("Arial", 10, "italic"), fg="#888", bg="#ffffff").pack(pady=50)
            return

        self._create_thumbnail_grid()

    def _gallery_load_scenario_media(self, esc_iid):
        """Carga media de todas las sesiones de un escenario."""
        all_media = []

        def collect_sessions(parent_iid):
            for child_iid in self._gallery_tree.get_children(parent_iid):
                tags = self._gallery_tree.item(child_iid, "tags")
                if "session" in tags:
                    session_id = self._gallery_session_map.get(child_iid)
                    if session_id:
                        filter_type = self._gallery_filter.get()
                        media = self._session_manager.get_session_media(session_id, filter_type)
                        all_media.extend(media)
                else:
                    collect_sessions(child_iid)

        collect_sessions(esc_iid)

        # Ordenar por fecha
        all_media.sort(key=lambda x: x.get("modified_time", ""), reverse=True)

        esc_text = self._gallery_tree.item(esc_iid, "text")
        self._gallery_header.configure(text=f"{esc_text}  •  {len(all_media)} archivos")

        with self._gallery_media_lock:
            self._gallery_media_list = all_media

        self._gallery_thumbnails.clear()
        for widget in self._thumb_frame.winfo_children():
            widget.destroy()

        if not all_media:
            tk.Label(self._thumb_frame, text="No hay archivos en este escenario",
                     font=("Arial", 10, "italic"), fg="#888", bg="#ffffff").pack(pady=50)
            return

        self._create_thumbnail_grid()

    def _gallery_load_media(self, session_id):
        """Carga los archivos multimedia de una sesión."""
        filter_type = self._gallery_filter.get()
        with self._gallery_media_lock:
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
                    img.close()  # Liberar memoria de PIL
                    label = tk.Label(thumb_frame, image=photo, bg="#eee", bd=2, relief="solid")
                    label.image = photo
                else:
                    label = tk.Label(thumb_frame, text="🎬", font=("Arial", 40),
                                     width=8, height=4, bg="#333", fg="white", bd=2, relief="solid")
            except Exception:
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

        # Videos: abrir con visor de Windows
        if media["type"] == "video":
            self._open_external(media["path"])
            return

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
        # Liberar imagen anterior para evitar memory leak
        if hasattr(self._viewer_label, 'image') and self._viewer_label.image:
            del self._viewer_label.image
        self._viewer_label.configure(image=photo, text="")
        self._viewer_label.image = photo
        # Cerrar imagen PIL
        img.close()

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

        # Liberar el archivo si es video (para que Windows permita borrarlo)
        self._stop_video_playback()

        with self._gallery_media_lock:
            if self._viewer_index < 0 or self._viewer_index >= len(self._gallery_media_list):
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
            else:
                messagebox.showerror("Error", "No se pudo eliminar el archivo")

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
                    if getattr(self, "_mission_canvas", None) is None or not self._mission_canvas.winfo_exists():
                        self._close_mission_window()
                    else:
                        self._mission_win.lift()
                        return
            except Exception:
                self._close_mission_window()

        self._mission_win = tk.Toplevel(self.root)
        self._mission_win.title("Editor de Misiones")
        self._mission_win.geometry(f"{MAP_SIZE_PX + 360}x{MAP_SIZE_PX + 50}")
        self._mission_win.protocol("WM_DELETE_WINDOW", self._close_mission_window)

        # Estado de la misión
        self._mission_waypoints = []  # Lista de waypoints con acciones
        self._mission_tool = tk.StringVar(value="waypoint")
        self._mission_exclusions = []  # Círculos: {'type': 'circle', 'cx', 'cy', 'r'}
        self._mission_rect_points = []  # Puntos temporales para rectángulo
        self._mission_poly_points = []  # Puntos temporales para polígono
        self._mission_geofence = None  # Geofence propio
        self._mission_template_name = None
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

        side_canvas = tk.Canvas(side_container, width=320, highlightthickness=0)
        scrollbar = tk.Scrollbar(side_container, orient="vertical", command=side_canvas.yview)
        side_panel = tk.Frame(side_canvas)

        side_panel.bind("<Configure>", lambda e: side_canvas.configure(scrollregion=side_canvas.bbox("all")))
        side_canvas.create_window((0, 0), window=side_panel, anchor="nw")
        side_canvas.configure(yscrollcommand=scrollbar.set)

        def _on_mousewheel_mission(event):
            side_canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")

        # Bind mousewheel solo cuando el cursor está sobre el panel (no global)
        def _bind_mousewheel(event):
            side_canvas.bind_all("<MouseWheel>", _on_mousewheel_mission)
        def _unbind_mousewheel(event):
            side_canvas.unbind_all("<MouseWheel>")

        side_canvas.bind("<Enter>", _bind_mousewheel)
        side_canvas.bind("<Leave>", _unbind_mousewheel)
        side_panel.bind("<Enter>", _bind_mousewheel)
        side_panel.bind("<Leave>", _unbind_mousewheel)

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

        # Selector de capa para obstáculos con checkboxes (permite combinaciones libres)
        layer_colors = ["#28a745", "#fd7e14", "#007bff"]  # Verde, Naranja, Azul
        self._mission_layer_c1 = tk.BooleanVar(value=True)
        self._mission_layer_c2 = tk.BooleanVar(value=False)
        self._mission_layer_c3 = tk.BooleanVar(value=False)

        layer_row = tk.Frame(tools_content, bg=BG_CARD)
        layer_row.pack(fill="x", pady=2)
        tk.Label(layer_row, text="Capas:", bg=BG_CARD, font=("Arial", 8)).pack(side="left")

        # Checkboxes para capas
        tk.Checkbutton(layer_row, text="C1", variable=self._mission_layer_c1,
                       bg=layer_colors[0], fg="white", selectcolor=layer_colors[0],
                       activebackground=layer_colors[0], font=("Arial", 8, "bold"),
                       indicatoron=0, width=3, command=self._on_mission_layer_change).pack(side="left", padx=1)
        tk.Checkbutton(layer_row, text="C2", variable=self._mission_layer_c2,
                       bg=layer_colors[1], fg="white", selectcolor=layer_colors[1],
                       activebackground=layer_colors[1], font=("Arial", 8, "bold"),
                       indicatoron=0, width=3, command=self._on_mission_layer_change).pack(side="left", padx=1)
        tk.Checkbutton(layer_row, text="C3", variable=self._mission_layer_c3,
                       bg=layer_colors[2], fg="white", selectcolor=layer_colors[2],
                       activebackground=layer_colors[2], font=("Arial", 8, "bold"),
                       indicatoron=0, width=3, command=self._on_mission_layer_change).pack(side="left", padx=1)

        # Botón "Todas" para marcar/desmarcar todas
        def toggle_mission_layers():
            all_on = self._mission_layer_c1.get() and self._mission_layer_c2.get() and self._mission_layer_c3.get()
            self._mission_layer_c1.set(not all_on)
            self._mission_layer_c2.set(not all_on)
            self._mission_layer_c3.set(not all_on)
            self._on_mission_layer_change()

        tk.Button(layer_row, text="ALL", command=toggle_mission_layers,
                  bg="#6c757d", fg="white", font=("Arial", 8, "bold"), bd=0, width=4).pack(side="left", padx=1)

        # Inicializar indicador con la capa de trabajo seleccionada
        self._on_mission_layer_change()

        # Botones para polígonos y eliminar
        obs_btn_row = tk.Frame(tools_content, bg=BG_CARD)
        obs_btn_row.pack(fill="x", pady=2)
        tk.Button(obs_btn_row, text="✓ Cerrar ⬡", command=self._close_mission_polygon,
                  bg="#6c757d", fg="white", font=("Arial", 8), bd=0).pack(side="left", padx=2)
        tk.Button(obs_btn_row, text="🗑 Último obs", command=self._delete_last_obstacle,
                  bg="#dc3545", fg="white", font=("Arial", 8), bd=0).pack(side="left", padx=2)

        # ═══════════════════════════════════════════════════════════════════
        # SECCIÓN: ESCENARIO
        # ═══════════════════════════════════════════════════════════════════
        card_scenario = tk.Frame(side_panel, bg=BG_CARD, bd=1, relief="solid")
        card_scenario.pack(fill="x", padx=4, pady=4)

        tk.Label(card_scenario, text="  ESCENARIO", font=("Arial", 9, "bold"),
                 bg=BG_HEADER, fg=FG_HEADER, anchor="w").pack(fill="x", ipady=4)

        scenario_content = tk.Frame(card_scenario, bg=BG_CARD)
        scenario_content.pack(fill="x", padx=8, pady=8)

        # Combo para seleccionar escenario
        scenario_row1 = tk.Frame(scenario_content, bg=BG_CARD)
        scenario_row1.pack(fill="x", pady=2)
        tk.Label(scenario_row1, text="Escenario:", bg=BG_CARD, font=("Arial", 8)).pack(side="left")

        self._scenario_combo_var = tk.StringVar(value="")
        self._scenario_combo = ttk.Combobox(scenario_row1, textvariable=self._scenario_combo_var,
                                            width=18, state="readonly")
        self._scenario_combo.pack(side="left", padx=4)
        self._scenario_combo.bind("<<ComboboxSelected>>", self._on_scenario_selected)

        # Botón refrescar lista
        tk.Button(scenario_row1, text="↻", command=self._refresh_scenario_list,
                  bg="#6c757d", fg="white", font=("Arial", 8), bd=0, width=2).pack(side="left", padx=2)

        # Botones de escenario: Cargar y Guardar
        scenario_btns = tk.Frame(scenario_content, bg=BG_CARD)
        scenario_btns.pack(fill="x", pady=2)
        tk.Button(scenario_btns, text="📂 Cargar", command=self._load_scenario_from_file,
                  bg="#17a2b8", fg="white", font=("Arial", 8), bd=0).pack(side="left", fill="x", expand=True, padx=2)
        tk.Button(scenario_btns, text="💾 Guardar", command=self._save_scenario_from_editor,
                  bg="#28a745", fg="white", font=("Arial", 8), bd=0).pack(side="left", fill="x", expand=True, padx=2)

        # Nombre del escenario actual
        self._scenario_name_var = tk.StringVar(value="(ninguno)")
        tk.Label(scenario_content, textvariable=self._scenario_name_var, bg=BG_CARD,
                 font=("Arial", 8, "italic"), fg="#666").pack(anchor="w", pady=2)

        # Separador visual
        tk.Frame(scenario_content, height=1, bg="#ccc").pack(fill="x", pady=4)

        # Combo para seleccionar plan de vuelo (del escenario seleccionado)
        plan_row = tk.Frame(scenario_content, bg=BG_CARD)
        plan_row.pack(fill="x", pady=2)
        tk.Label(plan_row, text="Plan:", bg=BG_CARD, font=("Arial", 8)).pack(side="left")

        self._plan_combo_var = tk.StringVar(value="")
        self._plan_combo = ttk.Combobox(plan_row, textvariable=self._plan_combo_var,
                                        width=18, state="readonly")
        self._plan_combo.pack(side="left", padx=4)
        self._plan_combo.bind("<<ComboboxSelected>>", self._on_plan_selected)

        # Mini-canvas de preview del plan
        self._plan_preview_canvas = tk.Canvas(scenario_content, width=120, height=80,
                                               bg="#f8f9fa", highlightthickness=1,
                                               highlightbackground="#dee2e6")
        self._plan_preview_canvas.pack(pady=4)
        self._plan_preview_canvas.create_text(60, 40, text="Sin plan", fill="#aaa",
                                               font=("Arial", 8), tags="placeholder")

        # Botones de plan: Cargar y Guardar
        plan_btns = tk.Frame(scenario_content, bg=BG_CARD)
        plan_btns.pack(fill="x", pady=2)
        tk.Button(plan_btns, text="📂 Cargar Plan", command=self._load_selected_plan,
                  bg="#6f42c1", fg="white", font=("Arial", 8), bd=0).pack(side="left", fill="x", expand=True, padx=2)
        tk.Button(plan_btns, text="💾 Guardar Plan", command=self._save_plan_to_scenario,
                  bg="#fd7e14", fg="white", font=("Arial", 8), bd=0).pack(side="left", fill="x", expand=True, padx=2)

        # Refrescar lista al inicio
        self._refresh_scenario_list()

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
        # SECCIÓN: GEOFENCE (zona segura) - Versión completa
        # ═══════════════════════════════════════════════════════════════════
        card_gf = tk.Frame(side_panel, bg=BG_CARD, bd=1, relief="solid")
        card_gf.pack(fill="x", padx=4, pady=4)

        tk.Label(card_gf, text="  GEOFENCE (zona segura)", font=("Arial", 9, "bold"),
                 bg=BG_HEADER, fg=FG_HEADER, anchor="w").pack(fill="x", ipady=4)

        content_gf = tk.Frame(card_gf, bg=BG_CARD)
        content_gf.pack(fill="x", padx=8, pady=8)

        # Botón para dibujar en el mapa
        tk.Button(content_gf, text="📍 Dibujar zona (2 clics)", command=self._mission_start_geofence_rect,
                  bg=ACCENT, fg="white", font=("Arial", 8), bd=0,
                  activebackground="#138496").pack(fill="x", pady=(0, 6))

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

        # ═══════════════════════════════════════════════════════════════════
        # SECCIÓN: CAPAS DE ALTITUD
        # ═══════════════════════════════════════════════════════════════════
        card_layers = tk.Frame(side_panel, bg=BG_CARD, bd=1, relief="solid")
        card_layers.pack(fill="x", padx=4, pady=4)

        tk.Label(card_layers, text="  CAPAS DE ALTITUD", font=("Arial", 9, "bold"),
                 bg=BG_HEADER, fg=FG_HEADER, anchor="w").pack(fill="x", ipady=4)

        layers_container = tk.Frame(card_layers, bg=BG_CARD)
        layers_container.pack(fill="x", padx=6, pady=6)

        # Colores de capas
        layer_colors = ["#28a745", "#fd7e14", "#007bff"]

        # Obtener valores actuales de las capas
        if hasattr(self.dron, "get_layers"):
            layers = self.dron.get_layers()
        else:
            layers = [
                {"z_min": 0, "z_max": 60},
                {"z_min": 60, "z_max": 120},
                {"z_min": 120, "z_max": 200},
            ]

        # Inicializar variables si no existen
        if self._layer1_max_var is None:
            self._layer1_max_var = tk.IntVar(value=int(layers[0]["z_max"]))
        if self._layer2_max_var is None:
            self._layer2_max_var = tk.IntVar(value=int(layers[1]["z_max"]))
        if self._layer3_max_var is None:
            self._layer3_max_var = tk.IntVar(value=int(layers[2]["z_max"]))

        # Variables para mostrar rangos
        if not hasattr(self, '_mission_layer1_range_var'):
            self._mission_layer1_range_var = tk.StringVar(value=f"0 - {layers[0]['z_max']} cm")
            self._mission_layer2_range_var = tk.StringVar(value=f"{layers[0]['z_max']} - {layers[1]['z_max']} cm")
            self._mission_layer3_range_var = tk.StringVar(value=f"{layers[1]['z_max']} - {layers[2]['z_max']} cm")

        MIN_GAP = 20

        def update_mission_ranges():
            c1 = self._layer1_max_var.get()
            c2 = self._layer2_max_var.get()
            c3 = self._layer3_max_var.get()
            self._mission_layer1_range_var.set(f"0 - {c1} cm")
            self._mission_layer2_range_var.set(f"{c1} - {c2} cm")
            self._mission_layer3_range_var.set(f"{c2} - {c3} cm")

            # Actualizar también el indicador de la esquina (usa checkboxes)
            if self._mission_layer_label:
                self._on_mission_layer_change()

        def on_mc1_change(*args):
            c1 = self._layer1_max_var.get()
            c2 = self._layer2_max_var.get()
            if c1 >= c2 - MIN_GAP:
                self._layer2_max_var.set(c1 + MIN_GAP)
            update_mission_ranges()

        def on_mc2_change(*args):
            c1 = self._layer1_max_var.get()
            c2 = self._layer2_max_var.get()
            c3 = self._layer3_max_var.get()
            if c2 <= c1 + MIN_GAP:
                self._layer1_max_var.set(c2 - MIN_GAP)
            if c2 >= c3 - MIN_GAP:
                self._layer3_max_var.set(c2 + MIN_GAP)
            update_mission_ranges()

        def on_mc3_change(*args):
            c2 = self._layer2_max_var.get()
            c3 = self._layer3_max_var.get()
            if c3 <= c2 + MIN_GAP:
                self._layer2_max_var.set(c3 - MIN_GAP)
            update_mission_ranges()

        # Capa 3 (arriba - azul)
        layer3_frame = tk.Frame(layers_container, bg=layer_colors[2], bd=0)
        layer3_frame.pack(fill="x", padx=2, pady=1)
        tk.Label(layer3_frame, text="C3", font=("Arial", 9, "bold"),
                 bg=layer_colors[2], fg="white", width=3).pack(side="left", padx=2)
        tk.Scale(layer3_frame, from_=100, to=300, orient="horizontal",
                 variable=self._layer3_max_var, length=90, sliderlength=18,
                 bg=layer_colors[2], fg="white", troughcolor=layer_colors[2],
                 highlightthickness=0, font=("Arial", 7), bd=0,
                 command=lambda v: on_mc3_change()).pack(side="left", padx=2)
        tk.Label(layer3_frame, textvariable=self._mission_layer3_range_var, font=("Arial", 7),
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
                 command=lambda v: on_mc2_change()).pack(side="left", padx=2)
        tk.Label(layer2_frame, textvariable=self._mission_layer2_range_var, font=("Arial", 7),
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
                 command=lambda v: on_mc1_change()).pack(side="left", padx=2)
        tk.Label(layer1_frame, textvariable=self._mission_layer1_range_var, font=("Arial", 7),
                 bg=layer_colors[0], fg="white", width=10).pack(side="right", padx=2)

        # Suelo
        suelo_frame = tk.Frame(layers_container, bg="#8B4513")
        suelo_frame.pack(fill="x", padx=2, pady=(4, 2))
        tk.Label(suelo_frame, text="═══ SUELO ═══", font=("Arial", 8, "bold"),
                 bg="#8B4513", fg="#FFE4C4").pack(fill="x", ipady=2)

        # Botón Aplicar
        tk.Button(layers_container, text="✓ Aplicar capas", command=self._apply_layers,
                  bg="#28a745", fg="white", font=("Arial", 8, "bold"), bd=0,
                  activebackground="#218838", cursor="hand2").pack(fill="x", padx=2, pady=(6, 4))

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
        self._action_video_start = tk.BooleanVar(value=False)
        self._action_video_stop = tk.BooleanVar(value=False)
        self._action_rotate = tk.BooleanVar(value=False)
        self._action_rotate_deg = tk.IntVar(value=90)
        self._action_wait = tk.BooleanVar(value=False)
        self._action_wait_sec = tk.IntVar(value=2)
        self._action_layer_up = tk.BooleanVar(value=False)
        self._action_layer_down = tk.BooleanVar(value=False)

        # Foto
        af1 = tk.Frame(actions_content, bg=BG_CARD)
        af1.pack(fill="x", pady=1)
        tk.Checkbutton(af1, text="📷 Tomar foto", variable=self._action_photo,
                       bg=BG_CARD, activebackground=BG_CARD, font=("Arial", 8),
                       command=self._update_wp_actions).pack(side="left")

        # Video (duración fija en este WP)
        af2 = tk.Frame(actions_content, bg=BG_CARD)
        af2.pack(fill="x", pady=1)
        self._cb_video = tk.Checkbutton(af2, text="🎬 Grabar", variable=self._action_video,
                       bg=BG_CARD, activebackground=BG_CARD, font=("Arial", 8),
                       command=self._on_video_mode_change)
        self._cb_video.pack(side="left")
        self._entry_video_dur = tk.Entry(af2, textvariable=self._action_video_duration, width=3)
        self._entry_video_dur.pack(side="left", padx=2)
        tk.Label(af2, text="seg", bg=BG_CARD, font=("Arial", 8)).pack(side="left")

        # Video Start/Stop (para videos entre waypoints)
        af2b = tk.Frame(actions_content, bg=BG_CARD)
        af2b.pack(fill="x", pady=1)
        self._cb_video_start = tk.Checkbutton(af2b, text="▶ Iniciar video", variable=self._action_video_start,
                       bg=BG_CARD, activebackground=BG_CARD, font=("Arial", 8), fg="#28a745",
                       command=self._on_video_mode_change)
        self._cb_video_start.pack(side="left")
        self._cb_video_stop = tk.Checkbutton(af2b, text="⏹ Parar video", variable=self._action_video_stop,
                       bg=BG_CARD, activebackground=BG_CARD, font=("Arial", 8), fg="#dc3545",
                       command=self._on_video_mode_change)
        self._cb_video_stop.pack(side="left", padx=(10, 0))

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

        # Cambiar de capa
        af5 = tk.Frame(actions_content, bg=BG_CARD)
        af5.pack(fill="x", pady=1)
        self._cb_layer_up = tk.Checkbutton(af5, text="⬆ Subir capa", variable=self._action_layer_up,
                       bg=BG_CARD, activebackground=BG_CARD, font=("Arial", 8), fg="#007bff",
                       command=self._on_layer_change)
        self._cb_layer_up.pack(side="left")
        self._cb_layer_down = tk.Checkbutton(af5, text="⬇ Bajar capa", variable=self._action_layer_down,
                       bg=BG_CARD, activebackground=BG_CARD, font=("Arial", 8), fg="#fd7e14",
                       command=self._on_layer_change)
        self._cb_layer_down.pack(side="left", padx=(10, 0))

        # ═══════════════════════════════════════════════════════════════════
        # SECCIÓN: EJECUTAR
        # ═══════════════════════════════════════════════════════════════════
        card_exec = tk.Frame(side_panel, bg=BG_CARD, bd=1, relief="solid")
        card_exec.pack(fill="x", padx=4, pady=4)

        tk.Label(card_exec, text="  EJECUTAR MISIÓN", font=("Arial", 9, "bold"),
                 bg=BG_HEADER, fg=FG_HEADER, anchor="w").pack(fill="x", ipady=4)

        exec_content = tk.Frame(card_exec, bg=BG_CARD)
        exec_content.pack(fill="x", padx=8, pady=8)

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

        # Crear label de capa AL FINAL de todo
        self._mission_layer_label = tk.Label(
            self._mission_win,
            text="Capa 1\n(0-60 cm)",
            font=("Arial", 12, "bold"),
            bg="#28a745",
            fg="white",
            padx=8,
            pady=4
        )
        self._mission_layer_label.place(x=20, y=20)

        # Dibujar mapa inicial
        self._draw_mission_map()

    def _close_mission_window(self):
        """Cierra el editor de misiones y limpia referencias."""
        try:
            if hasattr(self, "_mission_win") and self._mission_win:
                try:
                    self._mission_win.grab_release()
                except Exception:
                    pass
                try:
                    self._mission_win.destroy()
                except Exception:
                    pass
        finally:
            self._mission_win = None
            self._mission_canvas = None
            self._mission_layer_label = None
            self._mission_running = False

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

    def _on_mission_layer_change(self, *args):
        """Actualiza el indicador de capa cuando cambia la selección en Editor de Misiones (checkboxes)."""
        # Obtener capas seleccionadas desde checkboxes
        c1 = getattr(self, '_mission_layer_c1', None)
        c2 = getattr(self, '_mission_layer_c2', None)
        c3 = getattr(self, '_mission_layer_c3', None)

        selected = []
        if c1 and c1.get():
            selected.append(1)
        if c2 and c2.get():
            selected.append(2)
        if c3 and c3.get():
            selected.append(3)

        zmin, zmax = self._get_mission_layer_z_range()

        if self._mission_layer_label:
            # Determinar color según capas seleccionadas
            if len(selected) == 0:
                bg_color = "#6c757d"
                layer_text = "Todas las capas"
            elif len(selected) == 3:
                bg_color = "#6c757d"
                layer_text = "Todas las capas"
            elif len(selected) == 1:
                colors = {1: "#28a745", 2: "#fd7e14", 3: "#007bff"}
                bg_color = colors.get(selected[0], "#333333")
                layer_text = f"Capa {selected[0]}"
            else:
                bg_color = "#9b59b6"  # Púrpura para combinaciones
                layer_text = "Capas " + "+".join(str(s) for s in selected)

            self._mission_layer_label.config(text=f"{layer_text}\n({zmin}-{zmax} cm)", bg=bg_color)
        self._draw_mission_map()

    def _get_mission_layer_z_range(self):
        """Obtiene el rango Z para las capas seleccionadas en Editor de Misiones (checkboxes)."""
        # Leer de los sliders de la UI si existen
        c1_max = getattr(self, '_layer1_max_var', None)
        c2_max = getattr(self, '_layer2_max_var', None)
        c3_max = getattr(self, '_layer3_max_var', None)

        if c1_max and c2_max and c3_max:
            # Usar valores de los sliders
            layers = [
                {"z_min": 0, "z_max": c1_max.get()},
                {"z_min": c1_max.get(), "z_max": c2_max.get()},
                {"z_min": c2_max.get(), "z_max": c3_max.get()},
            ]
        elif hasattr(self.dron, "get_layers"):
            layers = self.dron.get_layers()
        else:
            layers = [
                {"z_min": 0, "z_max": 60},
                {"z_min": 60, "z_max": 120},
                {"z_min": 120, "z_max": 200},
            ]

        c1 = getattr(self, '_mission_layer_c1', None)
        c2 = getattr(self, '_mission_layer_c2', None)
        c3 = getattr(self, '_mission_layer_c3', None)

        selected = []
        if c1 and c1.get():
            selected.append(0)
        if c2 and c2.get():
            selected.append(1)
        if c3 and c3.get():
            selected.append(2)

        if not selected:
            return 0, int(layers[-1]["z_max"]) if layers else 200

        zmin = int(layers[min(selected)]["z_min"])
        zmax = int(layers[max(selected)]["z_max"])
        return zmin, zmax

    def _get_mission_selected_layers(self):
        """Devuelve la lista de capas seleccionadas (1, 2, 3) desde los checkboxes del Editor."""
        c1 = getattr(self, '_mission_layer_c1', None)
        c2 = getattr(self, '_mission_layer_c2', None)
        c3 = getattr(self, '_mission_layer_c3', None)

        selected = []
        if c1 and c1.get():
            selected.append(1)
        if c2 and c2.get():
            selected.append(2)
        if c3 and c3.get():
            selected.append(3)

        return selected if selected else [1, 2, 3]  # Si ninguna, todas

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

        # Dibujar geofence (igual que en Abrir Mapa: activo=sólido, desactivado=discontinuo)
        if self._mission_geofence:
            gf = self._mission_geofence
            x1, y1 = self._mission_world_to_canvas(gf['x1'], gf['y1'])
            x2, y2 = self._mission_world_to_canvas(gf['x2'], gf['y2'])

            # Comprobar si geofence está activo
            gf_enabled = getattr(self.dron, "_gf_enabled", False)

            if gf_enabled:
                # Activo: verde fuerte, línea gruesa
                self._mission_canvas.create_rectangle(x1, y1, x2, y2,
                                                      outline="#00aa00", fill="", width=3)
            else:
                # Desactivado (standby): verde tenue, línea fina punteada
                self._mission_canvas.create_rectangle(x1, y1, x2, y2,
                                                      outline="#88cc88", fill="", width=1, dash=(4, 4))

        # Obtener capas seleccionadas (checkboxes) para colorear obstáculos
        c1 = getattr(self, '_mission_layer_c1', None)
        c2 = getattr(self, '_mission_layer_c2', None)
        c3 = getattr(self, '_mission_layer_c3', None)
        selected_layers = []
        if c1 and c1.get():
            selected_layers.append(1)
        if c2 and c2.get():
            selected_layers.append(2)
        if c3 and c3.get():
            selected_layers.append(3)
        # Si no hay ninguna o están todas, mostrar todo
        show_all = len(selected_layers) == 0 or len(selected_layers) == 3

        # Colores para obstáculos (igual que Abrir mapa)
        COLOR_IN_LAYER = "#ff0000"      # Rojo - obstáculo en capa actual
        COLOR_OTHER_LAYER = "#ffcccc"   # Rojo claro - obstáculo en otra capa
        WIDTH_IN_LAYER = 3
        WIDTH_OTHER_LAYER = 1

        # Dibujar obstáculos
        for i, obs in enumerate(self._mission_exclusions):
            obs_type = obs.get('type', 'circle')

            # Determinar si está en alguna de las capas seleccionadas
            excl_layers = self._get_mission_exclusion_layers(obs)
            in_current_layer = show_all or any(l in excl_layers for l in selected_layers)

            # Color diferente si está seleccionado
            is_selected = (i == self._selected_obs_idx)
            if is_selected:
                outline_color = "#00ff00"  # Verde para seleccionado
                line_width = 4
            else:
                outline_color = COLOR_IN_LAYER if in_current_layer else COLOR_OTHER_LAYER
                line_width = WIDTH_IN_LAYER if in_current_layer else WIDTH_OTHER_LAYER

            # Calcular centro para el texto de capa
            text_cx, text_cy = 0, 0

            if obs_type == 'circle':
                cx, cy = self._mission_world_to_canvas(obs['cx'], obs['cy'])
                r = obs['r'] * PX_PER_CM
                self._mission_canvas.create_oval(cx - r, cy - r, cx + r, cy + r,
                                                  outline=outline_color, fill="", width=line_width)
                text_cx, text_cy = cx, cy
            elif obs_type == 'rect':
                x1, y1 = self._mission_world_to_canvas(obs['x1'], obs['y1'])
                x2, y2 = self._mission_world_to_canvas(obs['x2'], obs['y2'])
                self._mission_canvas.create_rectangle(x1, y1, x2, y2,
                                                       outline=outline_color, fill="", width=line_width)
                text_cx, text_cy = (x1 + x2) / 2, (y1 + y2) / 2
            elif obs_type == 'poly':
                points = []
                for px, py in obs['points']:
                    cx, cy = self._mission_world_to_canvas(px, py)
                    points.extend([cx, cy])
                if len(points) >= 6:
                    self._mission_canvas.create_polygon(points, outline=outline_color,
                                                         fill="", width=line_width)
                    # Centroide del polígono
                    pts = obs['points']
                    centroid_x = sum(p[0] for p in pts) / len(pts)
                    centroid_y = sum(p[1] for p in pts) / len(pts)
                    text_cx, text_cy = self._mission_world_to_canvas(centroid_x, centroid_y)

            # Mostrar en qué capas está el obstáculo
            if excl_layers:
                layers_text = "C" + ",".join(str(l) for l in excl_layers)
                self._mission_canvas.create_text(text_cx, text_cy, text=layers_text,
                                                  font=("Arial", 8, "bold"), fill=outline_color)

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
                if wp.get('layer_up'):
                    icons.append("⬆")
                if wp.get('layer_down'):
                    icons.append("⬇")
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

            # Actualizar indicador de capa/altura en tiempo real
            pose = getattr(self.dron, "pose", None)
            if pose:
                z = getattr(pose, "z_cm", None)
                self._update_mission_layer_indicator(z)
        except Exception:
            pass

    def _update_mission_layer_indicator(self, z_cm):
        """Actualiza el indicador de capa y altura en el editor de misiones."""
        if not hasattr(self, '_mission_layer_label') or not self._mission_layer_label:
            return

        # Si el dron no está volando (altura <= 5cm), no sobrescribir el indicador
        if z_cm is None or z_cm <= 5:
            return

        # Obtener capa actual usando el método del dron
        if hasattr(self.dron, "get_current_layer"):
            layer = self.dron.get_current_layer(z_cm)
        else:
            layer = self._calculate_layer(z_cm)

        # Obtener rango de altura de la capa actual
        z_range = ""
        if hasattr(self.dron, "get_layers"):
            layers = self.dron.get_layers()
            if 0 < layer <= len(layers):
                layer_info = layers[layer - 1]
                z_range = f"({layer_info['z_min']:.0f}-{layer_info['z_max']:.0f}cm)"

        # Color según la capa
        colors = {
            1: "#28a745",  # Verde - capa baja
            2: "#fd7e14",  # Naranja - capa media
            3: "#007bff",  # Azul - capa alta
        }
        bg_color = colors.get(layer, "#333333")

        # Actualizar el label con capa y altura actual
        z_str = f"{z_cm:.0f}" if z_cm is not None else "--"
        self._mission_layer_label.config(
            text=f"Capa {layer} {z_range}\nAltura: {z_str} cm",
            bg=bg_color
        )

        # Detectar cambio de capa y mostrar HUD
        if not hasattr(self, '_mission_current_layer'):
            self._mission_current_layer = 0

        if layer != self._mission_current_layer and self._mission_current_layer != 0:
            direction = "Subiendo" if layer > self._mission_current_layer else "Bajando"
            self._hud_show(f"{direction} a Capa {layer}", 2.0)

        self._mission_current_layer = layer

    def _on_mission_canvas_click(self, event):
        """Maneja clicks en el canvas de misiones."""
        wx, wy = self._mission_canvas_to_world(event.x, event.y)
        tool = self._mission_tool.get()

        if tool == "waypoint":
            # Obtener altura Z primero para validación 3D
            z = self._wp_default_z.get()

            # Validar límites de seguridad globales
            distance_from_origin = math.sqrt(wx * wx + wy * wy)
            if distance_from_origin > MISSION_MAX_DISTANCE_CM:
                messagebox.showwarning("Límite de seguridad",
                    f"El waypoint está a {distance_from_origin:.0f}cm del origen.\n"
                    f"Máximo permitido: {MISSION_MAX_DISTANCE_CM}cm")
                return

            if z > MISSION_MAX_HEIGHT_CM:
                messagebox.showwarning("Límite de seguridad",
                    f"Altura {z}cm excede el máximo permitido ({MISSION_MAX_HEIGHT_CM}cm)")
                return

            if z < MISSION_MIN_HEIGHT_CM:
                messagebox.showwarning("Límite de seguridad",
                    f"Altura {z}cm es menor que el mínimo seguro ({MISSION_MIN_HEIGHT_CM}cm)")
                return

            # Validar que está dentro del geofence
            if self._mission_geofence:
                gf = self._mission_geofence
                if not (gf['x1'] <= wx <= gf['x2'] and gf['y1'] <= wy <= gf['y2']):
                    messagebox.showwarning("Fuera de zona", "El waypoint está fuera del geofence")
                    return
                # Validar Z contra geofence
                gf_zmin = gf.get('zmin', 0)
                gf_zmax = gf.get('zmax', 200)
                if z < gf_zmin or z > gf_zmax:
                    messagebox.showwarning("Fuera de zona",
                        f"La altura {z}cm está fuera del geofence ({gf_zmin}-{gf_zmax}cm)")
                    return

            # Validar que no está en un obstáculo (con validación Z)
            for obs in self._mission_exclusions:
                if point_in_obstacle(wx, wy, obs, z=z):
                    obs_zmin = obs.get('zmin', 0)
                    obs_zmax = obs.get('zmax', 200)
                    messagebox.showwarning("Obstáculo",
                        f"El waypoint (z={z}cm) colisiona con obstáculo en capa {obs_zmin}-{obs_zmax}cm")
                    return
            wp = {
                'x': round(wx, 1),
                'y': round(wy, 1),
                'z': z,
                'photo': False,
                'video': False,
                'video_duration': 5,
                'video_start': False,   # Iniciar grabación continua
                'video_stop': False,    # Detener grabación continua
                'rotate': False,
                'rotate_deg': 0,
                'wait': False,
                'wait_sec': 0,
                'layer_up': False,
                'layer_down': False
            }
            self._mission_waypoints.append(wp)
            self._update_wp_listbox()
            self._draw_mission_map()

        elif tool == "obstacle":
            r = self._mission_obs_radius.get()
            zmin, zmax = self._get_mission_layer_z_range()
            selected_layers = self._get_mission_selected_layers()
            cx, cy = round(wx, 1), round(wy, 1)
            obs = {'type': 'circle', 'cx': cx, 'cy': cy, 'r': r, 'zmin': zmin, 'zmax': zmax, 'layers': selected_layers}
            self._mission_exclusions.append(obs)
            # Sincronizar con Abrir Mapa
            self._excl_circles.append({'cx': cx, 'cy': cy, 'r': r, 'zmin': zmin, 'zmax': zmax, 'layers': selected_layers})
            self._draw_mission_map()

        elif tool == "obs_rect":
            self._mission_rect_points.append((round(wx, 1), round(wy, 1)))
            if len(self._mission_rect_points) == 2:
                p1, p2 = self._mission_rect_points
                zmin, zmax = self._get_mission_layer_z_range()
                selected_layers = self._get_mission_selected_layers()
                x1, y1 = min(p1[0], p2[0]), min(p1[1], p2[1])
                x2, y2 = max(p1[0], p2[0]), max(p1[1], p2[1])
                obs = {
                    'type': 'rect',
                    'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2,
                    'zmin': zmin, 'zmax': zmax, 'layers': selected_layers
                }
                self._mission_exclusions.append(obs)
                # Sincronizar con Abrir Mapa (como polígono)
                rect_poly = [(x1, y1), (x2, y1), (x2, y2), (x1, y2)]
                self._excl_polys.append({'poly': rect_poly, 'zmin': zmin, 'zmax': zmax, 'layers': selected_layers})
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

        elif tool == "geofence_rect":
            # Dibujar geofence con 2 clics
            if not hasattr(self, '_mission_gf_pts'):
                self._mission_gf_pts = []

            self._mission_gf_pts.append((wx, wy))

            # Mostrar punto temporal
            cx_px, cy_px = self._mission_world_to_canvas(wx, wy)
            self._mission_canvas.create_oval(
                cx_px - 5, cy_px - 5, cx_px + 5, cy_px + 5,
                fill="#00aa00", outline="white", width=2, tags="gf_temp"
            )

            if len(self._mission_gf_pts) == 2:
                x1, y1 = self._mission_gf_pts[0]
                x2, y2 = self._mission_gf_pts[1]

                # Actualizar las variables de coordenadas (compartidas)
                self.gf_x1_var.set(str(int(min(x1, x2))))
                self.gf_y1_var.set(str(int(min(y1, y2))))
                self.gf_x2_var.set(str(int(max(x1, x2))))
                self.gf_y2_var.set(str(int(max(y1, y2))))

                # Guardar geofence local para misión
                gf_x1 = min(x1, x2)
                gf_y1 = min(y1, y2)
                gf_x2 = max(x1, x2)
                gf_y2 = max(y1, y2)

                self._mission_geofence = {
                    'x1': gf_x1, 'y1': gf_y1,
                    'x2': gf_x2, 'y2': gf_y2,
                    'zmin': int(self.gf_zmin_var.get() or 0),
                    'zmax': int(self.gf_zmax_var.get() or 200)
                }

                # Sincronizar también con _incl_rect para la ventana de Mapa
                self._incl_rect = (gf_x1, gf_y1, gf_x2, gf_y2)

                width_x = abs(x2 - x1)
                width_y = abs(y2 - y1)

                # Limpiar
                self._mission_gf_pts.clear()
                self._mission_tool.set("waypoint")
                self._mission_canvas.delete("gf_temp")

                self._hud_show(f"Zona definida: {width_x:.0f}x{width_y:.0f} cm", 2.0)
                self._draw_mission_map()

                # Redibujar también el mapa si está abierto
                if self._map_win and tk.Toplevel.winfo_exists(self._map_win):
                    self._redraw_map_static()

    def _mission_start_geofence_rect(self):
        """Inicia el dibujo de la zona de geofence en el mapa de misiones."""
        if not hasattr(self, '_mission_gf_pts'):
            self._mission_gf_pts = []
        self._mission_gf_pts.clear()
        self._mission_tool.set("geofence_rect")
        self._hud_show("Clic en 2 esquinas del geofence", 3.0)

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
        n_points = len(self._mission_poly_points)
        if n_points >= 3:
            zmin, zmax = self._get_mission_layer_z_range()
            selected_layers = self._get_mission_selected_layers()
            poly_points = list(self._mission_poly_points)
            obs = {'type': 'poly', 'points': poly_points, 'zmin': zmin, 'zmax': zmax, 'layers': selected_layers}
            self._mission_exclusions.append(obs)
            # Sincronizar con Abrir Mapa
            self._excl_polys.append({'poly': poly_points, 'zmin': zmin, 'zmax': zmax, 'layers': selected_layers})
            self._mission_poly_points.clear()
            self._draw_mission_map()
        elif n_points > 0:
            messagebox.showwarning("Polígono incompleto",
                f"Se necesitan al menos 3 puntos para un polígono.\nTienes {n_points} punto(s).")
        else:
            messagebox.showinfo("Sin polígono", "Primero dibuja puntos con la herramienta ⬡")

    def _get_mission_exclusion_layers(self, obs):
        """Determina en qué capas está un obstáculo."""
        # Si el obstáculo tiene el campo 'layers' explícito, usarlo directamente
        if 'layers' in obs and obs['layers']:
            return list(obs['layers'])

        # Fallback: calcular desde zmin/zmax
        zmin = obs.get('zmin', 0)
        zmax = obs.get('zmax', 200)

        # Obtener configuración de capas
        if hasattr(self.dron, "get_layers"):
            layers = self.dron.get_layers()
        else:
            layers = [
                {"z_min": 0, "z_max": 60},
                {"z_min": 60, "z_max": 120},
                {"z_min": 120, "z_max": 200},
            ]

        result = []
        for i, layer in enumerate(layers):
            layer_zmin = layer.get("z_min", 0)
            layer_zmax = layer.get("z_max", 200)
            # Si hay solapamiento entre el obstáculo y la capa
            if zmin < layer_zmax and zmax > layer_zmin:
                result.append(i + 1)

        return result if result else [1, 2, 3]  # Si no hay datos, mostrar "todas"

    def _get_mission_plan_id(self):
        """Devuelve el nombre de plan para la galería de vuelos."""
        plan_id = (self._mission_template_name or "").strip()
        return plan_id if plan_id else "mission_editor"

    def _delete_last_obstacle(self):
        """Elimina el último obstáculo añadido."""
        if self._mission_exclusions:
            self._mission_exclusions.pop()
            self._draw_mission_map()

    def _save_mission_template(self):
        """Guarda la plantilla del Editor de Misiones (solo waypoints y acciones)."""
        path = filedialog.asksaveasfilename(defaultextension=".json", filetypes=[("JSON", "*.json")])
        if not path:
            return
        template_name = os.path.splitext(os.path.basename(path))[0] or "mission_editor"

        data = {
            "type": "mission",  # Identificador para distinguir del otro tipo
            "name": template_name,
            "waypoints": self._mission_waypoints,
        }

        try:
            with open(path, "w") as f:
                json.dump(data, f, indent=2)
            self._mission_template_name = template_name
            messagebox.showinfo("Plantilla Misión", "Guardada correctamente.")
        except Exception as e:
            messagebox.showerror("Plantilla Misión", f"Error guardando: {e}")

    def _load_mission_template(self):
        """Carga la plantilla del Editor de Misiones."""
        path = filedialog.askopenfilename(filetypes=[("JSON", "*.json")])
        if not path:
            return
        template_name = os.path.splitext(os.path.basename(path))[0] or "mission_editor"

        try:
            with open(path, "r") as f:
                data = json.load(f)

            # Verificar que sea una plantilla de misión
            if data.get("type") != "mission":
                messagebox.showwarning("Plantilla Misión",
                    "Este archivo es una plantilla de 'Abrir mapa', no de 'Editor de misiones'.")
                return

            self._mission_waypoints = data.get("waypoints", [])
            self._mission_template_name = data.get("name", template_name)

            # Actualizar UI
            try:
                if hasattr(self, '_mission_canvas') and self._mission_canvas and self._mission_canvas.winfo_exists():
                    self._draw_mission_map()
                    self._mission_canvas.update_idletasks()
            except:
                pass

            try:
                self._update_wp_listbox()
            except:
                pass

            messagebox.showinfo("Plantilla Misión", "Cargada correctamente.")

        except Exception as e:
            messagebox.showerror("Plantilla Misión", f"Error cargando: {e}")

    # =========================================================================
    # MÉTODOS DE ESCENARIO
    # =========================================================================

    def _refresh_scenario_list(self):
        """Refresca la lista de escenarios disponibles."""
        scenarios = self._scenario_manager.list_scenarios()
        names = []
        self._scenario_display_map = {}
        for s in scenarios:
            display = f"{s['nombre']} ({s['id']})"
            names.append(display)
            self._scenario_display_map[display] = s['id']
        self._scenario_list = scenarios

        if hasattr(self, '_scenario_combo') and self._scenario_combo:
            self._scenario_combo['values'] = names
            if names and not self._scenario_combo_var.get():
                self._scenario_combo_var.set(names[0])
                # Cargar el escenario seleccionado automáticamente
                self._on_scenario_selected()

    def _on_scenario_selected(self, event=None):
        """Maneja selección de escenario en el combo - carga escenario y sus planes."""
        selected_name = self._scenario_combo_var.get()
        if not selected_name:
            return

        # Obtener ID del escenario
        scenario_id = None
        if hasattr(self, "_scenario_display_map"):
            scenario_id = self._scenario_display_map.get(selected_name)

        if not scenario_id:
            return

        # Cargar escenario (geofence, capas, obstáculos)
        scenario = self._scenario_manager.load_scenario(scenario_id)
        if scenario:
            self._apply_scenario(scenario, scenario_id=scenario_id)

            # Poblar dropdown de planes de este escenario
            self._populate_plans_combo(scenario_id)

    def _populate_plans_combo(self, scenario_id):
        """Pobla el combo de planes con los planes del escenario seleccionado."""
        if not hasattr(self, '_plan_combo'):
            return

        plans = self._scenario_manager.list_flight_plans(scenario_id)
        self._plan_list = plans
        self._plan_display_map = {}

        names = ["(nuevo plan)"]  # Opción para crear nuevo
        for p in plans:
            display = f"{p['nombre']} ({p['num_waypoints']} WPs)"
            names.append(display)
            self._plan_display_map[display] = p['id']

        self._plan_combo['values'] = names
        self._plan_combo_var.set(names[0] if names else "")

    def _on_plan_selected(self, event=None):
        """Maneja selección de plan en el combo - muestra preview."""
        self._draw_plan_preview()

    def _draw_plan_preview(self):
        """Dibuja un preview del plan seleccionado en el mini-canvas."""
        if not hasattr(self, '_plan_preview_canvas'):
            return

        canvas = self._plan_preview_canvas
        canvas.delete("all")

        selected = self._plan_combo_var.get()
        if not selected or selected == "(nuevo plan)":
            canvas.create_text(60, 40, text="Sin plan", fill="#aaa",
                              font=("Arial", 8))
            return

        # Obtener waypoints del plan
        if not hasattr(self, '_current_scenario_id') or not self._current_scenario_id:
            canvas.create_text(60, 40, text="Sin escenario", fill="#aaa",
                              font=("Arial", 8))
            return

        plan_id = getattr(self, '_plan_display_map', {}).get(selected)
        if not plan_id:
            return

        plan_data = self._scenario_manager.get_flight_plan(self._current_scenario_id, plan_id)
        if not plan_data or 'waypoints' not in plan_data:
            canvas.create_text(60, 40, text="Plan vacío", fill="#aaa",
                              font=("Arial", 8))
            return

        waypoints = plan_data['waypoints']
        if not waypoints:
            canvas.create_text(60, 40, text="Sin waypoints", fill="#aaa",
                              font=("Arial", 8))
            return

        # Calcular bounds para escalar (usando mismo sistema que Editor de Misiones)
        # En el editor: X mundo = arriba, Y mundo = derecha
        # En canvas: X canvas = derecha (basado en Y mundo), Y canvas = arriba (basado en X mundo invertido)
        xs = [wp.get('x', 0) for wp in waypoints]
        ys = [wp.get('y', 0) for wp in waypoints]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)

        # Añadir margen
        margin = 10
        w, h = 120 - 2*margin, 80 - 2*margin

        # Calcular escala (intercambiando rangos para que coincida con el editor)
        range_x = max_x - min_x if max_x != min_x else 1
        range_y = max_y - min_y if max_y != min_y else 1
        # El ancho del canvas usa range_y, el alto usa range_x
        scale = min(w / range_y, h / range_x)

        # Centrar
        offset_x = margin + (w - range_y * scale) / 2
        offset_y = margin + (h - range_x * scale) / 2

        def to_canvas(wx, wy):
            # Mismo sistema que _mission_world_to_canvas:
            # Y mundo → X canvas (derecha)
            # X mundo → Y canvas (arriba, invertido)
            cx = offset_x + (wy - min_y) * scale
            cy = offset_y + (max_x - wx) * scale
            return cx, cy

        # Dibujar líneas entre waypoints (mismo estilo que Editor de Misiones)
        points = [to_canvas(wp.get('x', 0), wp.get('y', 0)) for wp in waypoints]
        if len(points) > 1:
            for i in range(len(points) - 1):
                canvas.create_line(points[i][0], points[i][1],
                                  points[i+1][0], points[i+1][1],
                                  fill="#17a2b8", width=1, arrow=tk.LAST)

        # Dibujar waypoints (mismo estilo que Editor de Misiones: azul con número)
        for i, (cx, cy) in enumerate(points):
            r = 6
            canvas.create_oval(cx-r, cy-r, cx+r, cy+r,
                              fill="#17a2b8", outline="white", width=1)
            canvas.create_text(cx, cy, text=str(i + 1),
                              font=("Arial", 6, "bold"), fill="white")

    def _load_selected_plan(self):
        """Carga los waypoints del plan seleccionado al editor."""
        if not hasattr(self, '_current_scenario_id') or not self._current_scenario_id:
            messagebox.showwarning("Plan", "Primero selecciona un escenario.")
            return

        selected = self._plan_combo_var.get()
        if not selected or selected == "(nuevo plan)":
            # Limpiar waypoints para nuevo plan
            self._mission_waypoints = []
            self._wp_selected_idx = None
            self._update_wp_listbox()
            self._draw_mission_map()
            messagebox.showinfo("Plan", "Editor listo para crear nuevo plan.")
            return

        plan_id = self._plan_display_map.get(selected)
        if not plan_id:
            return

        # Obtener waypoints del plan
        plan_data = self._scenario_manager.get_flight_plan(self._current_scenario_id, plan_id)
        if plan_data and 'waypoints' in plan_data:
            self._mission_waypoints = plan_data['waypoints']
            self._mission_template_name = plan_data.get('nombre', plan_id)
            self._wp_selected_idx = None
            self._update_wp_listbox()
            self._draw_mission_map()
            messagebox.showinfo("Plan", f"Plan '{plan_data.get('nombre', plan_id)}' cargado con {len(self._mission_waypoints)} waypoints.")
        else:
            messagebox.showerror("Plan", "No se pudo cargar el plan.")

    def _save_plan_to_scenario(self):
        """Guarda los waypoints actuales como plan en el escenario seleccionado."""
        if not hasattr(self, '_current_scenario_id') or not self._current_scenario_id:
            messagebox.showwarning("Plan", "Primero selecciona un escenario.")
            return

        if not self._mission_waypoints:
            messagebox.showwarning("Plan", "No hay waypoints para guardar.")
            return

        # Pedir nombre del plan
        parent = getattr(self, '_mission_win', None) or self.root
        dialog = tk.Toplevel(parent)
        dialog.title("Guardar Plan de Vuelo")
        dialog.geometry("320x150")
        dialog.resizable(False, False)
        dialog.transient(parent)
        dialog.grab_set()

        # Centrar diálogo en la ventana padre
        dialog.update_idletasks()
        x = parent.winfo_x() + (parent.winfo_width() // 2) - 160
        y = parent.winfo_y() + (parent.winfo_height() // 2) - 75
        dialog.geometry(f"+{x}+{y}")

        tk.Label(dialog, text="Nombre del plan:", font=("Arial", 10)).pack(pady=(15, 5))
        entry = tk.Entry(dialog, width=35, font=("Arial", 10))
        entry.pack(pady=5, padx=15)
        default_name = getattr(self, '_mission_template_name', '') or ''
        if default_name:
            entry.insert(0, default_name)
        entry.focus_set()
        if default_name:
            entry.select_range(0, tk.END)

        def do_save():
            nombre = entry.get().strip()
            if not nombre:
                messagebox.showwarning("Plan", "El nombre no puede estar vacío.", parent=dialog)
                return

            # Generar ID del plan
            plan_id = nombre.lower().replace(" ", "_")
            plan_id = ''.join(c for c in plan_id if c.isalnum() or c == '_')

            # Guardar plan en el escenario
            success = self._scenario_manager.add_flight_plan(
                self._current_scenario_id,
                plan_id,
                nombre,
                self._mission_waypoints
            )

            if success:
                self._mission_template_name = nombre
                # Refrescar combo de planes
                self._populate_plans_combo(self._current_scenario_id)
                dialog.destroy()
                messagebox.showinfo("Plan", f"Plan '{nombre}' guardado en escenario.")
            else:
                messagebox.showerror("Plan", "Error guardando el plan.", parent=dialog)

        # Botones
        btn_frame = tk.Frame(dialog)
        btn_frame.pack(pady=15)
        tk.Button(btn_frame, text="💾 Guardar", command=do_save,
                  bg="#28a745", fg="white", font=("Arial", 10), width=10).pack(side="left", padx=5)
        tk.Button(btn_frame, text="Cancelar", command=dialog.destroy,
                  font=("Arial", 10), width=10).pack(side="left", padx=5)

        # Atajos de teclado
        dialog.bind("<Return>", lambda e: do_save())
        dialog.bind("<Escape>", lambda e: dialog.destroy())

    def _apply_scenario(self, scenario, scenario_id=None, source="editor"):
        """
        Función unificada para aplicar un escenario.
        Sincroniza datos entre Editor de Misiones y Abrir Mapa.

        Args:
            scenario: Dict con los datos del escenario
            scenario_id: ID opcional del escenario
            source: "editor" o "map" - desde dónde se llamó
        """
        if not scenario:
            messagebox.showwarning("Escenario", "No se pudo cargar el escenario.")
            return

        if scenario_id:
            scenario["id"] = scenario_id

        scenario_name = scenario.get("nombre") or scenario.get("id") or "Sin nombre"
        self._current_scenario_id = scenario.get('id')

        # Actualizar etiquetas de nombre en ambas ventanas
        if hasattr(self, '_scenario_name_var') and self._scenario_name_var:
            self._scenario_name_var.set(f"Cargado: {scenario_name}")
        if hasattr(self, '_map_scenario_name_var') and self._map_scenario_name_var:
            self._map_scenario_name_var.set(f"Cargado: {scenario_name}")

        # ─────────────────────────────────────────────────────────────────────
        # CARGAR GEOFENCE (sincronizado en ambos formatos)
        # ─────────────────────────────────────────────────────────────────────
        gf = scenario.get('geofence', {})
        if gf:
            x1 = gf.get('x1', -100)
            y1 = gf.get('y1', -100)
            x2 = gf.get('x2', 100)
            y2 = gf.get('y2', 100)
            zmin = gf.get('zmin', 0)
            zmax = gf.get('zmax', 200)

            # Variables globales de geofence
            self.gf_x1_var.set(str(int(x1)))
            self.gf_y1_var.set(str(int(y1)))
            self.gf_x2_var.set(str(int(x2)))
            self.gf_y2_var.set(str(int(y2)))
            self.gf_zmin_var.set(str(int(zmin)))
            self.gf_zmax_var.set(str(int(zmax)))

            # Formato para Editor de Misiones
            self._mission_geofence = {
                'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2,
                'zmin': zmin, 'zmax': zmax
            }

            # Formato para Abrir Mapa
            self._incl_rect = (x1, y1, x2, y2)
            if hasattr(self, '_incl_zmin_var') and self._incl_zmin_var:
                self._incl_zmin_var.set(int(zmin))
            if hasattr(self, '_incl_zmax_var') and self._incl_zmax_var:
                self._incl_zmax_var.set(int(zmax))

        # ─────────────────────────────────────────────────────────────────────
        # CARGAR CAPAS
        # ─────────────────────────────────────────────────────────────────────
        capas = scenario.get('capas', {})
        if capas:
            if hasattr(self, '_layer1_max_var') and self._layer1_max_var:
                self._layer1_max_var.set(int(capas.get('c1_max', 60)))
            if hasattr(self, '_layer2_max_var') and self._layer2_max_var:
                self._layer2_max_var.set(int(capas.get('c2_max', 120)))
            if hasattr(self, '_layer3_max_var') and self._layer3_max_var:
                self._layer3_max_var.set(int(capas.get('c3_max', 200)))

        # ─────────────────────────────────────────────────────────────────────
        # CARGAR OBSTÁCULOS (sincronizado en AMBOS formatos)
        # ─────────────────────────────────────────────────────────────────────
        obs = scenario.get('obstaculos', {})

        # Limpiar ambos formatos
        self._mission_exclusions = []
        self._excl_circles = []
        self._excl_polys = []

        # Cargar círculos
        for c in obs.get('circles', []):
            cx = c.get('cx', 0)
            cy = c.get('cy', 0)
            r = c.get('r', 30)
            obs_zmin = c.get('zmin', 0)
            obs_zmax = c.get('zmax', 60)
            nombre = c.get('nombre', '')

            # Formato Editor de Misiones
            self._mission_exclusions.append({
                'type': 'circle',
                'cx': cx, 'cy': cy, 'r': r,
                'zmin': obs_zmin, 'zmax': obs_zmax,
                'nombre': nombre
            })

            # Formato Abrir Mapa
            self._excl_circles.append({
                'cx': cx, 'cy': cy, 'r': r,
                'zmin': obs_zmin, 'zmax': obs_zmax
            })

        # Cargar polígonos/rectángulos
        for p in obs.get('polygons', []):
            points = p.get('poly', [])
            if not points and "x1" in p and "y1" in p and "x2" in p and "y2" in p:
                points = [
                    (p["x1"], p["y1"]),
                    (p["x2"], p["y1"]),
                    (p["x2"], p["y2"]),
                    (p["x1"], p["y2"]),
                ]
            obs_zmin = p.get('zmin', 0)
            obs_zmax = p.get('zmax', 60)
            nombre = p.get('nombre', '')
            layers = p.get('layers')

            # Formato Editor de Misiones
            self._mission_exclusions.append({
                'type': 'poly',
                'points': points,
                'zmin': obs_zmin, 'zmax': obs_zmax,
                'layers': layers,
                'nombre': nombre
            })

            # Formato Abrir Mapa
            self._excl_polys.append({
                'poly': points,
                'zmin': obs_zmin,
                'zmax': obs_zmax,
                'layers': layers
            })

        # ─────────────────────────────────────────────────────────────────────
        # REDIBUJAR MAPAS
        # ─────────────────────────────────────────────────────────────────────
        # Redibujar Editor de Misiones si está abierto
        if hasattr(self, '_mission_canvas') and self._mission_canvas:
            try:
                if self._mission_canvas.winfo_exists():
                    self._draw_mission_map()
            except Exception as e:
                print(f"[_apply_scenario] Error redibujando editor: {e}")

        # Redibujar Abrir Mapa si está abierto
        if self._map_win:
            try:
                if self._map_win.winfo_exists():
                    self._redraw_map_static()
            except Exception as e:
                print(f"[_apply_scenario] Error redibujando mapa: {e}")

        # Vincular escenario a la sesión activa si corresponde
        if source == "map" and self._session_manager.is_session_active():
            self._session_manager._session_data["escenario_id"] = self._current_scenario_id
            self._session_manager._session_data["tipo"] = "manual"
            self._session_manager._save_session_metadata()
            print(f"[Session] Escenario vinculado: {self._current_scenario_id}")

        messagebox.showinfo("Escenario", f"Escenario '{scenario_name}' cargado.")

    def _apply_scenario_to_editor(self, scenario, scenario_id=None):

        self._apply_scenario(scenario, scenario_id=scenario_id, source="editor")

    def _load_scenario_from_file(self):

        path = filedialog.askopenfilename(
            initialdir=self._scenario_manager.base_dir,
            filetypes=[("Escenario JSON", "*.json")]
        )
        if not path:
            return

        try:
            with open(path, "r", encoding="utf-8") as f:
                scenario = json.load(f)

            scenario_id = scenario.get("id") or os.path.splitext(os.path.basename(path))[0]
            self._apply_scenario_to_editor(scenario, scenario_id=scenario_id)
        except Exception as e:
            messagebox.showerror("Escenario", f"Error cargando escenario: {e}")

    def _load_scenario_to_editor(self):

        if not hasattr(self, '_scenario_list') or not self._scenario_list:
            messagebox.showwarning("Escenario", "No hay escenarios disponibles.")
            return

        selected_name = self._scenario_combo_var.get()
        scenario_id = None
        if hasattr(self, "_scenario_display_map"):
            scenario_id = self._scenario_display_map.get(selected_name)
        scenario = None
        if scenario_id:
            scenario = self._scenario_manager.load_scenario(scenario_id)
        else:
            for s in self._scenario_list:
                if s['nombre'] == selected_name:
                    scenario = self._scenario_manager.load_scenario(s['id'])
                    break

        self._apply_scenario_to_editor(scenario)


    def _refresh_map_scenario_list(self):

        scenarios = self._scenario_manager.list_scenarios()
        names = []
        self._map_scenario_display_map = {}
        for s in scenarios:
            display = f"{s['nombre']} ({s['id']})"
            names.append(display)
            self._map_scenario_display_map[display] = s['id']
        self._map_scenario_list = scenarios

        if hasattr(self, '_map_scenario_combo') and self._map_scenario_combo:
            self._map_scenario_combo['values'] = names
            if names and not self._map_scenario_combo_var.get():
                self._map_scenario_combo_var.set(names[0])

    def _load_scenario_to_map(self):

        if not hasattr(self, '_map_scenario_list') or not self._map_scenario_list:
            messagebox.showwarning("Escenario", "No hay escenarios disponibles.")
            return

        selected_name = self._map_scenario_combo_var.get()
        scenario_id = None
        if hasattr(self, "_map_scenario_display_map"):
            scenario_id = self._map_scenario_display_map.get(selected_name)
        scenario = None
        if scenario_id:
            scenario = self._scenario_manager.load_scenario(scenario_id)
        else:
            for s in self._map_scenario_list:
                if s['nombre'] == selected_name:
                    scenario = self._scenario_manager.load_scenario(s['id'])
                    break

        self._apply_scenario_to_map(scenario)

    def _apply_scenario_to_map(self, scenario, scenario_id=None):

        self._apply_scenario(scenario, scenario_id=scenario_id, source="map")

    def _load_scenario_from_file_to_map(self):

        path = filedialog.askopenfilename(
            initialdir=self._scenario_manager.base_dir,
            filetypes=[("Escenario JSON", "*.json")]
        )
        if not path:
            return

        try:
            with open(path, "r", encoding="utf-8") as f:
                scenario = json.load(f)

            scenario_id = scenario.get("id") or os.path.splitext(os.path.basename(path))[0]
            self._apply_scenario_to_map(scenario, scenario_id=scenario_id)
        except Exception as e:
            messagebox.showerror("Escenario", f"Error cargando escenario: {e}")

    def _build_scenario_data(self):

        # ─────────────────────────────────────────────────────────────────────
        # GEOFENCE
        # ─────────────────────────────────────────────────────────────────────
        geofence = None

        # Intentar desde _mission_geofence (Editor)
        if hasattr(self, '_mission_geofence') and self._mission_geofence:
            geofence = dict(self._mission_geofence)
        # Intentar desde _incl_rect (Abrir Mapa)
        elif hasattr(self, '_incl_rect') and self._incl_rect:
            x1, y1, x2, y2 = self._incl_rect
            # Obtener zmin/zmax de forma segura
            zmin_val = 0
            zmax_val = 200
            if hasattr(self, 'gf_zmin_var') and self.gf_zmin_var:
                try:
                    zmin_val = int(self.gf_zmin_var.get() or 0)
                except (ValueError, TypeError):
                    zmin_val = 0
            if hasattr(self, 'gf_zmax_var') and self.gf_zmax_var:
                try:
                    zmax_val = int(self.gf_zmax_var.get() or 200)
                except (ValueError, TypeError):
                    zmax_val = 200
            geofence = {
                'x1': int(x1), 'y1': int(y1),
                'x2': int(x2), 'y2': int(y2),
                'zmin': zmin_val,
                'zmax': zmax_val
            }
        # Intentar desde las variables de entrada
        elif hasattr(self, 'gf_x1_var') and self.gf_x1_var:
            geofence = {
                'x1': int(self.gf_x1_var.get() or -100),
                'y1': int(self.gf_y1_var.get() or -100),
                'x2': int(self.gf_x2_var.get() or 100),
                'y2': int(self.gf_y2_var.get() or 100),
                'zmin': int(self.gf_zmin_var.get() or 0),
                'zmax': int(self.gf_zmax_var.get() or 200)
            }
        else:
            # Valores por defecto
            geofence = {'x1': -100, 'y1': -100, 'x2': 100, 'y2': 100, 'zmin': 0, 'zmax': 200}

        # ─────────────────────────────────────────────────────────────────────
        # CAPAS
        # ─────────────────────────────────────────────────────────────────────
        def safe_get_layer(attr_name, default):
            var = getattr(self, attr_name, None)
            if var:
                try:
                    return int(var.get() or default)
                except (ValueError, TypeError):
                    return default
            return default

        capas = {
            'c1_max': safe_get_layer('_layer1_max_var', 60),
            'c2_max': safe_get_layer('_layer2_max_var', 120),
            'c3_max': safe_get_layer('_layer3_max_var', 200)
        }

        # ─────────────────────────────────────────────────────────────────────
        # OBSTÁCULOS (combinar ambos formatos)
        # ─────────────────────────────────────────────────────────────────────
        circles = []
        polygons = []

        print(f"[DEBUG _build_scenario_data] _mission_exclusions: {len(getattr(self, '_mission_exclusions', []))}")
        print(f"[DEBUG _build_scenario_data] _excl_circles: {len(getattr(self, '_excl_circles', []))}")
        print(f"[DEBUG _build_scenario_data] _excl_polys: {len(getattr(self, '_excl_polys', []))}")

        # Usar _mission_exclusions si tiene datos (Editor)
        if hasattr(self, '_mission_exclusions') and self._mission_exclusions:
            for exc in self._mission_exclusions:
                if exc.get('type') == 'circle':
                    circles.append({
                        'cx': exc.get('cx', 0),
                        'cy': exc.get('cy', 0),
                        'r': exc.get('r', 30),
                        'zmin': exc.get('zmin', 0),
                        'zmax': exc.get('zmax', 60),
                        'nombre': exc.get('nombre', '')
                    })
                elif exc.get('type') in ('polygon', 'poly', 'rect'):
                    points = exc.get('points', [])
                    # Si es rectángulo, convertir a puntos
                    if exc.get('type') == 'rect' and not points:
                        x1, y1 = exc.get('x1', 0), exc.get('y1', 0)
                        x2, y2 = exc.get('x2', 0), exc.get('y2', 0)
                        points = [[x1, y1], [x2, y1], [x2, y2], [x1, y2]]
                    polygons.append({
                        'poly': points,
                        'zmin': exc.get('zmin', 0),
                        'zmax': exc.get('zmax', 60),
                        'nombre': exc.get('nombre', '')
                    })
        else:
            # Usar formato de Abrir Mapa
            if hasattr(self, '_excl_circles') and self._excl_circles:
                for c in self._excl_circles:
                    circles.append({
                        'cx': c['cx'], 'cy': c['cy'], 'r': c['r'],
                        'zmin': c.get('zmin', 0), 'zmax': c.get('zmax', 60)
                    })
            if hasattr(self, '_excl_polys') and self._excl_polys:
                for p in self._excl_polys:
                    # Manejar ambos formatos: 'poly' o 'points'
                    pts = p.get('poly') or p.get('points', [])
                    polygons.append({
                        'poly': pts,
                        'zmin': p.get('zmin', 0), 'zmax': p.get('zmax', 60)
                    })

        obstaculos = {'circles': circles, 'polygons': polygons}

        return geofence, capas, obstaculos

    def _save_scenario_from_map(self):

        parent = getattr(self, '_map_win', None) or self.root
        self._show_save_scenario_dialog(include_waypoints=False, parent=parent)

    def _save_scenario_from_editor(self):

        parent = getattr(self, '_mission_win', None) or self.root
        self._show_save_scenario_dialog(include_waypoints=True, parent=parent)

    def _show_save_scenario_dialog(self, include_waypoints=False, parent=None):

        if parent is None:
            parent = self.root

        # Crear diálogo
        dialog = tk.Toplevel(parent)
        dialog.title("Guardar Escenario")
        dialog.geometry("300x120")
        dialog.resizable(False, False)
        dialog.transient(parent)
        dialog.attributes('-topmost', True)

        # Centrar en pantalla
        dialog.update_idletasks()
        x = (dialog.winfo_screenwidth() - 300) // 2
        y = (dialog.winfo_screenheight() - 120) // 2
        dialog.geometry(f"300x120+{x}+{y}")

        # Contenido
        tk.Label(dialog, text="Nombre del escenario:", font=("Arial", 10)).pack(pady=(15, 5))
        entry = tk.Entry(dialog, width=35, font=("Arial", 10))
        # Usar nombre actual si existe, sino "Nuevo Escenario"
        default_name = "Nuevo Escenario"
        if hasattr(self, '_current_scenario_id') and self._current_scenario_id:
            # Intentar obtener el nombre del escenario actual
            try:
                scenario = self._scenario_manager.load_scenario(self._current_scenario_id)
                if scenario and scenario.get('nombre'):
                    default_name = scenario['nombre']
            except:
                default_name = self._current_scenario_id
        entry.insert(0, default_name)
        entry.pack(pady=5)
        entry.select_range(0, tk.END)
        entry.focus_set()

        def do_save():
            nombre = entry.get().strip()
            if not nombre:
                messagebox.showwarning("Error", "El nombre no puede estar vacío.", parent=dialog)
                return

            # Generar ID desde el nombre
            scenario_id = nombre.lower().replace(" ", "_")
            scenario_id = ''.join(c for c in scenario_id if c.isalnum() or c == '_')

            try:
                geofence, capas, obstaculos = self._build_scenario_data()
                self._scenario_manager.create_scenario(scenario_id, nombre, geofence, capas, obstaculos)

                # Guardar waypoints si se solicita
                if include_waypoints and self._mission_waypoints:
                    self._scenario_manager.add_flight_plan(
                        scenario_id, "plan_editor", "Plan desde Editor",
                        self._mission_waypoints
                    )

                self._current_scenario_id = scenario_id

                # Actualizar etiquetas
                if hasattr(self, '_scenario_name_var') and self._scenario_name_var:
                    self._scenario_name_var.set(f"Guardado: {nombre}")
                if hasattr(self, '_map_scenario_name_var') and self._map_scenario_name_var:
                    self._map_scenario_name_var.set(f"Guardado: {nombre}")

                # Refrescar listas
                try:
                    self._refresh_scenario_list()
                except Exception:
                    pass
                try:
                    self._refresh_map_scenario_list()
                except Exception:
                    pass

                dialog.destroy()
                messagebox.showinfo("Escenario", f"Escenario '{nombre}' guardado.")
            except Exception as e:
                messagebox.showerror("Error", f"Error al guardar: {e}", parent=dialog)

        # Botones
        btn_frame = tk.Frame(dialog)
        btn_frame.pack(pady=10)
        tk.Button(btn_frame, text="💾 Guardar", command=do_save,
                  bg="#28a745", fg="white", font=("Arial", 10), width=12).pack(side="left", padx=5)
        tk.Button(btn_frame, text="Cancelar", command=dialog.destroy,
                  bg="#6c757d", fg="white", font=("Arial", 10), width=12).pack(side="left", padx=5)

        # Enter para guardar, Escape para cancelar
        dialog.bind('<Return>', lambda e: do_save())
        dialog.bind('<Escape>', lambda e: dialog.destroy())

        # Forzar visibilidad y foco
        dialog.lift()
        dialog.grab_set()
        dialog.focus_force()
        entry.focus_set()
        dialog.wait_window()

    def _mission_apply_geofence(self):

        # Obtener valores
        zmin = int(self.gf_zmin_var.get() or 0)
        zmax = int(self.gf_zmax_var.get() or 200)

        # Validar que zmin <= zmax
        if zmin > zmax:
            zmin, zmax = zmax, zmin
            self.gf_zmin_var.set(str(zmin))
            self.gf_zmax_var.set(str(zmax))
            print(f"[geofence] Corregido: zmin/zmax intercambiados → {zmin}-{zmax}")

        self._mission_geofence = {
            'x1': min(int(self.gf_x1_var.get() or -100), int(self.gf_x2_var.get() or 100)),
            'y1': min(int(self.gf_y1_var.get() or -100), int(self.gf_y2_var.get() or 100)),
            'x2': max(int(self.gf_x1_var.get() or -100), int(self.gf_x2_var.get() or 100)),
            'y2': max(int(self.gf_y1_var.get() or -100), int(self.gf_y2_var.get() or 100)),
            'zmin': zmin,
            'zmax': zmax
        }
        self._draw_mission_map()

    def _update_wp_listbox(self):

        self._wp_listbox.delete(0, tk.END)
        for i, wp in enumerate(self._mission_waypoints):
            actions = []
            if wp.get('photo'):
                actions.append("📷")
            if wp.get('video'):
                actions.append(f"🎬{wp.get('video_duration', 5)}s")
            if wp.get('video_start'):
                actions.append("▶REC")
            if wp.get('video_stop'):
                actions.append("⏹REC")
            if wp.get('rotate'):
                actions.append(f"🔄{wp.get('rotate_deg', 0)}°")
            if wp.get('wait'):
                actions.append(f"⏱{wp.get('wait_sec', 0)}s")
            if wp.get('layer_up'):
                actions.append("⬆Capa")
            if wp.get('layer_down'):
                actions.append("⬇Capa")
            action_str = " ".join(actions) if actions else ""
            self._wp_listbox.insert(tk.END, f"WP{i + 1}: ({wp['x']}, {wp['y']}, {wp['z']}) {action_str}")

    def _on_wp_select(self, event):

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
        self._action_video_start.set(wp.get('video_start', False))
        self._action_video_stop.set(wp.get('video_stop', False))
        self._action_rotate.set(wp.get('rotate', False))
        self._action_rotate_deg.set(wp.get('rotate_deg', 90))
        self._action_wait.set(wp.get('wait', False))
        self._action_wait_sec.set(wp.get('wait_sec', 2))
        self._action_layer_up.set(wp.get('layer_up', False))
        self._action_layer_down.set(wp.get('layer_down', False))

        # Actualizar estado de exclusión mutua de video
        self._update_video_checkbox_states()

        self._wp_action_label.configure(text=f"Editando WP{idx + 1}")
        self._draw_mission_map()

    def _update_wp_actions(self):

        if self._wp_selected_idx is None:
            return

        wp = self._mission_waypoints[self._wp_selected_idx]
        wp['photo'] = self._action_photo.get()
        wp['video'] = self._action_video.get()
        wp['video_duration'] = self._action_video_duration.get()
        wp['video_start'] = self._action_video_start.get()
        wp['video_stop'] = self._action_video_stop.get()
        wp['rotate'] = self._action_rotate.get()
        wp['rotate_deg'] = self._action_rotate_deg.get()
        wp['wait'] = self._action_wait.get()
        wp['wait_sec'] = self._action_wait_sec.get()
        wp['layer_up'] = self._action_layer_up.get()
        wp['layer_down'] = self._action_layer_down.get()

        self._update_wp_listbox()
        self._draw_mission_map()

    def _on_video_mode_change(self):

        # Si "Grabar Xs" está activo, desactivar start/stop
        if self._action_video.get():
            self._action_video_start.set(False)
            self._action_video_stop.set(False)
        # Si start o stop están activos, desactivar "Grabar Xs"
        elif self._action_video_start.get() or self._action_video_stop.get():
            self._action_video.set(False)

        self._update_video_checkbox_states()
        self._update_wp_actions()

    def _update_video_checkbox_states(self):

        # Si "Grabar Xs" está activo, deshabilitar start/stop
        if self._action_video.get():
            self._cb_video_start.configure(state="disabled")
            self._cb_video_stop.configure(state="disabled")
            self._entry_video_dur.configure(state="normal")
        # Si start o stop están activos, deshabilitar "Grabar Xs"
        elif self._action_video_start.get() or self._action_video_stop.get():
            self._cb_video.configure(state="disabled")
            self._entry_video_dur.configure(state="disabled")
            self._cb_video_start.configure(state="normal")
            self._cb_video_stop.configure(state="normal")
        # Si ninguno está activo, todos habilitados
        else:
            self._cb_video.configure(state="normal")
            self._entry_video_dur.configure(state="normal")
            self._cb_video_start.configure(state="normal")
            self._cb_video_stop.configure(state="normal")

    def _on_layer_change(self):
        """Hace que subir/bajar capa sean mutuamente excluyentes."""
        # Si se activa "Subir capa", desactivar "Bajar capa"
        if self._action_layer_up.get() and self._action_layer_down.get():
            # Se acaba de activar uno, desactivar el otro
            # Detectar cuál se activó último mirando el widget que disparó el evento
            pass  # Ambos no pueden estar activos a la vez

        if self._action_layer_up.get():
            self._action_layer_down.set(False)
        elif self._action_layer_down.get():
            self._action_layer_up.set(False)

        self._update_wp_actions()

    def _apply_wp_coords(self):

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

        # Validar límites de seguridad
        distance_from_origin = math.sqrt(new_x * new_x + new_y * new_y)
        if distance_from_origin > MISSION_MAX_DISTANCE_CM:
            messagebox.showwarning("Límite de seguridad",
                f"Distancia {distance_from_origin:.0f}cm excede el máximo ({MISSION_MAX_DISTANCE_CM}cm)")
            return

        if new_z > MISSION_MAX_HEIGHT_CM:
            messagebox.showwarning("Límite de seguridad",
                f"Altura {new_z}cm excede el máximo ({MISSION_MAX_HEIGHT_CM}cm)")
            return

        if new_z < MISSION_MIN_HEIGHT_CM:
            messagebox.showwarning("Límite de seguridad",
                f"Altura {new_z}cm es menor que el mínimo seguro ({MISSION_MIN_HEIGHT_CM}cm)")
            return

        # Validar contra obstáculos
        for obs in self._mission_exclusions:
            if point_in_obstacle(new_x, new_y, obs, z=new_z):
                messagebox.showwarning("Obstáculo", "La nueva posición colisiona con un obstáculo")
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

        if self._wp_selected_idx is not None:
            del self._mission_waypoints[self._wp_selected_idx]
            self._wp_selected_idx = None
            self._update_wp_listbox()
            self._draw_mission_map()

    def _move_wp(self, direction):

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

        if messagebox.askyesno("Confirmar", "¿Eliminar todos los waypoints?"):
            self._mission_waypoints.clear()
            self._wp_selected_idx = None
            self._update_wp_listbox()
            self._draw_mission_map()

    def _validate_mission_only(self):

        if not self._mission_waypoints:
            messagebox.showwarning("Sin waypoints", "Añade al menos un waypoint", parent=self._mission_win)
            return

        # Validar rutas contra obstáculos
        try:
            valid, error = validate_mission_paths(self._mission_waypoints, self._mission_exclusions)
        except Exception as e:
            self._mission_status_label.configure(text=f"❌ Error: {e}", fg="#dc3545")
            return

        if valid:
            # Ruta válida sin obstáculos en el camino
            msg = f"✅ Ruta válida ({len(self._mission_waypoints)} WPs)"
            self._mission_status_label.configure(text=msg, fg="#28a745")
            self._mission_win.after(50, lambda: messagebox.showinfo("Validación", msg, parent=self._mission_win))
        else:
            # Ruta inválida - cruza obstáculos
            msg = f"❌ {error}"
            self._mission_status_label.configure(text=msg, fg="#dc3545")
            detail = f"{error}\n\nMueve los waypoints para evitar los obstáculos."
            self._mission_win.after(50, lambda d=detail: messagebox.showerror("Ruta Inválida", d, parent=self._mission_win))

    def _execute_mission(self):

        if not self._mission_waypoints:
            messagebox.showwarning("Sin waypoints", "Añade al menos un waypoint")
            return

        if self.dron.state != "connected" and self.dron.state != "flying":
            messagebox.showwarning("No conectado", "Conecta el dron primero")
            return

        if self._mission_running:
            messagebox.showinfo("En ejecución", "Ya hay una misión en ejecución")
            return

        # Si hay vídeos en la misión, mantener el stream vivo con FPV para evitar cortes
        has_video = any(wp.get('video') or wp.get('video_start') for wp in self._mission_waypoints)
        if has_video and not self._fpv_running:
            print("[mission] Preparando stream FPV para grabación...")
            self.start_fpv()
            stream_ready = False
            for i in range(50):  # Timeout 5 segundos
                with self._frame_lock:
                    if self._last_bgr is not None:
                        print(f"[mission] Stream listo después de {i*0.1:.1f}s")
                        stream_ready = True
                        break
                time.sleep(0.1)

            if not stream_ready:
                print("[mission] ERROR: Timeout esperando frame del stream")
                if not messagebox.askyesno("Sin Stream",
                        "No se pudo iniciar el stream de video.\n"
                        "¿Continuar la misión sin grabar videos?",
                        parent=self._mission_win):
                    return

        # Registrar la sesión como misión aunque no haya escenario cargado
        if self._session_manager.is_session_active():
            self._session_manager._session_data["tipo"] = "plan"
            self._session_manager._session_data["plan_id"] = self._get_mission_plan_id()
            if self._current_scenario_id:
                self._session_manager._session_data["escenario_id"] = self._current_scenario_id
            self._session_manager._save_session_metadata()

        # DEBUG: Mostrar información de validación
        print(f"[DEBUG] Waypoints: {len(self._mission_waypoints)}")
        print(f"[DEBUG] Obstáculos: {len(self._mission_exclusions)}")

        # Validar rutas contra obstáculos (usa módulo tello_geometry)
        valid, error = validate_mission_paths(self._mission_waypoints, self._mission_exclusions)
        print(f"[DEBUG] Validación: valid={valid}, error={error}")

        # Si la ruta no es válida, mostrar error y no ejecutar
        if not valid:
            print("[DEBUG] Ruta inválida - cruza obstáculos")
            messagebox.showerror("Ruta inválida",
                f"⚠️ {error}\n\nMueve los waypoints para evitar los obstáculos.")
            return

        waypoints_to_execute = self._mission_waypoints

        print(f"[DEBUG] Ejecutando misión con {len(waypoints_to_execute)} waypoints")
        print("[DEBUG] Waypoints a ejecutar (con acciones):")
        for i, wp in enumerate(waypoints_to_execute):
            actions = []
            if wp.get('photo'): actions.append('📷foto')
            if wp.get('video'): actions.append(f"🎥video({wp.get('video_duration', 5)}s)")
            if wp.get('video_start'): actions.append('▶video_start')
            if wp.get('video_stop'): actions.append('⏹video_stop')
            actions_str = ', '.join(actions) if actions else '(sin acciones)'
            print(f"  [{i}] x={wp.get('x')}, y={wp.get('y')}, z={wp.get('z')} → {actions_str}")

        # ═══════════════════════════════════════════════════════════════
        # CONFIRMACIÓN ANTES DE EJECUTAR
        # ═══════════════════════════════════════════════════════════════
        n_waypoints = len(self._mission_waypoints)
        n_obstacles = len(self._mission_exclusions)

        # Contar acciones
        n_photos = sum(1 for wp in self._mission_waypoints if wp.get('photo', False))
        n_videos = sum(1 for wp in self._mission_waypoints if wp.get('video', False))

        confirm_msg = f"RESUMEN DE MISIÓN\n\n"
        confirm_msg += f"📍 Waypoints: {n_waypoints}"
        confirm_msg += f"\n🚧 Obstáculos: {n_obstacles}"
        if n_photos > 0:
            confirm_msg += f"\n📷 Fotos programadas: {n_photos}"
        if n_videos > 0:
            confirm_msg += f"\n🎥 Videos programados: {n_videos}"
        confirm_msg += f"\n\n¿Iniciar misión?"

        if not messagebox.askyesno("Confirmar Misión", confirm_msg, parent=self._mission_win):
            return

        # ═══════════════════════════════════════════════════════════════
        # SINCRONIZAR OBSTÁCULOS CON GEOFENCE REAL (protección en tiempo real)
        # ═══════════════════════════════════════════════════════════════
        if self._mission_exclusions:
            print(f"[DEBUG] Sincronizando {len(self._mission_exclusions)} obstáculos con geofence...")
            try:
                # Asegurar que existen las listas de exclusiones en el dron
                if not hasattr(self.dron, "_gf_excl_circles"):
                    self.dron._gf_excl_circles = []
                if not hasattr(self.dron, "_gf_excl_polys"):
                    self.dron._gf_excl_polys = []

                # Convertir y añadir obstáculos del editor al geofence
                for obs in self._mission_exclusions:
                    obs_type = obs.get('type', 'circle')
                    if obs_type == 'circle':
                        gf_circle = {
                            'cx': obs['cx'], 'cy': obs['cy'], 'r': obs['r'],
                            'zmin': obs.get('zmin', 0), 'zmax': obs.get('zmax', 200)
                        }
                        self.dron._gf_excl_circles.append(gf_circle)
                    elif obs_type == 'rect':
                        # Convertir rectángulo a polígono de 4 puntos
                        rect_poly = [
                            (obs['x1'], obs['y1']), (obs['x2'], obs['y1']),
                            (obs['x2'], obs['y2']), (obs['x1'], obs['y2'])
                        ]
                        gf_poly = {
                            'poly': rect_poly,
                            'zmin': obs.get('zmin', 0), 'zmax': obs.get('zmax', 200)
                        }
                        self.dron._gf_excl_polys.append(gf_poly)
                    elif obs_type == 'poly':
                        gf_poly = {
                            'poly': obs['points'],
                            'zmin': obs.get('zmin', 0), 'zmax': obs.get('zmax', 200)
                        }
                        self.dron._gf_excl_polys.append(gf_poly)

                # Activar geofence si no está activo
                if not getattr(self.dron, "_gf_enabled", False):
                    setattr(self.dron, "_gf_enabled", True)
                    print("[DEBUG] Geofence activado para protección de obstáculos")

                print(f"[DEBUG] Sincronización completada: {len(self.dron._gf_excl_circles)} círculos, {len(self.dron._gf_excl_polys)} polígonos")
            except Exception as e:
                print(f"[DEBUG] Error sincronizando obstáculos: {e}")

        # ═══════════════════════════════════════════════════════════════
        # REGISTRAR ESCENARIO EN LA SESIÓN
        # ═══════════════════════════════════════════════════════════════
        if self._session_manager.is_session_active():
            if self._current_scenario_id:
                self._session_manager._session_data["escenario_id"] = self._current_scenario_id
                self._session_manager._session_data["tipo"] = "plan"
                self._session_manager._session_data["plan_id"] = self._get_mission_plan_id()
                self._session_manager._save_session_metadata()
                print(f"[Session] Escenario registrado: {self._current_scenario_id}")

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
            # DEBUG: Mostrar información detallada
            pose = getattr(self.dron, "pose", None)
            pose_str = f"({pose.x_cm:.0f}, {pose.y_cm:.0f}, {pose.z_cm:.0f})" if pose else "(sin pose)"
            wp_info = self._mission_waypoints[idx] if idx < len(self._mission_waypoints) else {}
            wp_pos = f"({wp_info.get('x', '?')}, {wp_info.get('y', '?')}, {wp_info.get('z', '?')})"

            print(f"[on_action] ═══════════════════════════════════════")
            print(f"[on_action] ACCIÓN: {action_name} en WP{idx + 1}")
            print(f"[on_action] Posición esperada del WP: {wp_pos}")
            print(f"[on_action] Posición actual del dron: {pose_str}")
            print(f"[on_action] ═══════════════════════════════════════")

            action_texts = {
                'rotate': '🔄 Rotando', 'photo': '📷 Foto', 'video': '🎥 Video',
                'video_start': '▶ Iniciando REC', 'video_stop': '⏹ Parando REC',
                'wait': '⏳ Esperando'
            }
            text = action_texts.get(action_name, action_name)
            self._mission_win.after(0, lambda: self._mission_status_label.configure(
                text=f"WP{idx + 1}: {text}", fg="#17a2b8"))

            # Manejar acciones específicas
            if action_name == 'photo':
                # Tomar foto inmediatamente
                print("[on_action] Ejecutando take_snapshot()")
                import time as time_module
                try:
                    # Asegurar que el stream está activo
                    if not self._fpv_running:
                        print("[on_action] Stream no activo, iniciando FPV...")
                        self.start_fpv()

                    # IMPORTANTE: Esperar a un frame NUEVO (no usar frame antiguo)
                    # Esto evita que la foto se tome con un frame de antes del despegue
                    start_mark = time_module.monotonic()
                    print(f"[on_action] Esperando frame nuevo (timestamp > {start_mark:.1f})...")
                    frame_found = False
                    for i in range(50):  # Máximo 5 segundos
                        with self._frame_lock:
                            if self._last_frame_time > start_mark:
                                print(f"[on_action] Frame nuevo detectado tras {i*0.1:.1f}s")
                                frame_found = True
                                break
                        time_module.sleep(0.1)

                    if not frame_found:
                        print("[on_action] WARNING: Timeout esperando frame nuevo, intentando de todas formas...")

                    self.take_snapshot()
                    print("[on_action] Foto guardada correctamente")
                except Exception as e:
                    print(f"[on_action] Error tomando foto: {e}")
                    import traceback
                    traceback.print_exc()
            elif action_name == 'video':
                # Iniciar grabación de video (el mission executor controlará la duración)
                print(f"[on_action] Iniciando grabación de video")
                import time as time_module

                try:
                    # Esperar a un frame nuevo después de llegar al waypoint
                    start_mark = time_module.monotonic()
                    for i in range(30):  # Máximo ~3s
                        with self._frame_lock:
                            if self._last_frame_time > start_mark:
                                print(f"[on_action] Frame nuevo detectado tras {i*0.1:.1f}s")
                                break
                        time_module.sleep(0.1)

                    # _start_recording con hilo dedicado para misiones (evita conflictos con FPV loop)
                    duration = 5
                    try:
                        duration = float(self._mission_waypoints[idx].get('video_duration', 5) or 5)
                    except Exception:
                        duration = 5
                    self._start_recording(force_dedicated_thread=True, expected_duration=duration)

                    # Esperar brevemente a que el hilo empiece a grabar
                    for i in range(20):  # Máximo 2 segundos
                        if self._rec_frame_count > 0:
                            print(f"[on_action] Grabación confirmada, frames: {self._rec_frame_count}")
                            break
                        time_module.sleep(0.1)
                    else:
                        print(f"[on_action] WARNING: Grabación iniciada pero sin confirmar frames aún")

                    print(f"[on_action] Grabación iniciada, mission executor controlará duración")

                except Exception as e:
                    print(f"[on_action] Error iniciando grabación: {e}")
                    import traceback
                    traceback.print_exc()

            elif action_name == 'video_stop':
                # Detener grabación de video (llamado por mission executor o video_stop en waypoint)
                try:
                    frames_recorded = getattr(self, '_rec_frame_count', 0)
                    print(f"[on_action] Deteniendo grabación de video")
                    print(f"[on_action] Total frames grabados: {frames_recorded}")
                    self._stop_recording()

                    if frames_recorded == 0:
                        print("[on_action] ⚠️ ADVERTENCIA: El video tiene 0 frames")
                        print("[on_action] Posibles causas:")
                        print("[on_action]   - El stream de video del dron no estaba activo")
                        print("[on_action]   - El FPV no estaba corriendo")
                        print("[on_action]   - Problema de conexión con el dron")
                    else:
                        print(f"[on_action] Video guardado correctamente ({frames_recorded} frames)")
                except Exception as e:
                    print(f"[on_action] Error deteniendo grabación: {e}")
                    import traceback
                    traceback.print_exc()

            elif action_name == 'video_start':
                # Iniciar grabación continua (sin duración, hasta video_stop)
                print(f"[on_action] Iniciando grabación continua (hasta video_stop)")
                import time as time_module

                try:
                    # IMPORTANTE: Esperar a un frame NUEVO antes de empezar a grabar
                    # Esto evita grabar frames antiguos del buffer (ej. del despegue)
                    start_mark = time_module.monotonic()
                    print(f"[on_action] Esperando frame nuevo (timestamp > {start_mark:.1f})...")
                    for i in range(30):  # Máximo ~3s
                        with self._frame_lock:
                            if self._last_frame_time > start_mark:
                                print(f"[on_action] Frame nuevo detectado tras {i*0.1:.1f}s")
                                break
                        time_module.sleep(0.1)

                    self._start_recording(force_dedicated_thread=True, expected_duration=None)

                    # Esperar brevemente a que el hilo empiece a grabar
                    for i in range(20):  # Máximo 2 segundos
                        if self._rec_frame_count > 0:
                            print(f"[on_action] Grabación continua confirmada, frames: {self._rec_frame_count}")
                            break
                        time_module.sleep(0.1)
                    else:
                        print(f"[on_action] WARNING: Grabación iniciada pero sin confirmar frames aún")

                    print(f"[on_action] Grabación continua activa, continuará hasta video_stop")

                except Exception as e:
                    print(f"[on_action] Error iniciando grabación continua: {e}")
                    import traceback
                    traceback.print_exc()
            elif action_name == 'rotate':
                try:
                    rotate_deg = float(self._mission_waypoints[idx].get('rotate_deg', 0) or 0)
                except Exception:
                    rotate_deg = 0
                if rotate_deg == 0:
                    return
                try:
                    if rotate_deg >= 0:
                        self.dron.cw(int(round(rotate_deg)))
                    else:
                        self.dron.ccw(int(round(abs(rotate_deg))))
                    pose = getattr(self.dron, "pose", None)
                    real_yaw = None
                    try:
                        if hasattr(self.dron, "get_yaw"):
                            real_yaw = self.dron.get_yaw()
                    except Exception:
                        real_yaw = None
                    commanded_yaw = None
                    if real_yaw is not None:
                        try:
                            real_yaw = float(real_yaw)
                            if real_yaw > 180:
                                real_yaw = real_yaw - 360
                            commanded_yaw = real_yaw
                            if pose is not None:
                                pose.set_heading_from_absolute_yaw(float(real_yaw))
                        except Exception:
                            pass
                    if commanded_yaw is None and pose is not None:
                        commanded_yaw = float(getattr(pose, "yaw_deg", 0.0) or 0.0)
                    if commanded_yaw is not None:
                        setattr(self.dron, "_commanded_yaw", commanded_yaw)
                except Exception as e:
                    print(f"[on_action] Error rotando: {e}")
                    import traceback
                    traceback.print_exc()

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

        # Obtener configuración de capas para acciones layer_up/layer_down
        c1_max = getattr(self, '_layer1_max_var', None)
        c2_max = getattr(self, '_layer2_max_var', None)
        c3_max = getattr(self, '_layer3_max_var', None)
        if c1_max and c2_max and c3_max:
            mission_layers = [
                {"z_min": 0, "z_max": c1_max.get()},
                {"z_min": c1_max.get(), "z_max": c2_max.get()},
                {"z_min": c2_max.get(), "z_max": c3_max.get()},
            ]
        else:
            # Valores por defecto
            mission_layers = [
                {"z_min": 0, "z_max": 60},
                {"z_min": 60, "z_max": 120},
                {"z_min": 120, "z_max": 200},
            ]

        # Ejecutar misión
        def run_in_thread():
            try:
                self.dron.run_mission(
                    waypoints=waypoints_to_execute,
                    do_land=True,
                    face_target=True,  # Rotar hacia destino antes de moverse (como un coche)
                    layers=mission_layers,
                    blocking=True,
                    on_wp=on_wp_arrived,
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