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

        # POSE
        self.x_var = tk.StringVar(value="X: —")
        self.y_var = tk.StringVar(value="Y: —")
        self.z_var = tk.StringVar(value="Z: —")
        self.yaw_var = tk.StringVar(value="Yaw: —")

        # VELOCIDADES
        self.vx_var = tk.StringVar(value="Vx: —")
        self.vy_var = tk.StringVar(value="Vy: —")
        self.vz_var = tk.StringVar(value="Vz: —")

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
        self._shots_dir = os.path.abspath("captures")
        self._recs_dir = os.path.abspath("videos")
        os.makedirs(self._shots_dir, exist_ok=True)
        os.makedirs(self._recs_dir, exist_ok=True)
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
        tk.Button(conn, text="Conectar", width=12, command=self.on_connect, bg="#ffb347").grid(row=0, column=0, **pad)
        tk.Button(conn, text="Desconectar", width=12, command=self.on_disconnect).grid(row=0, column=1, **pad)
        tk.Button(conn, text="Salir", width=12, command=self.on_exit, bg="#ff6961").grid(row=0, column=2, **pad)

        # Vuelo
        flight = tk.Frame(self.root, bd=1, relief="groove")
        flight.pack(fill="x", **pad)
        tk.Button(flight, text="Despegar (Enter)", width=16, command=self.on_takeoff, bg="#90ee90").grid(row=0,
                                                                                                         column=0,
                                                                                                         **pad)
        tk.Button(flight, text="Aterrizar (Espacio)", width=16, command=self.on_land, bg="#ff6961").grid(row=0,
                                                                                                         column=1,
                                                                                                         **pad)
        tk.Label(flight, textvariable=self._rec_badge_var, fg="#d00", font=("Arial", 10, "bold")).grid(row=0, column=2,
                                                                                                       sticky="w",
                                                                                                       padx=10)

        # FPV MÁS GRANDE (480x360)
        fpv_frame = tk.Frame(self.root, bd=2, relief="groove", bg="black")
        fpv_frame.pack(fill="both", expand=False, padx=8, pady=8)

        self._fpv_w, self._fpv_h = 480, 360
        fpv_host = tk.Frame(fpv_frame, width=self._fpv_w, height=self._fpv_h, bg="black")
        fpv_host.pack(padx=4, pady=4)
        fpv_host.pack_propagate(False)

        self.fpv_label = tk.Label(fpv_host, text="FPV (detenido)", bg="black", fg="white", anchor="center",
                                  font=("Arial", 10))
        self.fpv_label.pack(fill="both", expand=True)

        # Controles FPV
        fpv_controls = tk.Frame(fpv_frame, bg="black")
        fpv_controls.pack(fill="x", padx=4, pady=(0, 4))

        tk.Button(fpv_controls, text="▶ FPV", command=self.start_fpv,
                  bg="#28a745", fg="white", font=("Arial", 9, "bold"), bd=0,
                  activebackground="#218838", padx=8).pack(side="left", padx=2)
        tk.Button(fpv_controls, text="⏸", command=self.stop_fpv,
                  bg="#6c757d", fg="white", font=("Arial", 12), bd=0, width=3,
                  activebackground="#5a6268").pack(side="left", padx=2)
        tk.Button(fpv_controls, text="📷", command=self.take_snapshot,
                  bg="#17a2b8", fg="white", font=("Arial", 12), bd=0, width=3,
                  activebackground="#138496").pack(side="left", padx=2)
        tk.Button(fpv_controls, text="⏺", command=self.toggle_recording,
                  bg="#dc3545", fg="white", font=("Arial", 12), bd=0, width=3,
                  activebackground="#c82333").pack(side="left", padx=2)

        tk.Label(fpv_controls, textvariable=self._joy_label_var, fg="white", bg="black", font=("Arial", 8)).pack(
            side="right", padx=4)

        # Mapa y Geofence (controles movidos dentro del mapa)
        gf = tk.Frame(self.root, bd=1, relief="groove")
        gf.pack(fill="x", **pad)
        tk.Button(gf, text="🗺 Abrir Mapa y Geofence", command=self.open_map_window,
                  bg="#87CEEB", font=("Arial", 10, "bold"), padx=20, pady=5).pack(pady=6)

        # Nota compacta
        note = tk.Label(self.root, fg="#555", font=("Arial", 8),
                        text="Joystick: IZQ=XY | DER=altura+yaw | B0=Foto | B1=Video | B2=Despegar | B3=Aterrizar")
        note.pack(pady=2)

        # POSE panel compacto
        pose_panel = tk.Frame(self.root, bd=1, relief="groove")
        pose_panel.pack(fill="x", **pad)
        tk.Label(pose_panel, textvariable=self.x_var, width=10, anchor="w", font=("Arial", 9)).grid(row=0, column=0,
                                                                                                    padx=4, pady=2,
                                                                                                    sticky="w")
        tk.Label(pose_panel, textvariable=self.y_var, width=10, anchor="w", font=("Arial", 9)).grid(row=0, column=1,
                                                                                                    padx=4, pady=2,
                                                                                                    sticky="w")
        tk.Label(pose_panel, textvariable=self.z_var, width=10, anchor="w", font=("Arial", 9)).grid(row=0, column=2,
                                                                                                    padx=4, pady=2,
                                                                                                    sticky="w")
        tk.Label(pose_panel, textvariable=self.yaw_var, width=12, anchor="w", font=("Arial", 9)).grid(row=0, column=3,
                                                                                                      padx=4, pady=2,
                                                                                                      sticky="w")

        # VELOCIDADES
        vel_panel = tk.Frame(self.root, bd=1, relief="groove")
        vel_panel.pack(fill="x", **pad)
        tk.Label(vel_panel, textvariable=self.vx_var, width=12, anchor="w", font=("Arial", 9)).grid(row=0, column=0,
                                                                                                    padx=4, pady=2,
                                                                                                    sticky="w")
        tk.Label(vel_panel, textvariable=self.vy_var, width=12, anchor="w", font=("Arial", 9)).grid(row=0, column=1,
                                                                                                    padx=4, pady=2,
                                                                                                    sticky="w")
        tk.Label(vel_panel, textvariable=self.vz_var, width=12, anchor="w", font=("Arial", 9)).grid(row=0, column=2,
                                                                                                    padx=4, pady=2,
                                                                                                    sticky="w")

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
        except Exception as e:
            messagebox.showerror("Conectar", f"No se pudo conectar: {e}")

    def on_disconnect(self):
        self._stop_keepalive()
        self.stop_fpv()
        self._stop_recording()
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
            self.x_var.set("X: —")
            self.y_var.set("Y: —")
            self.z_var.set("Z: —")
            self.yaw_var.set("Yaw: —")
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
                self.x_var.set(f"X: {x:.1f} cm" if x is not None else "X: —")
                self.y_var.set(f"Y: {y:.1f} cm" if y is not None else "Y: —")
                self.z_var.set(f"Z: {z:.1f} cm" if z is not None else "Z: —")
                self.yaw_var.set(f"Yaw: {yaw:.1f}°" if yaw is not None else "Yaw: —")
                if x is not None and y is not None:
                    try:
                        self._last_land_x = float(x)
                        self._last_land_y = float(y)
                        self._last_land_z = float(z) if z is not None else 0.0
                        self._last_land_yaw = float(yaw) if yaw is not None else 0.0
                    except Exception:
                        pass
            else:
                self.x_var.set("X: —")
                self.y_var.set("Y: —")
                self.z_var.set("Z: —")
                self.yaw_var.set("Yaw: —")

            # VELOCIDADES
            vx = getattr(self.dron, "vx_cm_s", None)
            vy = getattr(self.dron, "vy_cm_s", None)
            vz = getattr(self.dron, "vz_cm_s", None)
            self.vx_var.set(f"Vx: {vx} cm/s" if vx is not None else "Vx: —")
            self.vy_var.set(f"Vy: {vy} cm/s" if vy is not None else "Vy: —")
            self.vz_var.set(f"Vz: {vz} cm/s" if vz is not None else "Vz: —")
            # DEBUG
            if vx is None or vy is None or vz is None:
                print(f"[DEBUG VELOCIDADES] vx={vx}, vy={vy}, vz={vz}")

            self._update_map_drone()
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
        path = os.path.join(self._shots_dir, f"shot_{_ts()}.png")
        try:
            cv2.imwrite(path, frame)
            self._hud_show(f"Guardada", 1.8)
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
        self._rec_path = os.path.join(self._recs_dir, f"rec_{_ts()}.mp4")
        self._rec_running = True
        self._rec_writer = None
        self._hud_show("Grabando", 1.5)

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

            # Flecha de yaw (rotación) - roja corta
            if yaw is not None:
                th = math.radians(yaw)
                arrow_len = 20
                dx_world = arrow_len * math.cos(th)
                dy_world = arrow_len * math.sin(th)
                dx_canvas = dy_world * PX_PER_CM
                dy_canvas = -dx_world * PX_PER_CM

                self.map_canvas.create_line(
                    cx_px, cy_px,
                    cx_px + dx_canvas, cy_px + dy_canvas,
                    fill="red", width=2, arrow=tk.LAST, tags="drone"
                )

            # Flecha de velocidad (dirección de movimiento) - azul larga
            vx = getattr(pose, "vx", None) or getattr(pose, "vel_x", None)
            vy = getattr(pose, "vy", None) or getattr(pose, "vel_y", None)
            if vx is not None and vy is not None:
                speed = math.sqrt(vx**2 + vy**2)
                if speed > 5:  # Solo mostrar si hay movimiento significativo
                    # Normalizar y escalar la flecha
                    arrow_len_vel = min(50, speed * 2)  # Escalar con velocidad
                    dx_vel = (vx / speed) * arrow_len_vel * PX_PER_CM
                    dy_vel = -(vy / speed) * arrow_len_vel * PX_PER_CM

                    self.map_canvas.create_line(
                        cx_px, cy_px,
                        cx_px + dy_vel, cy_px + dx_vel,
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


#MAIN
if __name__ == "__main__":
    root = tk.Tk()
    app = MiniRemoteApp(root)
    root.mainloop()