import tkinter as tk
from tkinter import messagebox
import sys

try:
    from TelloLink.Tello import TelloDron
    HAVE_DRON = True
except Exception:
    TelloDron = None
    HAVE_DRON = False


# === Parámetros de la cuadrícula / escala ===
CANVAS_SIZE = 520           # lienzo cuadrado (px)
GRID_STEP_PX = 20           # separación de líneas de la cuadrícula
CM_PER_PX = 2.0             # escala: 1 px = 2 cm  (ajústalo si quieres)


class GeofenceDesigner(tk.Frame):
    def __init__(self, master):
        super().__init__(master)
        self.pack(fill="both", expand=True)

        # Estado dron (opcional)
        self.dron = TelloDron() if HAVE_DRON else None
        self.state_var = tk.StringVar(value="disconnected" if HAVE_DRON else "no-dron")

        # UI superior
        top = tk.Frame(self)
        top.pack(fill="x", padx=6, pady=6)

        tk.Label(top, text="Estado:").pack(side="left")
        tk.Label(top, textvariable=self.state_var, width=12).pack(side="left", padx=4)

        tk.Button(top, text="Conectar", command=self.on_connect, bg="#ffb347").pack(side="left", padx=4)
        tk.Button(top, text="Desconectar", command=self.on_disconnect).pack(side="left", padx=4)

        # Canvas + panel botones
        mid = tk.Frame(self)
        mid.pack(padx=6, pady=6)

        self.canvas = tk.Canvas(mid, width=CANVAS_SIZE, height=CANVAS_SIZE, bg="white")
        self.canvas.grid(row=0, column=0, padx=6, pady=6)

        side = tk.Frame(mid)
        side.grid(row=0, column=1, sticky="ns")

        tk.Button(side, text="Aplicar al dron", command=self.apply_to_drone, bg="#90ee90").pack(fill="x", pady=4)
        tk.Button(side, text="Borrar", command=self.clear_rect).pack(fill="x", pady=4)

        # Info numérica
        self.info_var = tk.StringVar(value="Dibuja un rectángulo con click-arrastrar")
        tk.Label(self, textvariable=self.info_var, fg="#555").pack(pady=4)

        # Eventos
        self.canvas.bind("<ButtonPress-1>", self.on_press)
        self.canvas.bind("<B1-Motion>", self.on_drag)
        self.canvas.bind("<ButtonRelease-1>", self.on_release)

        # Estado del rectángulo
        self._start = None
        self._preview_id = None
        self._rect_cm = None  # (xmin, xmax, ymin, ymax) en cm

        # Dibuja cuadrícula y ejes
        self.draw_grid()

    # --- Dron ---
    def on_connect(self):
        if not HAVE_DRON:
            messagebox.showinfo("Info", "Módulo Tello no disponible; modo diseño offline.")
            return
        if self.dron.state == "connected":
            return
        try:
            self.dron.connect()
            self.state_var.set(self.dron.state)
            messagebox.showinfo("Conectar", "Conectado.")
        except Exception as e:
            messagebox.showerror("Conectar", f"No se pudo conectar: {e}")

    def on_disconnect(self):
        if not HAVE_DRON:
            return
        try:
            self.dron.disconnect()
        except Exception:
            pass
        finally:
            self.state_var.set("disconnected")

    # --- Cuadrícula y ejes ---
    def draw_grid(self):
        c = self.canvas
        c.delete("all")

        # fondo
        c.create_rectangle(0, 0, CANVAS_SIZE, CANVAS_SIZE, fill="white", outline="")

        # líneas de cuadrícula
        for x in range(0, CANVAS_SIZE + 1, GRID_STEP_PX):
            c.create_line(x, 0, x, CANVAS_SIZE, fill="#eee")
        for y in range(0, CANVAS_SIZE + 1, GRID_STEP_PX):
            c.create_line(0, y, CANVAS_SIZE, y, fill="#eee")

        # ejes centrales
        cx, cy = self.center_px()
        c.create_line(0, cy, CANVAS_SIZE, cy, fill="#bbb")  # eje X (y=0)
        c.create_line(cx, 0, cx, CANVAS_SIZE, fill="#bbb")  # eje Y (x=0)

        # marcas de origen
        c.create_oval(cx - 3, cy - 3, cx + 3, cy + 3, outline="#333", fill="#333")

        # Si había un rectángulo previo, redibújalo
        if self._rect_cm is not None:
            self.draw_rect_from_cm(*self._rect_cm)

    def center_px(self):
        return CANVAS_SIZE // 2, CANVAS_SIZE // 2

    # --- Conversión px <-> cm (0,0 centrado en canvas) ---
    def px_to_cm(self, x_px, y_px):
        cx, cy = self.center_px()
        dx_px = x_px - cx
        dy_px = cy - y_px  # invertir Y para que ↑ sea +Y
        return dx_px * CM_PER_PX, dy_px * CM_PER_PX

    def cm_to_px(self, x_cm, y_cm):
        cx, cy = self.center_px()
        return int(cx + x_cm / CM_PER_PX), int(cy - y_cm / CM_PER_PX)

    # --- Interacción del ratón ---
    def on_press(self, e):
        self._start = (e.x, e.y)
        # limpia preview anterior
        if self._preview_id:
            self.canvas.delete(self._preview_id)
            self._preview_id = None

    def on_drag(self, e):
        if not self._start:
            return
        x0, y0 = self._start
        x1, y1 = e.x, e.y
        # borra preview anterior
        if self._preview_id:
            self.canvas.delete(self._preview_id)
        self._preview_id = self.canvas.create_rectangle(
            x0, y0, x1, y1, outline="#1e90ff", width=2, dash=(4, 3)
        )

        # muestra info en cm
        (xmin_cm, xmax_cm, ymin_cm, ymax_cm) = self.rect_px_to_cm(x0, y0, x1, y1)
        self.info_var.set(f"Inclusión provisional: X[{xmin_cm:.0f},{xmax_cm:.0f}] cm  "
                          f"Y[{ymin_cm:.0f},{ymax_cm:.0f}] cm")

    def on_release(self, e):
        if not self._start:
            return
        x0, y0 = self._start
        x1, y1 = e.x, e.y
        self._start = None

        (xmin_cm, xmax_cm, ymin_cm, ymax_cm) = self.rect_px_to_cm(x0, y0, x1, y1)
        # guarda rectángulo final en cm
        self._rect_cm = (xmin_cm, xmax_cm, ymin_cm, ymax_cm)

        # redibuja limpio
        self.draw_grid()
        self.draw_rect_from_cm(xmin_cm, xmax_cm, ymin_cm, ymax_cm)
        self.info_var.set(f"Inclusión: X[{xmin_cm:.0f},{xmax_cm:.0f}] cm  "
                          f"Y[{ymin_cm:.0f},{ymax_cm:.0f}] cm")

    def rect_px_to_cm(self, x0, y0, x1, y1):
        # normaliza px
        xa, xb = sorted([x0, x1])
        ya, yb = sorted([y0, y1])
        # esquinas a cm
        x_min_cm, y_max_cm = self.px_to_cm(xa, ya)  # ojo: ya es top
        x_max_cm, y_min_cm = self.px_to_cm(xb, yb)  # yb es bottom
        # devuelve (xmin, xmax, ymin, ymax)
        return (min(x_min_cm, x_max_cm), max(x_min_cm, x_max_cm),
                min(y_min_cm, y_max_cm), max(y_min_cm, y_max_cm))

    def draw_rect_from_cm(self, xmin_cm, xmax_cm, ymin_cm, ymax_cm):
        x0, y0 = self.cm_to_px(xmin_cm, ymax_cm)  # top-left
        x1, y1 = self.cm_to_px(xmax_cm, ymin_cm)  # bottom-right
        self.canvas.create_rectangle(x0, y0, x1, y1, outline="#0a0", width=2)
        # sombreado suave
        self.canvas.create_rectangle(x0, y0, x1, y1, outline="", fill="#00ff0040")

    # --- Aplicar al dron ---
    def apply_to_drone(self):
        if self._rect_cm is None:
            messagebox.showwarning("Geofence", "Dibuja primero un rectángulo de inclusión.")
            return
        xmin, xmax, ymin, ymax = self._rect_cm

        # Validación mínima
        if xmax - xmin < 20 or ymax - ymin < 20:
            messagebox.showwarning("Geofence", "La zona es demasiado pequeña.")
            return

        if not HAVE_DRON:
            messagebox.showinfo(
                "Geofence (offline)",
                f"Aplicaría set_geofence(x_min={xmin:.0f}, x_max={xmax:.0f}, "
                f"y_min={ymin:.0f}, y_max={ymax:.0f})"
            )
            return

        if getattr(self.dron, "state", "") != "connected":
            messagebox.showwarning("Geofence", "Conecta al dron antes de aplicar.")
            return

        try:
            self.dron.set_geofence(x_min=int(xmin), x_max=int(xmax),
                                   y_min=int(ymin), y_max=int(ymax))
            messagebox.showinfo("Geofence", "Zona de inclusión aplicada al dron.")
        except Exception as e:
            messagebox.showerror("Geofence", f"No se pudo aplicar: {e}")

    def clear_rect(self):
        self._rect_cm = None
        self._preview_id = None
        self.draw_grid()
        self.info_var.set("Dibuja un rectángulo con click-arrastrar")


def main():
    root = tk.Tk()
    root.title("Geofence Designer (inclusión)")
    root.geometry(f"{CANVAS_SIZE+170}x{CANVAS_SIZE+120}")

    if sys.platform == "darwin":
        root.configure(bg="#f0f0f0")
        root.tk_setPalette(
            background="#f0f0f0",
            foreground="black",
            activeBackground="#ffb347",
            activeForeground="black"
        )

    GeofenceDesigner(root)
    root.mainloop()


if __name__ == "__main__":
    main()