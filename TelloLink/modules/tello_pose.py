
import  math
from dataclasses import dataclass


# Función para mantener siempre el ángulo entre 0 y 360 grados
def _wrap_deg(deg: float) -> float:
    d = deg % 360.0
    return d if d >= 0 else d + 360.0


@dataclass
class PoseVirtual:
    x_cm: float = 0.0
    y_cm: float = 0.0
    z_cm: float = 0.0
    yaw_deg: float = 0.0

    # Referencia de yaw en el momento del despegue (para yaw relativo = 0) ---
    yaw0_deg: float = 0.0

    # Velocidad actual en cm/s (para flecha de dirección de movimiento)
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0

    # Métodos básicos
    def reset(self) -> None:
        # Reinicia la pose al origen (punto de despegue).
        self.x_cm = 0.0
        self.y_cm = 0.0
        self.z_cm = 0.0
        self.yaw_deg = 0.0
        # También reseteamos la referencia
        self.yaw0_deg = 0.0
        # Limpiar velocidades
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0

    def capture(self) -> dict:
        # Devuelve la pose actual y se redondea a un decimal
        return {
            "x_cm": round(self.x_cm, 1),
            "y_cm": round(self.y_cm, 1),
            "z_cm": round(self.z_cm, 1),
            "yaw_deg": round(self.yaw_deg, 1),
        }



    def set_from_telemetry(self, height_cm: float | None = None,
                           yaw_deg: float | None = None) -> None:
        if height_cm is not None:
            self.z_cm = float(height_cm)
        if yaw_deg is not None:
            # Interpretamos yaw_deg como yaw ABSOLUTO del Tello y lo pasamos a relativo
            self.set_heading_from_absolute_yaw(float(yaw_deg))

    def update_yaw(self, delta_deg: float) -> None:
        # Delta relativo (cw positivo) sobre el yaw relativo actual
        self.yaw_deg = _wrap_deg(self.yaw_deg + float(delta_deg))

    def update_move(self, direction: str, dist_cm: float) -> None:

        d = float(dist_cm)
        yaw = math.radians(self.yaw_deg)  # usamos el yaw RELATIVO

        if direction == "forward":
            self.x_cm += d * math.cos(yaw)
            self.y_cm += d * math.sin(yaw)

        elif direction == "back":
            self.x_cm -= d * math.cos(yaw)
            self.y_cm -= d * math.sin(yaw)

        elif direction == "right":
            self.x_cm -= d * math.sin(yaw)
            self.y_cm += d * math.cos(yaw)

        elif direction == "left":
            self.x_cm += d * math.sin(yaw)
            self.y_cm -= d * math.cos(yaw)

        elif direction == "up":
            self.z_cm += d

        elif direction == "down":
            self.z_cm -= d

    # Distancia entre una pose y otra
    def distance_to(self, other: "PoseVirtual") -> float:
        dx = self.x_cm - other.x_cm
        dy = self.y_cm - other.y_cm
        dz = self.z_cm - other.z_cm
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def set_takeoff_reference(self, yaw_abs_deg: float | None):

        if yaw_abs_deg is None:
            self.yaw0_deg = 0.0
        else:
            self.yaw0_deg = float(yaw_abs_deg) % 360.0
        # Al fijar la referencia, ponemos el yaw relativo a 0 (no tocamos x/y/z)
        self.yaw_deg = 0.0

    def set_heading_from_absolute_yaw(self, yaw_abs_deg: float):

        abs_norm = float(yaw_abs_deg) % 360.0
        zero = float(self.yaw0_deg or 0.0) % 360.0
        rel = (abs_norm - zero) % 360.0
        self.yaw_deg = rel

    def __repr__(self) -> str:
        return (f"PoseVirtual(x={self.x_cm:.1f}, y={self.y_cm:.1f}, "
                f"z={self.z_cm:.1f}, yaw={self.yaw_deg:.1f})")


#A partir de usar el joystick (modo rc), la pose se calcula de esta manera, a partir de las velocidades del joystick.
    def update_from_rc(self, vx_pct, vy_pct, vz_pct, yaw_pct, dt_sec=0.1):

        import math

        # Velocidad máxima del Tello en modo "slow" al usar el joystick. Es un valor que se encuentra en el SDK, el cual está en torno a 2-2.1 m/s
        MAX_SPEED_CM_S = 210  # cm/s
        MAX_YAW_DEG_S = 100  # grados/s

        # Convertir porcentajes de la velocidad leída del joystick (-100 a 100) a velocidades reales
        vx_cm_s = (vx_pct / 100.0) * MAX_SPEED_CM_S
        vy_cm_s = (vy_pct / 100.0) * MAX_SPEED_CM_S
        vz_cm_s = (vz_pct / 100.0) * MAX_SPEED_CM_S
        yaw_deg_s = (yaw_pct / 100.0) * MAX_YAW_DEG_S

        # Calcular desplazamiento en el tiempo dt, calcula cuantos centímetros se mueve el dron en el intervalo dt_sec
        dx_local = vx_cm_s * dt_sec  # adelante/atrás (en referencia del dron)
        dy_local = vy_cm_s * dt_sec  # izquierda/derecha (en referencia del dron)
        dz = vz_cm_s * dt_sec  # arriba/abajo
        dyaw = yaw_deg_s * dt_sec  # rotación

        # Convertir movimiento local relativo al dron a las coordenadas globales
        theta = math.radians(self.yaw_deg)
        cos_theta = math.cos(theta)
        sin_theta = math.sin(theta)

        # Rotación del movimiento al sistema global (yaw=0 → forward=+X, right=+Y)
        dx_global = dx_local * cos_theta - dy_local * sin_theta
        dy_global = dx_local * sin_theta + dy_local * cos_theta

        # Actualizar la posición y la orientación sumando el desplazamiento que se calcula en cada dt
        self.x_cm += dx_global
        self.y_cm += dy_global
        self.z_cm += dz
        self.yaw_deg = (self.yaw_deg + dyaw) % 360.0