import pygame
import time
import os
import threading

#Permitir eventos de joystick en segundo plano
os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")


class JoystickController:


    def __init__(self,
                 axis_left_x: int = 0,
                 axis_left_y: int = 1,
                 axis_right_x: int = 2,
                 axis_right_y: int = 4,
                 invert_left_y: bool = True,
                 invert_right_y: bool = True,
                 deadzone: float = 0.1,  #Umbral donde los valores pequeños del joystick se consideran 0 (debido a que los joysticks no devuelven exactamente 0 al soltarlos)
                 expo: float = 1.5): #Curva para ajustar la sensibilidad del joystick (si fuese 1.0, es muy probable que los movimientos suaves fuesen bruscos)

        self.axis_left_x = axis_left_x
        self.axis_left_y = axis_left_y
        self.axis_right_x = axis_right_x
        self.axis_right_y = axis_right_y

        self.invert_left_y = invert_left_y
        self.invert_right_y = invert_right_y

        self.deadzone = deadzone
        self.expo = expo

        self.joystick = None
        self._lock = threading.Lock()  # Lock para sincronizar acceso
        self._disconnecting = False

    def connect(self, full_reinit: bool = False) -> bool:
        """
        Conecta al joystick.

        Args:
            full_reinit: Si True, reinicia pygame completamente (usar para reconexión)
        """
        try:
            # Para reconexión: limpiar todo completamente
            if full_reinit:
                print("[Joystick] Reinicio completo de pygame...")
                try:
                    if self.joystick:
                        self.joystick.quit()
                        self.joystick = None
                except:
                    pass
                try:
                    pygame.joystick.quit()
                except:
                    pass
                try:
                    pygame.quit()
                except:
                    pass

                # Esperar a que DirectX libere los recursos
                time.sleep(0.3)

                # Reinicializar pygame desde cero
                pygame.init()
            else:
                # Inicialización normal (primera vez)
                if not pygame.get_init():
                    pygame.init()
                pygame.joystick.init()

            # Verificar joysticks disponibles
            if pygame.joystick.get_count() == 0:
                print("[Joystick] No hay joysticks conectados")
                return False

            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()

            print(f"[Joystick] Conectado: {self.joystick.get_name()}")
            print(f"[Joystick] Ejes: {self.joystick.get_numaxes()}")
            print(f"[Joystick] Botones: {self.joystick.get_numbuttons()}")

            return True

        except pygame.error as e:
            print(f"[Joystick] Error pygame: {e}")
            return False
        except Exception as e:
            print(f"[Joystick] Error: {e}")
            return False

    def _apply_deadzone(self, value: float) -> float:

        if abs(value) < self.deadzone:
            return 0.0
        return value

    def _apply_expo(self, value: float) -> float:

        if value == 0:
            return 0.0
        sign = 1 if value >= 0 else -1
        return sign * (abs(value) ** self.expo)

    def read_axes(self) -> tuple:
        # Verificar si se está desconectando
        if self._disconnecting:
            return (0, 0, 0, 0)

        if not self.joystick:
            return (0, 0, 0, 0)

        try:
            # Actualizar eventos de pygame (necesario para leer valores)
            pygame.event.pump()
        except Exception:
            return (0, 0, 0, 0)

        # Leer ejes  del joystick
        left_x = self.joystick.get_axis(self.axis_left_x)
        left_y = self.joystick.get_axis(self.axis_left_y)
        right_x = self.joystick.get_axis(self.axis_right_x)
        right_y = self.joystick.get_axis(self.axis_right_y)

        # Invertir ejes si es necesario
        # (algunos joysticks tienen arriba=-1, queremos arriba=+1)
        if self.invert_left_y:
            left_y = -left_y
        if self.invert_right_y:
            right_y = -right_y

        # Aplicar zona muerta
        left_x = self._apply_deadzone(left_x)
        left_y = self._apply_deadzone(left_y)
        right_x = self._apply_deadzone(right_x)
        right_y = self._apply_deadzone(right_y)

        # Aplicar curva exponencial
        left_x = self._apply_expo(left_x)
        left_y = self._apply_expo(left_y)
        right_x = self._apply_expo(right_x)
        right_y = self._apply_expo(right_y)

        # Convertir a valores RC (-100 a +100)
        vx = int(left_x * 100)  # Izquierda/Derecha
        vy = int(left_y * 100)  # Adelante/Atrás
        vz = int(right_y * 100)  # Arriba/Abajo
        yaw = int(right_x * 100)  # Rotación

        # Limitar al rango válido (seguridad)
        vx = max(-100, min(100, vx))
        vy = max(-100, min(100, vy))
        vz = max(-100, min(100, vz))
        yaw = max(-100, min(100, yaw))

        return (vx, vy, vz, yaw)

    def get_button(self, button_index: int) -> bool:

        if not self.joystick:
            return False

        pygame.event.pump()
        return self.joystick.get_button(button_index)

    def disconnect(self):

        with self._lock:
            self._disconnecting = True
            try:
                if self.joystick:
                    self.joystick.quit()
                    self.joystick = None
                pygame.joystick.quit()
                pygame.quit()
            except Exception as e:
                print(f"[Joystick] Error al desconectar: {e}")
            finally:
                self._disconnecting = False