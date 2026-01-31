# Uso básico de TelloLink

## Vuelo mínimo

```python
from TelloLink import TelloDron

dron = TelloDron()
dron.connect()
dron.startTelemetry()

dron.takeOff(0.5, blocking=True)
dron.forward(100)
dron.Land(blocking=True)

dron.disconnect()
```

## Movimiento y rotación

```python
dron.forward(100)   # Avanzar 100 cm
dron.back(50)       # Retroceder 50 cm
dron.left(80)       # Izquierda 80 cm
dron.right(80)      # Derecha 80 cm
dron.up(30)         # Subir 30 cm
dron.down(30)       # Bajar 30 cm

dron.cw(90)         # Girar 90° en sentido horario
dron.ccw(45)        # Girar 45° en sentido antihorario

dron.set_speed(50)  # Fijar velocidad a 50 cm/s
```

## Cámara y capturas

```python
dron.stream_on()              # Activar streaming de vídeo
frame = dron.get_frame()      # Obtener último frame (numpy array)
ruta = dron.snapshot()        # Guardar foto en disco
dron.stream_off()             # Desactivar streaming
```

## Vídeo en ventana OpenCV

```python
dron.start_video()                          # Abrir ventana de vídeo (hilo separado)
dron.stop_video()                           # Cerrar ventana

dron.show_video_blocking(resize=(640, 480)) # Ventana bloqueante (pulsar 'q' para salir)
```

## Navegación por coordenadas

```python
# Mover el dron 100 cm en X, 50 cm en Y, subir 30 cm en Z
dron.goto_rel(dx_cm=100, dy_cm=50, dz_cm=30, blocking=True)

# Consultar posición actual estimada (dead reckoning)
print(dron.pose)  # PoseVirtual(x=100.0, y=50.0, z=80.0, yaw=0.0)
```

## Misiones automáticas

```python
waypoints = [
    {"x": 100, "y": 0,   "z": 80},
    {"x": 100, "y": 100, "z": 80},
    {"x": 0,   "y": 100, "z": 80, "photo": True},
    {"x": 0,   "y": 0,   "z": 80}
]

dron.run_mission(waypoints, do_land=True, blocking=True)
```

## Geofence

```python
# Definir zona de vuelo permitida (200x200 cm, altura máxima 150 cm)
dron.set_geofence(max_x_cm=200, max_y_cm=200, max_z_cm=150)

# Añadir zona de exclusión circular (centro 100,100 radio 30 cm)
dron.add_exclusion_circle(cx=100, cy=100, r_cm=30)

# Desactivar geofence
dron.disable_geofence()
```

## Joystick

```python
from TelloLink import JoystickController

joy = JoystickController()
joy.connect()

# Leer ejes convertidos a valores RC (-100 a 100)
vx, vy, vz, yaw = joy.read_axes()

# Enviar al dron en modo RC (control continuo)
dron.rc(vx, vy, vz, yaw)
```

## Despegue no bloqueante con callback

```python
def on_takeoff_done(success):
    if success:
        print("Despegue completado")
    else:
        print("Despegue fallido")

dron.takeOff(0.5, blocking=False, callback=on_takeoff_done)
```
