# Módulos de TelloLink

La clase principal `TelloDron` (definida en `Tello.py`) importa los métodos de cada módulo. La aplicación solo necesita instanciar `TelloDron()` para acceder a todas las funcionalidades.

## Estructura

```
TelloLink/
├── Tello.py          ← Clase principal TelloDron
├── __init__.py       ← Exporta TelloDron y JoystickController
└── modules/
    ├── tello_connect.py
    ├── tello_takeOff.py
    ├── tello_land.py
    ├── tello_move.py
    ├── tello_heading.py
    ├── tello_telemetry.py
    ├── tello_camera.py
    ├── tello_video.py
    ├── tello_pose.py
    ├── tello_goto.py
    ├── tello_mission.py
    ├── tello_geofence.py
    ├── tello_geometry.py
    ├── tello_session.py
    ├── tello_scenario.py
    └── tello_joystick.py
```

## Descripción de módulos

### tello_connect.py — Conexión y desconexión

Gestiona la comunicación WiFi con el dron mediante la librería djitellopy.

| Método | Descripción |
|--------|-------------|
| `connect(freq, blocking, callback, params)` | Conecta al dron. Soporta modo bloqueante y no bloqueante |
| `disconnect()` | Cierra la conexión, para telemetría y libera recursos |

### tello_takeOff.py — Despegue

Controla el despegue con verificación de altura y protección contra peticiones duplicadas.

| Método | Descripción |
|--------|-------------|
| `takeOff(altura_objetivo_m, blocking, callback)` | Despega a la altura indicada (por defecto 0.5 m). Soporta callback |

### tello_land.py — Aterrizaje

Gestiona el aterrizaje con lectura de altura en tiempo real.

| Método | Descripción |
|--------|-------------|
| `Land(blocking, callback, params)` | Aterriza el dron. Verifica estado y altura antes de enviar el comando |

### tello_move.py — Movimiento lineal

Movimientos en los 6 ejes con distancia acotada por el SDK (20–500 cm).

| Método | Descripción |
|--------|-------------|
| `forward(dist_cm)` | Avanzar |
| `back(dist_cm)` | Retroceder |
| `left(dist_cm)` | Desplazar a la izquierda |
| `right(dist_cm)` | Desplazar a la derecha |
| `up(dist_cm)` | Subir (respeta techo de seguridad) |
| `down(dist_cm)` | Bajar (respeta altura actual) |
| `set_speed(speed_cm_s)` | Fijar velocidad (10–100 cm/s) |
| `rc(vx, vy, vz, yaw)` | Control RC continuo (-100 a 100) |

### tello_heading.py — Rotación

Giro del dron en el eje vertical con fragmentación en pasos de 360° máximo.

| Método | Descripción |
|--------|-------------|
| `rotate(deg)` | Girar los grados indicados (positivo = horario) |
| `cw(deg)` | Atajo para giro horario |
| `ccw(deg)` | Atajo para giro antihorario |

### tello_telemetry.py — Telemetría

Hilo en segundo plano que lee periódicamente altura, batería, temperatura, WiFi y yaw del dron.

| Método | Descripción |
|--------|-------------|
| `startTelemetry(period_s)` | Inicia el hilo de lectura de telemetría |
| `stopTelemetry()` | Detiene el hilo de telemetría |

### tello_camera.py — Cámara

Acceso directo al streaming de vídeo y captura de imágenes.

| Método | Descripción |
|--------|-------------|
| `stream_on()` | Activa el streaming de la cámara del dron |
| `stream_off()` | Desactiva el streaming |
| `get_frame()` | Devuelve el último frame como array numpy |
| `snapshot(path)` | Guarda una foto en disco (JPG) |

### tello_video.py — Ventana de vídeo

Visualización del vídeo en ventanas OpenCV.

| Método | Descripción |
|--------|-------------|
| `start_video(window_name, resize)` | Abre ventana de vídeo en hilo separado |
| `stop_video()` | Cierra la ventana de vídeo |
| `show_video_blocking(window_name, resize)` | Ventana bloqueante (pulsar 'q' para cerrar) |

### tello_pose.py — Posicionamiento virtual

Sistema de dead reckoning que estima la posición del dron en coordenadas relativas al punto de despegue.

| Clase / Método | Descripción |
|--------|-------------|
| `PoseVirtual` | Dataclass con x_cm, y_cm, z_cm, yaw_deg |
| `reset()` | Reinicia la pose al origen |
| `update_move(direction, dist_cm)` | Actualiza posición tras un movimiento |
| `update_yaw(delta_deg)` | Actualiza el yaw relativo |
| `distance_to(other)` | Distancia euclidiana a otra pose |
| `set_from_telemetry(height_cm, yaw_deg)` | Sincroniza con datos reales del dron |
| `capture()` | Devuelve diccionario con la pose actual |

### tello_goto.py — Navegación por coordenadas

Movimiento a posiciones relativas usando el comando SDK "go x y z speed".

| Método | Descripción |
|--------|-------------|
| `goto_rel(dx_cm, dy_cm, dz_cm, ...)` | Mueve el dron a una posición relativa. Soporta face_target, speed y callback |
| `abort_goto()` | Aborta el movimiento en curso |

### tello_mission.py — Misiones automáticas

Ejecución secuencial de waypoints con acciones (foto, vídeo) en cada punto.

| Método | Descripción |
|--------|-------------|
| `run_mission(waypoints, do_land, face_target, ...)` | Ejecuta la misión. Callbacks: on_wp, on_action, on_finish |
| `abort_mission()` | Aborta la misión en curso |

### tello_geofence.py — Geofence

Delimita la zona de vuelo permitida y detecta violaciones en tiempo real mediante un hilo monitor.

| Método | Descripción |
|--------|-------------|
| `set_geofence(max_x_cm, max_y_cm, max_z_cm, ...)` | Activa el geofence con los límites indicados |
| `disable_geofence()` | Desactiva el geofence |
| `recenter_geofence()` | Recentra el geofence a la posición actual del dron |
| `add_exclusion_circle(cx, cy, r_cm, ...)` | Añade zona de exclusión circular |
| `add_exclusion_poly(points, ...)` | Añade zona de exclusión poligonal |
| `clear_exclusions()` | Elimina todas las zonas de exclusión |
| `set_layers(layers)` | Configura las capas de altura |
| `get_layers()` | Devuelve la configuración de capas |
| `get_current_layer()` | Devuelve la capa en la que se encuentra el dron |

### tello_geometry.py — Geometría de colisiones

Funciones de intersección usadas por el sistema de misiones para validar rutas contra obstáculos.

| Función | Descripción |
|--------|-------------|
| `validate_mission_paths(waypoints, obstacles)` | Valida que ningún tramo de la misión cruce un obstáculo |
| `line_intersects_obstacle(...)` | Comprueba si un segmento intersecta con un obstáculo |
| `point_in_obstacle(...)` | Comprueba si un punto está dentro de un obstáculo |

### tello_session.py — Sesiones de vuelo

Gestión de sesiones para organizar fotos y vídeos capturados durante un vuelo.

| Método | Descripción |
|--------|-------------|
| `start_flight_session()` | Inicia una sesión con carpetas para fotos y vídeos |
| `end_flight_session()` | Finaliza la sesión activa |
| `get_session_photo_path(filename)` | Devuelve la ruta para guardar una foto en la sesión |
| `get_session_video_path(filename)` | Devuelve la ruta para guardar un vídeo en la sesión |

### tello_scenario.py — Escenarios

Gestión de escenarios que combinan geofence, obstáculos, capas y planes de vuelo en ficheros JSON.

| Función / Clase | Descripción |
|--------|-------------|
| `ScenarioManager` | Clase para crear, guardar, cargar y listar escenarios |
| `get_scenario_manager(base_dir)` | Devuelve la instancia singleton del gestor de escenarios |

### tello_joystick.py — Controlador de joystick

Lectura de un mando USB/Bluetooth mediante pygame, con zona muerta y curva expo.

| Clase / Método | Descripción |
|--------|-------------|
| `JoystickController` | Clase para gestionar el joystick |
| `connect(full_reinit)` | Conecta al joystick detectado |
| `read_axes()` | Devuelve tupla (vx, vy, vz, yaw) en rango -100 a 100 |
| `disconnect()` | Desconecta el joystick |
