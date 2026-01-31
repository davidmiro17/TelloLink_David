# TelloLink

Proyecto completo de control de drones DJI Tello desarrollado como Trabajo de Fin de Grado en la EETAC (UPC). Incluye la librería  **TelloLink** y una aplicación de demostración con interfaz gráfica que integra control manual y misiones autónomas.



## Estructura del repositorio

```
TelloDroneLinkPriv/
├── TelloLink/                 # Librería de control del dron
│   ├── modules/               # 16 módulos 
│   ├── tests/                 # Scripts de prueba 
│   ├── docs/                  # Documentación de la librería
│   ├── Tello.py               # Clase principal TelloDron
│   └── __init__.py
├── demostradores/
│   └── minidemotodo.py        # Aplicación 
├── escenarios/                # Escenarios guardados (.json)
└── Extensions/                # Extensiones adicionales
```

## Requisitos previos

- **Python 3.10+**
- **DJI Tello** (compatible con Tello, Tello EDU y Tello Talent)
- **Joystick** (opcional, cualquier mando compatible con pygame)
- **Windows 10/11** (probado en Windows)

## Instalación

1. Clonar el repositorio:
```bash
git clone https://github.com/davidmiro17/TelloLink_David.git
cd TelloLink_David
```

2. Instalar dependencias:
```bash
pip install djitellopy opencv-python pygame numpy shapely Pillow
```

3. Verificar que tkinter está disponible (viene incluido con Python en Windows):
```bash
python -c "import tkinter; print('OK')"
```

## Conexión con el dron

1. Encender el dron pulsando el botón lateral.
2. Esperar a que el LED frontal parpadee en amarillo/verde.
3. Conectar el ordenador a la red WiFi del dron:
   - Con expansion kit (matriz LED): red **RMTT-XXXXXX**
   - Sin expansion kit: red **TELLO-XXXXXX**
4. Una vez conectado, el LED frontal parpadeará en verde.

## Ejecutar la aplicación

```bash
cd demostradores
python minidemotodo.py
```

Se abrirá la ventana principal con los siguientes paneles:

**Ventana principal:**
- Conexión y desconexión del dron
- Despegue y aterrizaje (también con Enter y Espacio)
- Panel de telemetría en tiempo real (batería, posición X/Y/Z, yaw, velocidades, tiempo de vuelo)
- Controles de movimiento con flechas del teclado
- Sliders de paso (cm), velocidad (cm/s) y ángulo (°)
- Botón FPV para abrir ventana de vídeo en directo
- Captura de fotos y grabación de vídeo

**Ventanas adicionales:**
- **Mapa y Geofence:** Visualización de la posición del dron, definición de zonas seguras y obstáculos por capas (círculos, polígonos).
- **Editor de Misiones:** Creación  de rutas con waypoints con acciones automáticas (foto, vídeo, rotación, espera, subir o bajar de capa).
- **Galería de Vuelos:** Navegación por sesiones de vuelo con visor de fotos y vídeos

## Control con joystick

La aplicación detecta automáticamente mandos compatibles con pygame al iniciar.

| Control | Función |
|---|---|
| Palanca izquierda horizontal | Izquierda / Derecha |
| Palanca izquierda vertical | Adelante / Atrás |
| Palanca derecha vertical | Subir / Bajar |
| Palanca derecha horizontal | Rotación (yaw) |
| Botón 0 | Capturar foto |
| Botón 1 | Iniciar/parar grabación |
| Botón 2 | Despegar |
| Botón 3 | Aterrizar |

Si el joystick no se detecta al iniciar, pulsa el icono 🎮 en el panel de telemetría para reconectar.

## Control con teclado

| Tecla | Función |
|---|---|
| ↑ ↓ ← → | Movimiento horizontal |
| Re Pág / Av Pág | Subir / Bajar |
| Q / E | Rotar izquierda / derecha |
| Enter | Despegar |
| Espacio | Aterrizar |

## Escenarios

Los escenarios son archivos JSON que almacenan una configuración completa de vuelo: geofence, obstáculos por capas de altura y planes de vuelo con waypoints. Se guardan en la carpeta `escenarios/` y pueden cargarse desde el Mapa o desde el Editor de Misiones.

Cada escenario puede contener varios planes de vuelo reutilizables, lo que permite definir distintas rutas sobre un mismo entorno.

## Resolución de problemas

**"No se pudo conectar al Tello"**
- Verificar que el ordenador está conectado a la red WiFi del dron (RMTT-XXXXXX o TELLO-XXXXXX).
- Comprobar que no hay otro programa usando el puerto UDP 8889.
- Reiniciar el dron apagándolo y volviéndolo a encender.

**El dron no despega**
- Comprobar que la batería está por encima del 20%.
- Asegurar que el dron está sobre una superficie con textura. El sensor de flujo óptico necesita patrones visuales en el suelo para estabilizarse; superficies muy lisas o brillantes causan inestabilidad.
- Verificar que no hay ningún objeto obstruyendo las hélices.

**El vídeo FPV no aparece o se congela**
- Pulsar el botón FPV para reiniciar la ventana.
- Si usas antivirus o firewall, permitir el tráfico UDP en el puerto 11111.
- Reiniciar la conexión con el dron (desconectar y volver a conectar).

**El joystick no responde**
- Verificar que el mando está conectado antes de iniciar la aplicación.
- Si se desconectó durante el uso, pulsar el icono 🎮 para reconectar.
- Comprobar que pygame detecta el mando: `python -c "import pygame; pygame.init(); pygame.joystick.init(); print(pygame.joystick.get_count())"`.

**Las coordenadas del dron derivan con el tiempo**
- Esto es inherente al sistema de posicionamiento diseñado por flujo óptico (dead reckoning). El sensor acumula error progresivamente.

**"shapely no está instalado"**
- Instalar con `pip install shapely`. Esta librería es necesaria para la validación geométrica de rutas contra obstáculos en el editor de misiones.

**Error de timeout en comandos**
- El dron puede tardar varios segundos en responder a ciertos comandos (despegue, aterrizaje). El timeout está configurado a 15 segundos.
- Si persiste, reiniciar el dron y reconectar.

## Librería TelloLink

La librería se puede usar de forma independiente sin la aplicación. La documentación completa está disponible en el repositorio [TelloLink](https://github.com/davidmiro17/TelloLink).

Ejemplo de uso mínimo:

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
