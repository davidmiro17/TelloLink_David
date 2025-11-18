
import time
from TelloLink.Tello import TelloDron

dron = TelloDron()

print("Conectando...")
dron.connect()
print("OK - Conectado")

print("\nActivando mission pads...")
dron.enable_mission_pads()
print("OK - Activado")

print("\n*** LEVANTA EL DRON 30cm DEL SUELO  ***")
print("Esperando 3 segundos...\n")
time.sleep(3)

print("Leyendo 20 veces (cada segundo):\n")

for i in range(20):
    try:
        mid = dron._tello.get_mission_pad_id()
        x = dron._tello.get_mission_pad_distance_x()
        y = dron._tello.get_mission_pad_distance_y()
        z = dron._tello.get_mission_pad_distance_z()

        print(f"{i+1:02d}. mid={mid} | x={x} | y={y} | z={z}")
    except Exception as e:
        print(f"{i+1:02d}. ERROR: {e}")

    time.sleep(1)

print("\nDesconectando...")
dron.disable_mission_pads()
dron.disconnect()
print("FIN")