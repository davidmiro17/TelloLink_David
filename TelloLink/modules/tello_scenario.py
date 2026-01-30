# tello_scenario.py - Gestión de escenarios (salas de museo)
# Un escenario contiene: geofence, capas, obstáculos y planes de vuelo

import os
import json
import re
from datetime import datetime
from typing import Optional, Dict, List, Any


def sanitize_filename(name: str) -> str:

    # Reemplazar caracteres no válidos
    safe = re.sub(r'[<>:"/\\|?*]', '_', name)
    # Reemplazar espacios múltiples por uno
    safe = re.sub(r'\s+', ' ', safe).strip()
    # Limitar longitud
    if len(safe) > 100:
        safe = safe[:100]
    return safe or "sin_nombre"


class ScenarioManager:


    def __init__(self, base_dir: str = "escenarios"):
        self.base_dir = os.path.abspath(base_dir)
        self._current_scenario: Optional[Dict] = None
        self._current_scenario_id: Optional[str] = None
        self._ensure_base_dir()

    def _ensure_base_dir(self):
        if not os.path.exists(self.base_dir):
            os.makedirs(self.base_dir)



    def create_scenario(self, scenario_id: str, nombre: str,
                        geofence: Dict, capas: Dict,
                        obstaculos: Dict) -> Dict:

        scenario = {
            "id": scenario_id,
            "nombre": nombre,
            "created": datetime.now().isoformat(),
            "modified": datetime.now().isoformat(),
            "geofence": geofence,
            "capas": capas,
            "obstaculos": obstaculos,
            "planes_vuelo": []
        }

        self._save_scenario(scenario)
        return scenario

    def save_scenario(self, scenario: Dict) -> bool:

        scenario["modified"] = datetime.now().isoformat()
        return self._save_scenario(scenario)

    def _save_scenario(self, scenario: Dict) -> bool:

        scenario_id = scenario.get("id")
        nombre = scenario.get("nombre", scenario_id)
        if not scenario_id:
            return False

        # Usar el nombre del escenario como nombre de archivo (sanitizado)
        filename = sanitize_filename(nombre)
        path = os.path.join(self.base_dir, f"{filename}.json")

        try:
            with open(path, "w", encoding="utf-8") as f:
                json.dump(scenario, f, indent=2, ensure_ascii=False)
            return True
        except Exception as e:
            print(f"[scenario] Error guardando: {e}")
            return False

    def load_scenario(self, scenario_id: str) -> Optional[Dict]:

        # Primero intentar cargar por ID (formato antiguo)
        path = os.path.join(self.base_dir, f"{scenario_id}.json")
        if not os.path.exists(path):
            # Buscar por nombre sanitizado
            path = os.path.join(self.base_dir, f"{sanitize_filename(scenario_id)}.json")
        if not os.path.exists(path):
            # Buscar en todos los archivos por ID interno
            for filename in os.listdir(self.base_dir):
                if filename.endswith(".json"):
                    filepath = os.path.join(self.base_dir, filename)
                    try:
                        with open(filepath, "r", encoding="utf-8") as f:
                            data = json.load(f)
                        if data.get("id") == scenario_id:
                            path = filepath
                            break
                    except Exception:
                        continue
            else:
                return None

        try:
            with open(path, "r", encoding="utf-8") as f:
                scenario = json.load(f)
            self._current_scenario = scenario
            self._current_scenario_id = scenario.get("id", scenario_id)
            return scenario
        except Exception as e:
            print(f"[scenario] Error cargando: {e}")
            return None

    def delete_scenario(self, scenario_id: str) -> bool:

        # Primero intentar por ID
        path = os.path.join(self.base_dir, f"{scenario_id}.json")
        if not os.path.exists(path):
            # Buscar por nombre sanitizado
            path = os.path.join(self.base_dir, f"{sanitize_filename(scenario_id)}.json")
        if not os.path.exists(path):
            # Buscar en todos los archivos por ID interno
            for filename in os.listdir(self.base_dir):
                if filename.endswith(".json"):
                    filepath = os.path.join(self.base_dir, filename)
                    try:
                        with open(filepath, "r", encoding="utf-8") as f:
                            data = json.load(f)
                        if data.get("id") == scenario_id:
                            path = filepath
                            break
                    except Exception:
                        continue

        if os.path.exists(path):
            try:
                os.remove(path)
                if self._current_scenario_id == scenario_id:
                    self._current_scenario = None
                    self._current_scenario_id = None
                return True
            except Exception as e:
                print(f"[scenario] Error eliminando: {e}")
        return False

    def list_scenarios(self) -> List[Dict]:

        scenarios = []
        if not os.path.exists(self.base_dir):
            return scenarios

        for filename in os.listdir(self.base_dir):
            if filename.endswith(".json"):
                path = os.path.join(self.base_dir, filename)
                try:
                    with open(path, "r", encoding="utf-8") as f:
                        data = json.load(f)
                    scenarios.append({
                        "id": data.get("id", filename[:-5]),
                        "nombre": data.get("nombre", "Sin nombre"),
                        "created": data.get("created"),
                        "modified": data.get("modified"),
                        "num_planes": len(data.get("planes_vuelo", []))
                    })
                except Exception:
                    pass

        return sorted(scenarios, key=lambda x: x.get("modified") or "", reverse=True)



    def add_flight_plan(self, scenario_id: str, plan_id: str, nombre: str,
                        waypoints: List[Dict]) -> bool:

        scenario = self.load_scenario(scenario_id)
        if not scenario:
            return False

        # Verificar que no exista un plan con el mismo ID
        for plan in scenario.get("planes_vuelo", []):
            if plan.get("id") == plan_id:
                # Actualizar existente
                plan["nombre"] = nombre
                plan["waypoints"] = waypoints
                plan["modified"] = datetime.now().isoformat()
                return self.save_scenario(scenario)

        # Crear nuevo
        plan = {
            "id": plan_id,
            "nombre": nombre,
            "created": datetime.now().isoformat(),
            "modified": datetime.now().isoformat(),
            "waypoints": waypoints
        }

        if "planes_vuelo" not in scenario:
            scenario["planes_vuelo"] = []
        scenario["planes_vuelo"].append(plan)

        return self.save_scenario(scenario)

    def get_flight_plan(self, scenario_id: str, plan_id: str) -> Optional[Dict]:

        scenario = self.load_scenario(scenario_id)
        if not scenario:
            return None

        for plan in scenario.get("planes_vuelo", []):
            if plan.get("id") == plan_id:
                return plan
        return None

    def delete_flight_plan(self, scenario_id: str, plan_id: str) -> bool:

        scenario = self.load_scenario(scenario_id)
        if not scenario:
            return False

        planes = scenario.get("planes_vuelo", [])
        scenario["planes_vuelo"] = [p for p in planes if p.get("id") != plan_id]

        return self.save_scenario(scenario)

    def list_flight_plans(self, scenario_id: str) -> List[Dict]:

        scenario = self.load_scenario(scenario_id)
        if not scenario:
            return []

        return [
            {
                "id": p.get("id"),
                "nombre": p.get("nombre"),
                "num_waypoints": len(p.get("waypoints", [])),
                "created": p.get("created"),
                "modified": p.get("modified")
            }
            for p in scenario.get("planes_vuelo", [])
        ]



    def get_current_scenario(self) -> Optional[Dict]:

        return self._current_scenario

    def get_current_scenario_id(self) -> Optional[str]:

        return self._current_scenario_id

    def set_current_scenario(self, scenario_id: str) -> bool:

        scenario = self.load_scenario(scenario_id)
        return scenario is not None

    def clear_current_scenario(self):

        self._current_scenario = None
        self._current_scenario_id = None



    def import_from_template(self, template_path: str, scenario_id: str,
                             nombre: str) -> Optional[Dict]:

        try:
            with open(template_path, "r", encoding="utf-8") as f:
                data = json.load(f)
        except Exception as e:
            print(f"[scenario] Error leyendo plantilla: {e}")
            return None

        # Detectar tipo de plantilla
        if data.get("type") == "mission":
            # Plantilla de misión
            geofence = data.get("geofence", {})
            obstaculos = {
                "circles": [e for e in data.get("exclusions", []) if e.get("type") == "circle"],
                "polygons": [e for e in data.get("exclusions", []) if e.get("type") in ("polygon", "poly", "rect")]
            }
            capas = {"c1_max": 60, "c2_max": 120, "c3_max": 200}

            # Crear escenario
            scenario = self.create_scenario(scenario_id, nombre, geofence, capas, obstaculos)

            # Añadir el plan de vuelo si hay waypoints
            if data.get("waypoints"):
                self.add_flight_plan(
                    scenario_id,
                    "plan_importado",
                    "Plan importado",
                    data["waypoints"]
                )

            return self.load_scenario(scenario_id)
        else:
            # Plantilla de geofence/mapa
            inclusion = data.get("inclusion", [0, 0, 100, 100])
            geofence = {
                "x1": inclusion[0] if len(inclusion) > 0 else 0,
                "y1": inclusion[1] if len(inclusion) > 1 else 0,
                "x2": inclusion[2] if len(inclusion) > 2 else 100,
                "y2": inclusion[3] if len(inclusion) > 3 else 100,
                "zmin": data.get("zmin", 0),
                "zmax": data.get("zmax", 200)
            }

            obstaculos = {
                "circles": data.get("circles", []),
                "polygons": data.get("polygons", [])
            }

            capas = data.get("layers", {"c1_max": 60, "c2_max": 120, "c3_max": 200})

            return self.create_scenario(scenario_id, nombre, geofence, capas, obstaculos)



_scenario_manager: Optional[ScenarioManager] = None


def get_scenario_manager(base_dir: str = "escenarios") -> ScenarioManager:

    global _scenario_manager
    if _scenario_manager is None:
        _scenario_manager = ScenarioManager(base_dir)
    return _scenario_manager