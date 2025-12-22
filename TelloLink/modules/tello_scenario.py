# tello_scenario.py - Gestión de escenarios (salas de museo)
# Un escenario contiene: geofence, capas, obstáculos y planes de vuelo

import os
import json
from datetime import datetime
from typing import Optional, Dict, List, Any


class ScenarioManager:
    """Gestiona escenarios (salas) con sus obstáculos y planes de vuelo."""

    def __init__(self, base_dir: str = "escenarios"):
        self.base_dir = os.path.abspath(base_dir)
        self._current_scenario: Optional[Dict] = None
        self._current_scenario_id: Optional[str] = None
        self._ensure_base_dir()

    def _ensure_base_dir(self):
        if not os.path.exists(self.base_dir):
            os.makedirs(self.base_dir)

    # =========================================================================
    # CRUD de Escenarios
    # =========================================================================

    def create_scenario(self, scenario_id: str, nombre: str,
                        geofence: Dict, capas: Dict,
                        obstaculos: Dict) -> Dict:
        """
        Crea un nuevo escenario.

        Args:
            scenario_id: ID único del escenario (ej: "sala_picasso")
            nombre: Nombre descriptivo (ej: "Sala Picasso")
            geofence: {"x1", "y1", "x2", "y2", "zmin", "zmax"}
            capas: {"c1_max", "c2_max", "c3_max"}
            obstaculos: {"circles": [...], "polygons": [...]}

        Returns:
            El escenario creado
        """
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
        """Guarda un escenario existente."""
        scenario["modified"] = datetime.now().isoformat()
        return self._save_scenario(scenario)

    def _save_scenario(self, scenario: Dict) -> bool:
        """Guarda el escenario a disco."""
        scenario_id = scenario.get("id")
        if not scenario_id:
            return False

        path = os.path.join(self.base_dir, f"{scenario_id}.json")
        try:
            with open(path, "w", encoding="utf-8") as f:
                json.dump(scenario, f, indent=2, ensure_ascii=False)
            return True
        except Exception as e:
            print(f"[scenario] Error guardando: {e}")
            return False

    def load_scenario(self, scenario_id: str) -> Optional[Dict]:
        """Carga un escenario por su ID."""
        path = os.path.join(self.base_dir, f"{scenario_id}.json")
        if not os.path.exists(path):
            return None

        try:
            with open(path, "r", encoding="utf-8") as f:
                scenario = json.load(f)
            self._current_scenario = scenario
            self._current_scenario_id = scenario_id
            return scenario
        except Exception as e:
            print(f"[scenario] Error cargando: {e}")
            return None

    def delete_scenario(self, scenario_id: str) -> bool:
        """Elimina un escenario."""
        path = os.path.join(self.base_dir, f"{scenario_id}.json")
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
        """Lista todos los escenarios disponibles."""
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

        return sorted(scenarios, key=lambda x: x.get("modified", ""), reverse=True)

    # =========================================================================
    # Gestión de Planes de Vuelo dentro de un Escenario
    # =========================================================================

    def add_flight_plan(self, scenario_id: str, plan_id: str, nombre: str,
                        waypoints: List[Dict], return_home: bool = False) -> bool:
        """
        Añade un plan de vuelo a un escenario.

        Args:
            scenario_id: ID del escenario
            plan_id: ID único del plan (ej: "recorrido_cuadros")
            nombre: Nombre descriptivo
            waypoints: Lista de waypoints con acciones
            return_home: Si debe volver al origen al final
        """
        scenario = self.load_scenario(scenario_id)
        if not scenario:
            return False

        # Verificar que no exista un plan con el mismo ID
        for plan in scenario.get("planes_vuelo", []):
            if plan.get("id") == plan_id:
                # Actualizar existente
                plan["nombre"] = nombre
                plan["waypoints"] = waypoints
                plan["return_home"] = return_home
                plan["modified"] = datetime.now().isoformat()
                return self.save_scenario(scenario)

        # Crear nuevo
        plan = {
            "id": plan_id,
            "nombre": nombre,
            "created": datetime.now().isoformat(),
            "modified": datetime.now().isoformat(),
            "waypoints": waypoints,
            "return_home": return_home
        }

        if "planes_vuelo" not in scenario:
            scenario["planes_vuelo"] = []
        scenario["planes_vuelo"].append(plan)

        return self.save_scenario(scenario)

    def get_flight_plan(self, scenario_id: str, plan_id: str) -> Optional[Dict]:
        """Obtiene un plan de vuelo específico."""
        scenario = self.load_scenario(scenario_id)
        if not scenario:
            return None

        for plan in scenario.get("planes_vuelo", []):
            if plan.get("id") == plan_id:
                return plan
        return None

    def delete_flight_plan(self, scenario_id: str, plan_id: str) -> bool:
        """Elimina un plan de vuelo de un escenario."""
        scenario = self.load_scenario(scenario_id)
        if not scenario:
            return False

        planes = scenario.get("planes_vuelo", [])
        scenario["planes_vuelo"] = [p for p in planes if p.get("id") != plan_id]

        return self.save_scenario(scenario)

    def list_flight_plans(self, scenario_id: str) -> List[Dict]:
        """Lista los planes de vuelo de un escenario."""
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

    # =========================================================================
    # Escenario actual
    # =========================================================================

    def get_current_scenario(self) -> Optional[Dict]:
        """Retorna el escenario actualmente cargado."""
        return self._current_scenario

    def get_current_scenario_id(self) -> Optional[str]:
        """Retorna el ID del escenario actualmente cargado."""
        return self._current_scenario_id

    def set_current_scenario(self, scenario_id: str) -> bool:
        """Establece el escenario actual."""
        scenario = self.load_scenario(scenario_id)
        return scenario is not None

    def clear_current_scenario(self):
        """Limpia el escenario actual."""
        self._current_scenario = None
        self._current_scenario_id = None

    # =========================================================================
    # Conversión desde plantillas existentes
    # =========================================================================

    def import_from_template(self, template_path: str, scenario_id: str,
                             nombre: str) -> Optional[Dict]:
        """
        Importa una plantilla existente como escenario.

        Args:
            template_path: Ruta al archivo JSON de la plantilla
            scenario_id: ID para el nuevo escenario
            nombre: Nombre del escenario
        """
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
                    data["waypoints"],
                    return_home=False
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


# Instancia global (singleton)
_scenario_manager: Optional[ScenarioManager] = None


def get_scenario_manager(base_dir: str = "escenarios") -> ScenarioManager:
    """Obtiene la instancia del gestor de escenarios."""
    global _scenario_manager
    if _scenario_manager is None:
        _scenario_manager = ScenarioManager(base_dir)
    return _scenario_manager