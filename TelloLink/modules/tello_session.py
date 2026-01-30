import os
import json
from datetime import datetime
from typing import Optional, Dict, List, Any


class SessionManager:


    def __init__(self, base_dir: str = "sesiones"):
        self.base_dir = os.path.abspath(base_dir)
        self._current_session: Optional[str] = None
        self._session_data: Dict[str, Any] = {}
        self._ensure_base_dir()

    def _ensure_base_dir(self):

        if not os.path.exists(self.base_dir):
            os.makedirs(self.base_dir)

    def start_session(self, escenario_id: Optional[str] = None,
                       plan_id: Optional[str] = None,
                       tipo: str = "manual") -> str:

        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        session_path = os.path.join(self.base_dir, timestamp)

        # Crear estructura de carpetas
        os.makedirs(os.path.join(session_path, "fotos"), exist_ok=True)
        os.makedirs(os.path.join(session_path, "videos"), exist_ok=True)

        # Inicializar datos de sesión
        self._current_session = timestamp
        self._session_data = {
            "start_time": datetime.now().isoformat(),
            "end_time": None,
            "escenario_id": escenario_id,
            "tipo": tipo,
            "plan_id": plan_id,
            "drone_connected": True,
            "takeoffs": 0,
            "total_flight_time_sec": 0,
            "photos_count": 0,
            "videos_count": 0
        }

        self._save_session_metadata()
        return timestamp

    def end_session(self):

        if self._current_session:
            self._session_data["end_time"] = datetime.now().isoformat()
            self._save_session_metadata()
            self._current_session = None
            self._session_data = {}

    def _save_session_metadata(self):

        if not self._current_session:
            return

        session_path = os.path.join(self.base_dir, self._current_session)

        # Asegurar que la carpeta de sesión existe
        if not os.path.exists(session_path):
            os.makedirs(os.path.join(session_path, "fotos"), exist_ok=True)
            os.makedirs(os.path.join(session_path, "videos"), exist_ok=True)

        json_path = os.path.join(session_path, "session.json")

        try:
            with open(json_path, "w", encoding="utf-8") as f:
                json.dump(self._session_data, f, indent=2, ensure_ascii=False)
        except Exception as e:
            print(f"[Session] Error guardando metadata: {e}")

    def get_photo_path(self, filename: Optional[str] = None) -> str:

        if not self._current_session:
            self.start_session()

        if filename is None:
            filename = f"shot_{datetime.now().strftime('%H%M%S_%f')[:-3]}.png"

        self._session_data["photos_count"] += 1
        self._save_session_metadata()

        return os.path.join(self.base_dir, self._current_session, "fotos", filename)

    def get_video_path(self, filename: Optional[str] = None) -> str:

        if not self._current_session:
            self.start_session()

        if filename is None:
            filename = f"rec_{datetime.now().strftime('%H%M%S_%f')[:-3]}.avi"

        self._session_data["videos_count"] += 1
        self._save_session_metadata()

        return os.path.join(self.base_dir, self._current_session, "videos", filename)

    def increment_takeoffs(self):

        if self._current_session:
            self._session_data["takeoffs"] += 1
            self._save_session_metadata()

    def update_flight_time(self, seconds: int):

        if self._current_session:
            self._session_data["total_flight_time_sec"] = seconds
            self._save_session_metadata()

    def get_current_session_id(self) -> Optional[str]:

        return self._current_session

    def is_session_active(self) -> bool:

        return self._current_session is not None



    def list_sessions(self) -> List[Dict[str, Any]]:

        sessions = []

        if not os.path.exists(self.base_dir):
            return sessions

        for folder in os.listdir(self.base_dir):
            session_path = os.path.join(self.base_dir, folder)
            if not os.path.isdir(session_path):
                continue

            # Leer metadatos si existen
            json_path = os.path.join(session_path, "session.json")
            if os.path.exists(json_path):
                try:
                    with open(json_path, "r", encoding="utf-8") as f:
                        metadata = json.load(f)
                except (json.JSONDecodeError, IOError):
                    metadata = {}
            else:
                metadata = {}

            # Contar archivos si no hay metadatos
            fotos_dir = os.path.join(session_path, "fotos")
            videos_dir = os.path.join(session_path, "videos")

            photos_count = len([f for f in os.listdir(fotos_dir) if f.lower().endswith(('.png', '.jpg', '.jpeg'))]) if os.path.exists(fotos_dir) else 0
            videos_count = len([f for f in os.listdir(videos_dir) if f.lower().endswith(('.mp4', '.avi', '.mov'))]) if os.path.exists(videos_dir) else 0

            sessions.append({
                "id": folder,
                "path": session_path,
                "start_time": metadata.get("start_time", folder),
                "end_time": metadata.get("end_time"),
                "escenario_id": metadata.get("escenario_id"),
                "tipo": metadata.get("tipo", "manual"),
                "plan_id": metadata.get("plan_id"),
                "takeoffs": metadata.get("takeoffs", 0),
                "photos_count": photos_count,
                "videos_count": videos_count,
                "total_flight_time_sec": metadata.get("total_flight_time_sec", 0)
            })

        # Ordenar por ID (fecha) descendente
        sessions.sort(key=lambda x: x["id"], reverse=True)
        return sessions

    def get_session_media(self, session_id: str, media_type: str = "all") -> List[Dict[str, Any]]:

        session_path = os.path.join(self.base_dir, session_id)
        media_list = []

        if not os.path.exists(session_path):
            return media_list

        # Fotos
        if media_type in ("all", "photos"):
            fotos_dir = os.path.join(session_path, "fotos")
            if os.path.exists(fotos_dir):
                for filename in os.listdir(fotos_dir):
                    if filename.lower().endswith(('.png', '.jpg', '.jpeg')):
                        filepath = os.path.join(fotos_dir, filename)
                        stat = os.stat(filepath)
                        media_list.append({
                            "type": "photo",
                            "filename": filename,
                            "path": filepath,
                            "size_bytes": stat.st_size,
                            "modified_time": datetime.fromtimestamp(stat.st_mtime).isoformat()
                        })

        # Vídeos
        if media_type in ("all", "videos"):
            videos_dir = os.path.join(session_path, "videos")
            if os.path.exists(videos_dir):
                for filename in os.listdir(videos_dir):
                    if filename.lower().endswith(('.mp4', '.avi', '.mov')):
                        filepath = os.path.join(videos_dir, filename)
                        stat = os.stat(filepath)
                        media_list.append({
                            "type": "video",
                            "filename": filename,
                            "path": filepath,
                            "size_bytes": stat.st_size,
                            "modified_time": datetime.fromtimestamp(stat.st_mtime).isoformat()
                        })

        # Ordenar por tiempo de modificación (más reciente primero)
        media_list.sort(key=lambda x: x["modified_time"], reverse=True)
        return media_list

    def delete_media(self, filepath: str) -> bool:

        try:
            if os.path.exists(filepath):
                os.remove(filepath)
                return True
        except OSError:
            pass
        return False

    def delete_session(self, session_id: str) -> bool:

        import shutil
        session_path = os.path.join(self.base_dir, session_id)
        try:
            if os.path.exists(session_path):
                shutil.rmtree(session_path)
                return True
        except OSError:
            pass
        return False



    def list_sessions_by_scenario(self, escenario_id: str) -> List[Dict[str, Any]]:

        all_sessions = self.list_sessions()
        return [s for s in all_sessions if s.get("escenario_id") == escenario_id]

    def list_sessions_by_plan(self, escenario_id: str, plan_id: str) -> List[Dict[str, Any]]:

        all_sessions = self.list_sessions()
        return [
            s for s in all_sessions
            if s.get("escenario_id") == escenario_id and s.get("plan_id") == plan_id
        ]

    def list_manual_sessions(self, escenario_id: str) -> List[Dict[str, Any]]:

        all_sessions = self.list_sessions()
        return [
            s for s in all_sessions
            if s.get("escenario_id") == escenario_id and s.get("tipo") == "manual"
        ]

    def get_scenario_stats(self, escenario_id: str) -> Dict[str, Any]:

        sessions = self.list_sessions_by_scenario(escenario_id)

        total_photos = sum(s.get("photos_count", 0) for s in sessions)
        total_videos = sum(s.get("videos_count", 0) for s in sessions)
        total_flights = len(sessions)
        plan_sessions = [s for s in sessions if s.get("tipo") == "plan"]
        manual_sessions = [s for s in sessions if s.get("tipo") == "manual"]

        return {
            "escenario_id": escenario_id,
            "total_sessions": total_flights,
            "plan_sessions": len(plan_sessions),
            "manual_sessions": len(manual_sessions),
            "total_photos": total_photos,
            "total_videos": total_videos
        }




def migrate_legacy_files(captures_dir: str, videos_dir: str, sessions_dir: str) -> bool:

    import shutil

    has_captures = os.path.exists(captures_dir) and any(
        f.lower().endswith(('.png', '.jpg', '.jpeg')) for f in os.listdir(captures_dir)
    ) if os.path.exists(captures_dir) else False

    has_videos = os.path.exists(videos_dir) and any(
        f.lower().endswith(('.mp4', '.avi', '.mov')) for f in os.listdir(videos_dir)
    ) if os.path.exists(videos_dir) else False

    if not has_captures and not has_videos:
        return False

    # Crear sesión legacy
    legacy_path = os.path.join(sessions_dir, "legacy")
    os.makedirs(os.path.join(legacy_path, "fotos"), exist_ok=True)
    os.makedirs(os.path.join(legacy_path, "videos"), exist_ok=True)

    photos_migrated = 0
    videos_migrated = 0

    # Migrar fotos
    if has_captures:
        for filename in os.listdir(captures_dir):
            if filename.lower().endswith(('.png', '.jpg', '.jpeg')):
                src = os.path.join(captures_dir, filename)
                dst = os.path.join(legacy_path, "fotos", filename)
                shutil.move(src, dst)
                photos_migrated += 1

    # Migrar vídeos
    if has_videos:
        for filename in os.listdir(videos_dir):
            if filename.lower().endswith(('.mp4', '.avi', '.mov')):
                src = os.path.join(videos_dir, filename)
                dst = os.path.join(legacy_path, "videos", filename)
                shutil.move(src, dst)
                videos_migrated += 1

    # Crear metadatos
    metadata = {
        "start_time": "legacy",
        "end_time": None,
        "drone_connected": False,
        "takeoffs": 0,
        "total_flight_time_sec": 0,
        "photos_count": photos_migrated,
        "videos_count": videos_migrated,
        "note": "Archivos migrados desde captures/ y videos/"
    }

    with open(os.path.join(legacy_path, "session.json"), "w", encoding="utf-8") as f:
        json.dump(metadata, f, indent=2, ensure_ascii=False)

    return True




def start_flight_session(self) -> str:

    if not hasattr(self, '_session_manager'):
        from TelloLink.modules.tello_session import SessionManager
        self._session_manager = SessionManager()
    return self._session_manager.start_session()


def end_flight_session(self):

    if hasattr(self, '_session_manager'):
        self._session_manager.end_session()


def get_session_photo_path(self, filename: Optional[str] = None) -> str:

    if not hasattr(self, '_session_manager'):
        from TelloLink.modules.tello_session import SessionManager
        self._session_manager = SessionManager()
    return self._session_manager.get_photo_path(filename)


def get_session_video_path(self, filename: Optional[str] = None) -> str:

    if not hasattr(self, '_session_manager'):
        from TelloLink.modules.tello_session import SessionManager
        self._session_manager = SessionManager()
    return self._session_manager.get_video_path(filename)


def get_session_manager(self) -> SessionManager:

    if not hasattr(self, '_session_manager'):
        from TelloLink.modules.tello_session import SessionManager
        self._session_manager = SessionManager()
    return self._session_manager