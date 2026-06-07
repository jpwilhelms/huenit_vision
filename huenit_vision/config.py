"""
huenit_vision/config.py
Zentrale Konfigurationskonstanten für den Huenit-Roboterarm und Kamera-Feedback.
"""

import os
import json
from dataclasses import dataclass, asdict
from typing import Self

@dataclass(frozen=True)
class RobotConfig:
    # --- 1. Serielle Schnittstelle ---
    baudrate: int = 115200
    serial_timeout: float = 2.0
    gcode_response_timeout: float = 15.0

    # --- 2. Kameras (Device-IDs / Pfade) ---
    overhead_camera_id: int | str = 2  # Deckenkamera (Index oder V4L2-Device-Pfad)
    hand_camera_id: int | str = 6      # Handkamera (Index oder V4L2-Device-Pfad)

    # --- 3. Kamera Auflösungen ---
    overhead_width: int = 1280
    overhead_height: int = 720
    hand_width: int = 640
    hand_height: int = 480

    # --- 4. Kamera-Tuning & Puffer ---
    camera_buffer_size: int = 1
    frame_flush_count: int = 8
    camera_settle_time: float = 0.5    # Beruhigungszeit nach Fahrt vor Bildaufnahme (s)
    camera_warmup_frames: int = 15     # Anzahl verworfener Initialisierungsframes

    # --- 5. Mechanische Z-Offsets (relativ zu Z_start) ---
    # *Empirische Parameter (Änderungen erfordern physische Verifikation)*
    z_place_offset: float = 1.0        # +1.0mm: Freigabe aus 1mm Fallhöhe
    z_pick_offset: float = -4.5        # -4.5mm: Kompression für zuverlässigen Pick
    z_travel_offset: float = 40.0      # +40.0mm: Sichere Reisehöhe
    z_verify: float = 77.3             # Absolute Z-Höhe für Handkamera-Detektion

    # --- 6. Parkposition ---
    # *Empirische Parameter*
    park_x: float = -150.0             # Arm aus dem Sichtfeld der Deckenkamera
    park_y: float = 150.0

    # --- 7. Vakuum & Ventil-Timing ---
    # *Empirische Parameter*
    vacuum_buildup_time: float = 0.5   # Sekunden bis Vakuum voll aufgebaut
    vacuum_release_time: float = 1.5   # Sekunden bis Vakuum vollständig abgebaut

    # --- 8. Hardware-Leistungskonstanten ---
    pump_max_power: int = 1023         # Maximaler Pumpenstrom (M1400 A1023)
    pump_hold_power: int = 623         # Halte-Pumpenstrom für Greifer-Operationen

    # --- 9. Bewegungsgeschwindigkeiten (mm/min) ---
    feed_travel: int = 8000            # Schnellfahrt horizontal
    feed_vertical: int = 4000          # Vertikalbewegung (Pick/Place)
    feed_servoing: int = 5000          # Feinjustierung mit Handkamera

    # --- 10. Visual Servoing Parameter ---
    # *Empirische Parameter*
    servoing_gain_x: float = -0.12     # mm/Pixel bei Z=77.3 (Kamera-X invertiert zu Roboter-X)
    servoing_gain_y: float = 0.12      # mm/Pixel bei Z=77.3
    servoing_tolerance_px: float = 4.0 # Pixel-Toleranz für Konvergenz
    servoing_max_steps: int = 5        # Maximale Korrekturschritte
    servoing_step_limit_mm: float = 5.0 # Maximaler Korrekturschritt pro Iteration (Sicherheitsgrenze)

    # --- 11. Positions-Verifikation ---
    position_tolerance_mm: float = 1.0 # Maximale Soll-Ist-Abweichung

    # --- 12. Pick-Versuche ---
    max_pick_retries: int = 3

    # --- 13. Kalibrierungs-Gitter ---
    grid_step_mm: float = 30.0         # BFS-Gitter-Schrittweite (3cm)

    # ==========================================
    # Helper Methods
    # ==========================================

    def get_z_place(self, z_start: float) -> float:
        """Berechnet die Z-Höhe für das Ablegen basierend auf der Objekthöhe."""
        return z_start + self.z_place_offset

    def get_z_pick(self, z_start: float) -> float:
        """Berechnet die Z-Höhe für das Aufnehmen basierend auf der Objekthöhe."""
        return z_start + self.z_pick_offset

    def get_z_travel(self, z_start: float) -> float:
        """Berechnet die Z-Sicherheitshöhe basierend auf der Objekthöhe."""
        return z_start + self.z_travel_offset

    def get_servoing_gains(self, z_current: float) -> tuple[float, float]:
        """
        Berechnet die Visual-Servoing-Gains proportional zur aktuellen Höhe.
        Gains ändern sich proportional zur Z-Höhe relativ zu z_verify.
        """
        if self.z_verify == 0:
            return self.servoing_gain_x, self.servoing_gain_y
        scale = z_current / self.z_verify
        return self.servoing_gain_x * scale, self.servoing_gain_y * scale

    # ==========================================
    # Serialization Methods
    # ==========================================

    def to_dict(self) -> dict:
        """Konvertiert die Konfiguration in ein Standard-Dictionary."""
        return asdict(self)

    @classmethod
    def from_dict(cls, data: dict) -> Self:
        """Erstellt eine Konfiguration aus einem Dictionary mit Typkonvertierungen."""
        import typing
        import types

        valid_keys = cls.__dataclass_fields__.keys()
        coerced_data = {}
        for k, v in data.items():
            if k not in valid_keys:
                continue

            field = cls.__dataclass_fields__[k]
            target_type = field.type

            # Find origin and args to handle Union types
            origin = typing.get_origin(target_type)
            if origin is typing.Union or (hasattr(types, "UnionType") and origin is types.UnionType):
                args = typing.get_args(target_type)
            elif hasattr(types, "UnionType") and isinstance(target_type, types.UnionType):
                args = typing.get_args(target_type)
            else:
                args = (target_type,)

            # Coerce the value to one of the target types
            coerced_val = v
            coerced = False
            for t in args:
                if t is float:
                    try:
                        coerced_val = float(v)
                        coerced = True
                        break
                    except (ValueError, TypeError):
                        pass
                elif t is int:
                    try:
                        if isinstance(v, str):
                            try:
                                coerced_val = int(v)
                            except ValueError:
                                coerced_val = int(float(v))
                        else:
                            coerced_val = int(v)
                        coerced = True
                        break
                    except (ValueError, TypeError):
                        pass
                elif t is str:
                    try:
                        coerced_val = str(v)
                        coerced = True
                        break
                    except (ValueError, TypeError):
                        pass
                elif t is type(None):
                    if v is None:
                        coerced_val = None
                        coerced = True
                        break

            if coerced:
                coerced_data[k] = coerced_val
            else:
                coerced_data[k] = v

        return cls(**coerced_data)

    def save_to_json(self, filepath: str) -> None:
        """Speichert die Konfiguration in eine JSON-Datei."""
        os.makedirs(os.path.dirname(os.path.abspath(filepath)), exist_ok=True)
        with open(filepath, 'w', encoding='utf-8') as f:
            json.dump(self.to_dict(), f, indent=4)

    @classmethod
    def load_from_json(cls, filepath: str) -> Self:
        """Lädt die Konfiguration aus einer JSON-Datei."""
        if not os.path.exists(filepath):
            raise FileNotFoundError(f"Konfigurationsdatei nicht gefunden: {filepath}")
        with open(filepath, 'r', encoding='utf-8') as f:
            data = json.load(f)
        return cls.from_dict(data)
