#!/usr/bin/env python3
"""
ROS2 Action Server zur Erfassung aller 6 Rubik-Seiten und Berechnung der Lösung.
Action: cube_solver.action.SolveCube  (aus action/SolveCube.action)
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile

import cv2
import numpy as np
import time
from typing import List, Tuple

try:
    from cube_solver.action import SolveCube
except Exception as e:
    raise RuntimeError(
        "Konnte Action-Type cube_solver.action.SolveCube nicht importieren. "
        "Stelle sicher, dass du die Action 'SolveCube.action' im Paket hattest und das Paket gebaut wurde."
    )

# (Optional) Typ für Drive action des Roboters, als Platzhalter:
# from robot_interfaces.action import DriveFace
# Wir verwenden hier keinen konkreten Typ, machen nur ein optionales Client-Call-Pattern.

FACE_ORDER = ["U", "R", "F", "D", "L", "B"]
# Farbgrenzen (HSV)
COLOR_RANGES = {
    "white": ((0, 0, 180), (180, 60, 255)),
    "yellow": ((20, 100, 100), (35, 255, 255)),
    "red": ((0, 120, 70), (10, 255, 255)),
    "red2": ((170, 120, 70), (180, 255, 255)),
    "orange": ((10, 120, 100), (20, 255, 255)),
    "green": ((40, 70, 70), (85, 255, 255)),
    "blue": ((90, 70, 70), (130, 255, 255)),
}

MOVE_DESCRIPTIONS = {
    "U": "Oben im Uhrzeigersinn",
    "U'": "Oben gegen den Uhrzeigersinn",
    "U2": "Oben doppelt",
    "R": "Rechts im Uhrzeigersinn",
    "R'": "Rechts gegen den Uhrzeigersinn",
    "R2": "Rechts doppelt",
    "F": "Vorne im Uhrzeigersinn",
    "F'": "Vorne gegen den Uhrzeigersinn",
    "F2": "Vorne doppelt",
    "D": "Unten im Uhrzeigersinn",
    "D'": "Unten gegen den Uhrzeigersinn",
    "D2": "Unten doppelt",
    "L": "Links im Uhrzeigersinn",
    "L'": "Links gegen den Uhrzeigersinn",
    "L2": "Links doppelt",
    "B": "Hinten im Uhrzeigersinn",
    "B'": "Hinten gegen den Uhrzeigersinn",
    "B2": "Hinten doppelt",
}

def describe_solution(solution: str) -> str:
    if not solution:
        return "Keine Züge erforderlich."
    parts = []
    for move in solution.split():
        parts.append(MOVE_DESCRIPTIONS.get(move, move))
    return " · ".join(parts)


def classify_color(hsv_value: np.ndarray) -> str:
    """
    Klassifiziert anhand der COLOR_RANGES. hsv_value ist (H,S,V).
    Gibt einen Namen zurück oder 'unknown'.
    """
    h, s, v = int(hsv_value[0]), int(hsv_value[1]), int(hsv_value[2])
    for name, (lower, upper) in COLOR_RANGES.items():
        low = np.array(lower, dtype=np.uint8)
        up = np.array(upper, dtype=np.uint8)
        # cv2.inRange erwartet ein Bild, hier bauen wir eine 1x1 px
        px = np.uint8([[[h, s, v]]])
        mask = cv2.inRange(px, low, up)
        if mask[0, 0] != 0:
            return "red" if name == "red2" else name
    return "unknown"


class SolveCubeActionServer(Node):
    def __init__(self):
        super().__init__("solve_cube_action_server")
        self._action_server = ActionServer(
            self,
            SolveCube,
            'solve_cube',
            execute_callback=self.execute_callback
        )
        self.get_logger().info("SolveCube Action Server gestartet.")
        # Optionaler ActionClient-Name/Typ für Fahren (Platzhalter)
        # self.drive_client = ActionClient(self, DriveFace, 'drive_to_face')
        # Wir verwenden keine Typen, nur optionales Warten auf Server.
        self.camera = None

    def execute_callback(self, goal_handle):
        self.get_logger().info("Goal erhalten: Starte Erfassung aller Seiten...")
        feedback_msg = SolveCube.Feedback()
        result = SolveCube.Result()

        # Parameter aus Goal lesen
        camera_index = 0
        try:
            camera_index = int(goal_handle.request.camera_index)
        except Exception:
            camera_index = 0

        # Versuche Kamera zu öffnen
        cap = cv2.VideoCapture(camera_index)
        if not cap.isOpened():
            msg = f"Kamera index={camera_index} nicht verfügbar."
            self.get_logger().error(msg)
            result.success = False
            result.message = msg
            goal_handle.abort()
            return result

        captured_faces = {}  # face -> list[str] 9 Farben
        color_to_face = {}   # center_color -> face

        total_faces = len(FACE_ORDER)
        faces_captured = 0

        try:
            for face in FACE_ORDER:
                # 1) Feedback: wir starten diese Seite
                feedback_msg.current_face = face
                feedback_msg.faces_captured = faces_captured
                feedback_msg.total_faces = total_faces
                feedback_msg.status = f"Anfordern: Roboter zeigt Seite {face}."
                goal_handle.publish_feedback(feedback_msg)
                self.get_logger().info(f"Fordere Roboter auf, Seite {face} zu zeigen (platzhalter).")

                # --- Platzhalter: Action an Roboter senden, damit richtige Seite gezeigt wird ---
                # Wenn ein echter ActionServer existiert, hier einen ActionClient benutzen, z.B.:
                #   drive_goal = DriveFace.Goal()
                #   drive_goal.face = face
                #   self.drive_client.wait_for_server(timeout_sec=2.0)
                #   send_goal_future = self.drive_client.send_goal_async(drive_goal)
                #   ...
                # Wir implementieren hier: warte kurz und fahre dann weiter.
                # Wichtig: Nicht blockierend lange warten in einem Action-Server (hier kurz und robust).
                placeholder_wait_s = 1.0
                self.get_logger().info(f"(Platzhalter) Warte {placeholder_wait_s}s, damit Roboter sich dreht.")
                time.sleep(placeholder_wait_s)

                # 2) Erfasse die Seite per Kamera (mitteln über mehrere Frames)
                feedback_msg.status = f"Erfasse Seite {face} per Kamera..."
                goal_handle.publish_feedback(feedback_msg)
                colors = self.capture_face_colors(cap, sample_frames=6)
                if colors is None or len(colors) != 9:
                    msg = f"Fehler beim Erfassen der Seite {face}."
                    self.get_logger().warn(msg)
                    # wir fahren trotzdem fort — aber markieren result
                    result.success = False
                    result.message = msg
                    # optional: abbrechen oder weitermachen? Wir machen weiter, aber merken Fehler.
                else:
                    # Center-Farbe der Seite = colors[4]
                    center = colors[4]
                    # Wenn center noch unbekannt, ordne diese Seite zu
                    color_to_face[center] = face
                    captured_faces[face] = colors
                    faces_captured += 1
                    feedback_msg.faces_captured = faces_captured
                    feedback_msg.status = f"Seite {face} gespeichert ({faces_captured}/{total_faces})."
                    goal_handle.publish_feedback(feedback_msg)
                    self.get_logger().info(f"Seite {face} gespeichert: {colors}")

            # Ende Schleife über alle Seiten
            # Prüfe Vollständigkeit
            if len(captured_faces) != 6:
                msg = f"Nur {len(captured_faces)}/6 Seiten erfasst."
                self.get_logger().warn(msg)
                result.success = False
                result.message = msg
                # Wir geben dennoch das Ergebnis zurück mit Fehlerhinweis.
            else:
                # Baue facelet_string für kociemba
                facelet_order = []
                for face in FACE_ORDER:
                    colors = captured_faces[face]
                    if len(colors) != 9:
                        msg = f"Seite {face} unvollständig."
                        self.get_logger().error(msg)
                        result.success = False
                        result.message = msg
                        break
                    # map colors to face letters via center mapping color_to_face
                    for c in colors:
                        mapped = color_to_face.get(c, "?")
                        if mapped == "?":
                            # unknown color — unsichere Zuordnung
                            pass
                        facelet_order.append(mapped)
                # Wenn Fragezeichen in facelet_order -> Fehler
                if "?" in facelet_order or len(facelet_order) != 54:
                    msg = "Unvollständige oder unklare Farberkennung: facelet string ungültig."
                    self.get_logger().error(msg)
                    result.success = False
                    result.message = msg
                else:
                    facelet_string = "".join(facelet_order)
                    self.get_logger().info(f"Facelet string: {facelet_string}")
                    # Versuche kociemba zu verwenden
                    try:
                        import kociemba
                        solution = kociemba.solve(facelet_string)
                        description = describe_solution(solution)
                        result.solution = solution
                        result.description = description
                        result.success = True
                        result.message = "Erfolgreich gelöst."
                        self.get_logger().info(f"Solver Ergebnis: {solution}")
                    except ImportError:
                        msg = "kociemba nicht installiert (pip install kociemba)."
                        self.get_logger().error(msg)
                        result.success = False
                        result.message = msg
                    except Exception as e:
                        msg = f"Solver-Fehler: {e}"
                        self.get_logger().error(msg)
                        result.success = False
                        result.message = msg

            # Abschließendes Feedback
            feedback_msg.status = "Fertig."
            goal_handle.publish_feedback(feedback_msg)
            # Sende Result und setze Goal auf succeeded (auch bei Fehlern, so dass der Client das Ergebnis lesen kann)
            goal_handle.succeed()
            return result

        finally:
            cap.release()

    def capture_face_colors(self, cap: cv2.VideoCapture, sample_frames: int = 6) -> List[str]:
        """
        Erfasse die 9 Facelet-Farben für die aktuell sichtbare Seite.
        Wir nehmen mehrere Frames, extrahieren für jeden Frame die 3x3-Zellen,
        mitteln den HSV und klassifizieren die Farbe.
        """
        collected = []  # wird Liste von lists (frames x 9) sein
        attempts = 0
        max_attempts = sample_frames * 2
        while len(collected) < sample_frames and attempts < max_attempts:
            ret, frame = cap.read()
            attempts += 1
            if not ret:
                time.sleep(0.05)
                continue
            h, w, _ = frame.shape
            face_size = min(h, w) // 2
            x0, y0 = (w - face_size) // 2, (h - face_size) // 2
            cell = face_size // 3
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            colors = []
            ok = True
            for row in range(3):
                for col in range(3):
                    cx = x0 + col * cell + cell // 2
                    cy = y0 + row * cell + cell // 2
                    # clamp coords
                    x1 = max(cx - 5, 0); x2 = min(cx + 5, w-1)
                    y1 = max(cy - 5, 0); y2 = min(cy + 5, h-1)
                    sample = hsv[y1:y2+1, x1:x2+1]
                    if sample.size == 0:
                        ok = False
                        break
                    mean = sample.reshape(-1, 3).mean(axis=0)
                    color_name = classify_color(mean.astype(np.uint8))
                    colors.append(color_name)
                if not ok:
                    break
            if ok and len(colors) == 9:
                collected.append(colors)
            else:
                time.sleep(0.05)

        if not collected:
            return None

        # Nun mitteln: für jedes facelet wähle die am häufigsten vorkommende Klassifikation über Frames
        final = []
        for idx in range(9):
            votes = {}
            for frame_colors in collected:
                c = frame_colors[idx]
                votes[c] = votes.get(c, 0) + 1
            # Wähle größtes Vote
            best = max(votes.items(), key=lambda kv: kv[1])[0]
            final.append(best)
        return final


# def main(args=None):
#     rclpy.init(args=args)
#     try:
#         server = SolveCubeActionServer()
#         try:
#             rclpy.spin(server)
#         except KeyboardInterrupt:
#             server.get_logger().info("KeyboardInterrupt, herunterfahren...")
#         finally:
#             server.destroy()
#     finally:
#         rclpy.shutdown()


def main(args=None):

    rclpy.init(args=args)
    
    try:
        server = SolveCubeActionServer()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(server)
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"FATAL ERROR: {e}")
        import traceback
        traceback.print_exc()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
