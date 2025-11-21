#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient

from cube_motion.action import RotateFace, HandOver

# Roboter 1 hält U, F, D; Roboter 2 hält L, B, R
ROBOT_FACES = ["U", "F", "D"], ["L", "B", "R"]

def decode_kociemba_move(move):
    """
    Decodes a single Kociemba move like 'U', 'R2', "F'", etc.

    Returns:
        face (str): One of U, D, L, R, F, B
        angle_deg (int): +90, -90, or 180
    """

    if len(move) == 0:
        raise ValueError("Move string is empty")

    # First character is always the face
    face = move[0]

    if face not in ["U", "D", "L", "R", "F", "B"]:
        raise ValueError(f"Invalid face in move: {move}")

    # Default angle: 90°
    angle_deg = 90

    # Check for modifier (', 2, nothing)
    if len(move) > 1:
        modifier = move[1]

        if modifier == "'":
            angle_deg = -90
        elif modifier == "2":
            angle_deg = 180
        else:
            raise ValueError(f"Invalid move modifier: {modifier}")

    return face, angle_deg


class CubeMotionServer(Node):

    def __init__(self):
        super().__init__('cube_motion_server')

        # Roboter, der den Würfel hält (1 oder 2)
        self.cube_robot = 1     

        # ActionServer: Drehen einer Würfelseite
        self.rotate_server = ActionServer(
            self,
            RotateFace,
            'rotate_face',
            self.execute_rotate_face
        )

        # ActionServer: Griff wechseln
        self.hand_over_server = ActionServer(
            self,
            HandOver,
            'hand_over',
            self.execute_hand_over
        )

        # interner Client: RotateFace kann HandOver aufrufen
        self.hand_over_client = ActionClient(self, HandOver, 'hand_over')

        self.get_logger().info("Cube Motion Server gestartet.")


    # -------------------------------
    # HAND OVER ACTION
    # -------------------------------
    async def execute_hand_over(self, goal_handle):
        targeted_cube_robot = goal_handle.request.cube_robot

        # Wenn kein Roboter angegeben wurde, zum anderen wechseln
        if (targeted_cube_robot == 0):
            targeted_cube_robot = (self.cube_robot) % 2 +1  # Wechsel zum anderen Roboter

        if (targeted_cube_robot == self.cube_robot):
            self.get_logger().warn(">>> [HandOver] Kein Griffwechsel nötig.")
            result = HandOver.Result()
            result.success = True
            result.cube_robot = self.cube_robot
            goal_handle.abort()
            return result

        self.get_logger().info(">>> [HandOver] Ausführen des Griffwechsels")



        # TODO: tatsächliche Roboterlogik hier implementieren
        
        self.cube_robot = targeted_cube_robot



        result = HandOver.Result()
        result.success = True
        result.cube_robot = self.cube_robot

        self.get_logger().info(">>> [HandOver] abgeschlossen.")
        if (result.success):
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result


    # -------------------------------
    # ROTATE FACE ACTION
    # -------------------------------
    async def execute_rotate_face(self, goal_handle):

        move = goal_handle.request.move
        self.get_logger().info(f">>> [RotateFace] Receive move {move}")

        face, angle = decode_kociemba_move(move)

        self.get_logger().info(f">>> [RotateFace] Rotating face {face} by {angle}°")

        # Beispiel: Vor dem Drehen muss evtl. umgegriffen werden
        if face not in ROBOT_FACES[self.cube_robot-1]:
            self.get_logger().info(">>> HandOver erforderlich")
            await self.call_hand_over()


        # TODO: Roboterbewegung implementieren


        result = RotateFace.Result()
        result.success = True
        self.get_logger().info(">>> [RotateFace] abgeschlossen.")
        if (result.success):
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    # -------------------------------
    # interne Hilfsfunktionen
    # -------------------------------
    async def call_hand_over(self):
        """Interner Aufruf des HandOver Actionsservers."""

        server_ready = self.hand_over_client.wait_for_server(timeout_sec=5.0)
        if not server_ready:
            self.get_logger().error("[HandOver] HandOver action server not available")
            return

        goal = HandOver.Goal()

        goal_future = self.hand_over_client.send_goal_async(goal)
        goal_handle = await goal_future

        result_future = goal_handle.get_result_async()
        result = await result_future

        if not result.result.success:
            self.get_logger().error("[HandOver] Fehler beim Umgreifen!")
        else:
            self.get_logger().info("[HandOver] internes Umgreifen erfolgreich.")


def main(args=None):
    rclpy.init(args=args)
    node = CubeMotionServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
