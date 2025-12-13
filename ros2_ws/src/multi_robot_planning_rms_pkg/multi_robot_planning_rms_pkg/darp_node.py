import rclpy
import os
import sys
import numpy as np
from rclpy.node import Node
from multi_robot_planning_rms_msgs.srv import DarpPetition
from multi_robot_planning_rms_msgs.msg import Trajectory2D
from vision_msgs.msg import Point2D

scripts_path = os.path.join(os.path.dirname(__file__), "..", "scripts")
sys.path.insert(0, os.path.abspath(scripts_path))

from multiRobotPathPlanner import MultiRobotPathPlanner


class DarpNode(Node):
    def __init__(self):
        super().__init__("darp_node")

        # Servicio para procesar peticiones DARP
        self.srv = self.create_service(
            DarpPetition, "darp_service", self.service_callback
        )

        self.get_logger().info("Nodo DARP listo para recibir peticiones.")

    def service_callback(self, request, response):
        self.get_logger().info("Petición recibida")

        rows = request.max_y - request.min_y + 1
        cols = request.max_x - request.min_x + 1

        # Convertir coordenadas de interfaz a índices de celda DARP
        initial_positions, obstacles_positions = self.process_darp_input(
            request.initial_positions,
            request.obstacle_points,
            request.min_x,
            request.min_y,
            cols
        )
        
        # Ejecutar algoritmo DARP
        darp = MultiRobotPathPlanner(
            nx=rows,
            ny=cols,
            notEqualPortions=False,
            initial_positions=initial_positions,
            portions=[],
            obs_pos=obstacles_positions,
            visualization=request.visualization,
        )

        if not darp.DARP_success:
            self.get_logger().error("DARP failed")
            return response

        # Convertir resultados de DARP a mensajes ROS2
        response.trajectories, response.zones = self.process_darp_output(
            darp,
            request.min_x,
            request.min_y,
            rows,
            cols
        )

        self.get_logger().info("Petición procesada exitosamente")
        return response

    """ Convierte puntos Point2D (coordenadas del mundo) a índices de celda para DARP """
    def process_darp_input(self, initial_positions_msg, obstacle_points_msg, min_x, min_y, cols):

        # Convertir posiciones iniciales de robots
        initial_positions = []
        for point in initial_positions_msg:
            x = int(point.x)
            y = int(point.y)
            
            # Trasladar coordenadas al origen del grid
            grid_x = x - min_x
            grid_y = y - min_y
            
            # Convertir a índice de celda
            cell = grid_y * cols + grid_x
            initial_positions.append(cell)

        # Convertir posiciones de obstáculos
        obstacles_positions = []
        for point in obstacle_points_msg:
            x = int(point.x)
            y = int(point.y)
            
            grid_x = x - min_x
            grid_y = y - min_y
            
            cell = grid_y * cols + grid_x
            obstacles_positions.append(cell)

        return initial_positions, obstacles_positions

    """ Convierte resultados de DARP (trayectorias y zonas) a mensajes ROS2. """
    def process_darp_output(self, planner, min_x, min_y, rows, cols):

        trajectories = []

        # Procesar trayectorias de cada robot
        for robot_id in range(planner.darp_instance.droneNo):
            path = planner.best_case.paths[robot_id]

            traj = Trajectory2D()

            if len(path) > 0:
                # Punto inicial
                first_move = path[0]
                p = Point2D()
                p.x = float(first_move[1] + min_x)  # col_from + offset_x
                p.y = float(first_move[0] + min_y)  # row_from + offset_y
                traj.points.append(p)

                # Puntos siguientes
                for move in path:
                    p = Point2D()
                    p.x = float(move[3] + min_x)  # col_to + offset_x
                    p.y = float(move[2] + min_y)  # row_to + offset_y
                    traj.points.append(p)

            trajectories.append(traj)

        # Procesar matriz de zonas
        assignment_matrix = planner.darp_instance.A
        zones_matrix = np.zeros((rows, cols), dtype=np.int32)

        for i in range(rows):
            for j in range(cols):
                cell_value = int(assignment_matrix[i, j])
                # Si es obstáculo, asignar 0. Si no, asignar ID de UAV + 1
                if cell_value == planner.darp_instance.droneNo:
                    zones_matrix[i, j] = 0
                else:
                    zones_matrix[i, j] = cell_value + 1

        # Aplanar matriz para el mensaje
        zones = zones_matrix.flatten().tolist()

        return trajectories, zones


def main(args=None):
    rclpy.init(args=args)
    node = DarpNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
