import rclpy
import os
import sys
import math
import numpy as np
import cv2
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

        # Parámetros
        self.declare_parameter("cell_size", 2.0)
        self.cell_size = self.get_parameter("cell_size").get_parameter_value().double_value

        # Servicio para procesar peticiones DARP
        self.srv = self.create_service(
            DarpPetition, "darp_service", self.service_callback
        )

        self.get_logger().info("Nodo DARP listo para recibir peticiones.")

    def service_callback(self, request, response):
        self.get_logger().info("Petición recibida")

        extent_x = request.max_x - request.min_x
        extent_y = request.max_y - request.min_y

        cols = int(round(extent_x / self.cell_size))
        rows = int(round(extent_y / self.cell_size))

        num_uavs = len(request.initial_positions)

        # Convertir coordenadas de interfaz a índices de celda DARP
        initial_positions, obstacles_positions = self.process_darp_input(
            request.initial_positions,
            request.obstacle_points,
            request.min_x,
            request.min_y,
            rows,
            cols,
        )

        # Detectar zonas de obstáculos y UAV divididos
        zones = self.process_darp_request(rows, cols, initial_positions, obstacles_positions)

        # Estructuras para combinar resultados
        all_trajectories = [None] * num_uavs
        grid_zones = np.zeros((rows, cols), dtype=np.int32)

        # Ejecutar DARP para cada zona
        for zone in zones:
            darp = MultiRobotPathPlanner(
                nx=rows,
                ny=cols,
                notEqualPortions=False,
                initial_positions=zone['uav_cells'],
                portions=[],
                obs_pos=zone['obstacles'],
                visualization=request.visualization,
            )
            if not darp.DARP_success:
                self.get_logger().error(f"DARP falló para zona {zone['label']}")
                return response

            # Cada trayectoria con índice correspondiente al UAV
            for i, idx_original in enumerate(zone['uav_indices']):
                all_trajectories[idx_original] = darp.best_case.paths[i]

            # Combinar matriz de zonas (mapear IDs locales a originales)
            for i in range(rows):
                for j in range(cols):
                    local_id = int(darp.darp_instance.A[i, j])
                    if local_id < len(zone['uav_indices']):
                        original_id = zone['uav_indices'][local_id]
                        grid_zones[i, j] = original_id + 1

        # Convertir resultados combinados a mensajes ROS2
        response.trajectories, response.zones = self.process_darp_output(
            all_trajectories,
            grid_zones,
            request.min_x,
            request.min_y
        )

        self.get_logger().info("Petición procesada exitosamente")
        return response

    """ Convierte puntos Point2D (coordenadas del mundo) a índices de celda para DARP """
    def process_darp_input(self, initial_positions_msg, obstacle_points_msg, min_x, min_y, rows, cols):
        # Convertir posiciones iniciales de robots
        initial_positions = []
        for point in initial_positions_msg:
            x = float(point.x)
            y = float(point.y)

            # Trasladar coordenadas al origen del grid
            grid_x = int(math.floor((x - float(min_x)) / self.cell_size))
            grid_y = int(math.floor((y - float(min_y)) / self.cell_size))

            # Asegurar que las coordenadas estén dentro del grid
            grid_x = max(0, min(cols - 1, grid_x))
            grid_y = max(0, min(rows - 1, grid_y))

            # Convertir a índice de celda
            cell = grid_y * cols + grid_x
            initial_positions.append(cell)

        # Convertir posiciones de obstáculos
        obstacles_positions = []
        for point in obstacle_points_msg:
            x = float(point.x)
            y = float(point.y)

            # Trasladar coordenadas al origen del grid
            grid_x = int(math.floor((x - float(min_x)) / self.cell_size))
            grid_y = int(math.floor((y - float(min_y)) / self.cell_size))

            # Asegurar que las coordenadas estén dentro del grid
            grid_x = max(0, min(cols - 1, grid_x))
            grid_y = max(0, min(rows - 1, grid_y))

            cell = grid_y * cols + grid_x
            obstacles_positions.append(cell)

        return initial_positions, obstacles_positions

    """ Rellena con obstáculos las zonas aisladas que no contienen UAVs y divide las que sí tienen """
    def process_darp_request(self, rows, cols, initial_positions, obstacles_positions):
        
        # Celdas libres = 255, obstáculos = 0
        grid = np.zeros((rows, cols), dtype=np.uint8)
        grid.fill(255)
        for obs_cell in obstacles_positions:
            row, col = obs_cell // cols, obs_cell % cols
            if 0 <= row < rows and 0 <= col < cols:
                grid[row, col] = 0

        # Detectar componentes conexos y agrupar UAVs por zona
        num_labels, labels_im = cv2.connectedComponents(grid, connectivity=4)
        uavs_por_zona = {}
        for idx, uav_cell in enumerate(initial_positions):
            row, col = uav_cell // cols, uav_cell % cols
            label = labels_im[row, col]
            if label not in uavs_por_zona:
                uavs_por_zona[label] = []
            uavs_por_zona[label].append(idx)

        # Preparar datos para cada zona con UAVs
        zones = []
        for label, indices_uavs in uavs_por_zona.items():
            # Obstáculos de una zona compuestos por los originales y las celdas de otras zonas
            obs_zona = list(obstacles_positions)
            for row in range(rows):
                for col in range(cols):
                    cell_label = labels_im[row, col]
                    if cell_label > 0 and cell_label != label:
                        obs_zona.append(row * cols + col)

            zones.append({
                'label': label,
                'uav_cells': [initial_positions[i] for i in indices_uavs],
                'uav_indices': indices_uavs,
                'obstacles': obs_zona
            })

        if len(zones) > 1:
            self.get_logger().info(f"Detectadas {len(zones)} zonas con UAVs")

        return zones

    def process_darp_output(self, todas_las_trayectorias, matriz_zonas, min_x, min_y):
        """Convierte trayectorias y matriz de zonas combinadas a mensajes ROS2."""
        trajectories = []

        subcell_size = self.cell_size / 2.0

        for path in todas_las_trayectorias:
            traj = Trajectory2D()

            if path and len(path) > 0:
                # Punto inicial
                first_move = path[0]
                p = Point2D()
                # (row, col) de subcelda -> (x,y) en metros (centro de subcelda)
                p.x = float(float(min_x) + (float(first_move[1]) + 0.5) * subcell_size)
                p.y = float(float(min_y) + (float(first_move[0]) + 0.5) * subcell_size)
                traj.points.append(p)

                # Puntos siguientes
                for move in path:
                    p = Point2D()
                    p.x = float(float(min_x) + (float(move[3]) + 0.5) * subcell_size)
                    p.y = float(float(min_y) + (float(move[2]) + 0.5) * subcell_size)
                    traj.points.append(p)

            trajectories.append(traj)

        # Aplanar matriz para el mensaje
        zones = matriz_zonas.flatten().tolist()

        return trajectories, zones


def main(args=None):
    rclpy.init(args=args)
    node = DarpNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
