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
        initial_positions, obstacles_positions, traversed_positions = self.process_darp_input(
            request.initial_positions,
            request.obstacle_points,
            request.traversed_points,
            request.min_x,
            request.min_y,
            rows,
            cols
        )

        # Detectar zonas alcanzables y sub-regiones
        zones = self.process_darp_request(
            rows, cols,
            initial_positions,
            obstacles_positions,
            traversed_positions
        )

        # Estructuras para combinar resultados
        all_trajectories = [[] for _ in range(num_uavs)]
        grid_zones = np.zeros((rows, cols), dtype=np.int32)

        # Ejecutar DARP para cada zona/sub-región
        darp_results = []
        for zone in zones:
            free_cells_count = len(zone['free_cells'])

            # Recortar grid al bounding box de la sub-región
            crop_rows, crop_cols, crop_uav_cells, crop_obstacles, row_off, col_off = \
                self.crop_for_darp(zone, cols)

            self.get_logger().info(
                f"Zona {zone['label']}: {free_cells_count} celdas libres, "
                f"grid recortado {crop_rows}x{crop_cols}, offset=({row_off},{col_off})"
            )

            # Para sub-regiones muy pequeñas, usar cobertura simple directamente
            if free_cells_count <= 3:
                simple_path, assignment = self.generate_simple_coverage(
                    crop_rows, crop_cols, crop_uav_cells[0], crop_obstacles
                )
                darp = self._make_mock(simple_path, assignment)
            else:
                # Ejecutar DARP con grid recortado
                darp = MultiRobotPathPlanner(
                    nx=crop_rows,
                    ny=crop_cols,
                    notEqualPortions=False,
                    initial_positions=crop_uav_cells,
                    portions=[],
                    obs_pos=crop_obstacles,
                    visualization=request.visualization,
                )

                if not darp.DARP_success:
                    self.get_logger().warn(
                        f"DARP falló para zona {zone['label']} ({free_cells_count} celdas), "
                        f"generando trayectoria simple"
                    )
                    simple_path, assignment = self.generate_simple_coverage(
                        crop_rows, crop_cols, crop_uav_cells[0], crop_obstacles
                    )
                    darp = self._make_mock(simple_path, assignment)

            # Mapear paths de vuelta a coordenadas del grid completo
            uncropped_paths = [
                self._uncrop_path(p, row_off, col_off)
                for p in darp.best_case.paths
            ]
            darp.best_case = type('obj', (object,), {'paths': uncropped_paths})()

            darp_results.append(darp)

            # Combinar matriz de zonas (mapear de grid recortado a completo)
            for i in range(crop_rows):
                for j in range(crop_cols):
                    local_id = int(darp.darp_instance.A[i, j])
                    if 0 <= local_id < len(zone['uav_indices']):
                        original_id = zone['uav_indices'][local_id]
                        grid_zones[i + row_off, j + col_off] = original_id + 1

        # === POST-PROCESAMIENTO: Combinar sub-regiones por UAV ===
        for uav_idx in range(num_uavs):
            uav_subtrajectories = []

            # Recopilar trayectorias de todas las sub-regiones de este UAV
            for zone_idx, zone in enumerate(zones):
                # Sub-región principal (tiene el UAV directamente)
                if uav_idx in zone['uav_indices']:
                    local_idx = zone['uav_indices'].index(uav_idx)
                    uav_subtrajectories.append({
                        'trajectory': darp_results[zone_idx].best_case.paths[local_idx],
                        'is_secondary': False
                    })
                # Sub-región secundaria (pertenece a este UAV vía parent_uav)
                elif zone.get('parent_uav') == uav_idx:
                    # La trayectoria es del "UAV virtual" en índice 0
                    uav_subtrajectories.append({
                        'trajectory': darp_results[zone_idx].best_case.paths[0],
                        'is_secondary': True
                    })

            if len(uav_subtrajectories) == 0:
                all_trajectories[uav_idx] = []
            elif len(uav_subtrajectories) == 1:
                all_trajectories[uav_idx] = uav_subtrajectories[0]['trajectory']
            else:
                # Múltiples sub-regiones: ordenar (primarias primero) y concatenar
                uav_subtrajectories.sort(key=lambda x: x['is_secondary'])
                combined = []
                for sub in uav_subtrajectories:
                    # Transición point-to-point entre sub-trayectorias
                    if combined and sub['trajectory']:
                        prev_end = combined[-1]
                        next_start = sub['trajectory'][0]
                        combined.append((prev_end[2], prev_end[3], next_start[0], next_start[1]))
                    combined.extend(sub['trajectory'])
                all_trajectories[uav_idx] = combined

        # Convertir resultados combinados a mensajes ROS2
        response.trajectories, response.zones = self.process_darp_output(
            all_trajectories,
            grid_zones,
            request.min_x,
            request.min_y
        )

        self.get_logger().info("Petición procesada exitosamente")
        return response

    def process_darp_input(self, initial_positions_msg, obstacle_points_msg,
                          traversed_points_msg, min_x, min_y, rows, cols):
        """Convierte puntos Point2D (coordenadas del mundo) a índices de celda para DARP"""

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

        # Convertir posiciones de trayectoria recorrida
        traversed_positions = []
        for point in traversed_points_msg:
            x = float(point.x)
            y = float(point.y)

            # Trasladar coordenadas al origen del grid
            grid_x = int(math.floor((x - float(min_x)) / self.cell_size))
            grid_y = int(math.floor((y - float(min_y)) / self.cell_size))

            # Asegurar que las coordenadas estén dentro del grid
            grid_x = max(0, min(cols - 1, grid_x))
            grid_y = max(0, min(rows - 1, grid_y))

            cell = grid_y * cols + grid_x
            traversed_positions.append(cell)

        return initial_positions, obstacles_positions, traversed_positions

    def process_darp_request(self, rows, cols, initial_positions,
                            obstacles_positions, traversed_positions):
        """
        Detecta zonas alcanzables y sub-regiones considerando:
        - Zonas alcanzables: Solo separadas por obstáculos reales
        - Sub-regiones: Separadas por obstáculos reales + trayectoria recorrida
        """

        # ===== PASO 1: Detectar zonas ALCANZABLES (solo obstáculos reales) =====
        grid_reachable = np.zeros((rows, cols), dtype=np.uint8)
        grid_reachable.fill(255)
        for obs_cell in obstacles_positions:
            row, col = obs_cell // cols, obs_cell % cols
            if 0 <= row < rows and 0 <= col < cols:
                grid_reachable[row, col] = 0

        _, labels_reachable = cv2.connectedComponents(grid_reachable, connectivity=4)

        # ===== PASO 2: Detectar SUB-REGIONES (obstáculos + traversed) =====
        grid_subregions = np.zeros((rows, cols), dtype=np.uint8)
        grid_subregions.fill(255)

        # Marcar obstáculos reales
        for obs_cell in obstacles_positions:
            row, col = obs_cell // cols, obs_cell % cols
            if 0 <= row < rows and 0 <= col < cols:
                grid_subregions[row, col] = 0

        # Marcar trayectorias recorridas
        for trav_cell in traversed_positions:
            row, col = trav_cell // cols, trav_cell % cols
            if 0 <= row < rows and 0 <= col < cols:
                grid_subregions[row, col] = 0

        num_subregions, labels_subregions = cv2.connectedComponents(grid_subregions, connectivity=4)

        # ===== PASO 3: Agrupar UAVs por zona alcanzable =====
        uavs_per_reachable = {}
        for idx, uav_cell in enumerate(initial_positions):
            row, col = uav_cell // cols, uav_cell % cols
            reach_label = labels_reachable[row, col]
            if reach_label not in uavs_per_reachable:
                uavs_per_reachable[reach_label] = []
            uavs_per_reachable[reach_label].append(idx)

        # ===== PASO 4: Para cada zona alcanzable, identificar sub-regiones =====
        all_zones = []

        for reach_label, uav_indices in uavs_per_reachable.items():
            # Encontrar todas las sub-regiones en esta zona alcanzable
            subregions_info = []

            for sub_label in range(1, num_subregions):
                # Celdas de esta sub-región que pertenecen a la zona alcanzable
                cells_in_sub = []
                for row in range(rows):
                    for col in range(cols):
                        if (labels_subregions[row, col] == sub_label and
                            labels_reachable[row, col] == reach_label):
                            cells_in_sub.append(row * cols + col)

                if len(cells_in_sub) == 0:
                    continue

                # Filtrar solo celdas libres (no traversed ni obstáculos)
                free_cells = [c for c in cells_in_sub
                             if c not in traversed_positions and c not in obstacles_positions]

                if len(free_cells) == 0:
                    # Sub-región completamente recorrida o con obstáculos
                    continue

                # ¿Hay UAV en esta sub-región?
                uavs_in_sub = [idx for idx in uav_indices
                              if initial_positions[idx] in cells_in_sub]

                subregions_info.append({
                    'sub_label': sub_label,
                    'all_cells': cells_in_sub,
                    'free_cells': free_cells,
                    'has_uav': len(uavs_in_sub) > 0,
                    'uav_indices': uavs_in_sub
                })

            if len(subregions_info) == 0:
                # No hay sub-regiones con celdas libres en esta zona alcanzable
                continue

            # ===== PASO 5: Planificar cada sub-región =====
            for sub in subregions_info:
                if sub['has_uav']:
                    # Sub-región con UAV: usar posiciones reales
                    uav_cells = [initial_positions[i] for i in sub['uav_indices']]
                    zone_uav_indices = sub['uav_indices']
                    parent_uav = None
                else:
                    # Sub-región sin UAV: encontrar punto de entrada óptimo
                    entry_cell = self.find_entry_point(
                        sub['free_cells'],
                        subregions_info,
                        cols
                    )
                    uav_cells = [entry_cell]
                    zone_uav_indices = []
                    parent_uav = uav_indices[0]

                # Crear obstáculos para esta sub-región
                obs_zona = list(obstacles_positions) + list(traversed_positions)

                # Añadir otras sub-regiones como obstáculos virtuales
                for other_sub in subregions_info:
                    if other_sub['sub_label'] != sub['sub_label']:
                        obs_zona.extend(other_sub['all_cells'])

                # Añadir otras zonas alcanzables como obstáculos
                for row in range(rows):
                    for col in range(cols):
                        other_reach_label = labels_reachable[row, col]
                        if other_reach_label > 0 and other_reach_label != reach_label:
                            obs_zona.append(row * cols + col)

                # Log detallado de la sub-región
                self.get_logger().info(
                    f"Sub-región {reach_label}.{sub['sub_label']}: "
                    f"UAVs={zone_uav_indices if zone_uav_indices else f'virtual (parent={parent_uav})'}, "
                    f"Celdas libres={len(sub['free_cells'])}"
                )

                all_zones.append({
                    'label': f"{reach_label}.{sub['sub_label']}",
                    'uav_cells': uav_cells,
                    'uav_indices': zone_uav_indices,
                    'parent_uav': parent_uav,
                    'obstacles': obs_zona,
                    'free_cells': sub['free_cells'],
                    'is_secondary': not sub['has_uav']
                })

        if len(all_zones) > len(uavs_per_reachable):
            self.get_logger().info(
                f"Detectadas {len(all_zones)} sub-regiones a planificar "
                f"en {len(uavs_per_reachable)} zonas alcanzables"
            )

        return all_zones

    def crop_for_darp(self, zone, full_cols):
        """Recorta el grid al bounding box de las celdas libres + UAV de la zona."""
        relevant = list(zone['free_cells']) + list(zone['uav_cells'])
        coords = [(cell // full_cols, cell % full_cols) for cell in relevant]

        min_row = min(r for r, c in coords)
        max_row = max(r for r, c in coords)
        min_col = min(c for r, c in coords)
        max_col = max(c for r, c in coords)

        crop_rows = max_row - min_row + 1
        crop_cols = max_col - min_col + 1

        # Mapear UAV cells a coordenadas recortadas
        crop_uav_cells = []
        for cell in zone['uav_cells']:
            r, c = cell // full_cols, cell % full_cols
            crop_uav_cells.append((r - min_row) * crop_cols + (c - min_col))

        # Mapear obstáculos: solo los que caen dentro del bounding box
        obstacles_set = set(zone['obstacles'])
        crop_obstacles = []
        for r in range(min_row, max_row + 1):
            for c in range(min_col, max_col + 1):
                cell = r * full_cols + c
                if cell in obstacles_set:
                    crop_obstacles.append((r - min_row) * crop_cols + (c - min_col))

        return crop_rows, crop_cols, crop_uav_cells, crop_obstacles, min_row, min_col

    def _uncrop_path(self, path, row_off, col_off):
        """Mapea un path sub-celda (2x) de coordenadas recortadas al grid completo."""
        sr, sc = 2 * row_off, 2 * col_off
        return [
            (m[0] + sr, m[1] + sc, m[2] + sr, m[3] + sc)
            for m in path
        ]

    def _make_mock(self, simple_path, assignment_matrix):
        """Crea un objeto mock compatible con la interfaz de DARP."""
        return type('obj', (object,), {
            'DARP_success': True,
            'best_case': type('obj', (object,), {
                'paths': [simple_path]
            })(),
            'darp_instance': type('obj', (object,), {
                'A': assignment_matrix
            })()
        })()

    def generate_simple_coverage(self, rows, cols, start_cell, obstacles):
        """
        Genera un patrón de cobertura simple para regiones pequeñas.
        Usado como fallback cuando DARP falla.
        Retorna: (path, assignment_matrix)
        """
        obstacles_set = set(obstacles)

        # Identificar todas las celdas libres
        free_cells = []
        for row in range(rows):
            for col in range(cols):
                cell = row * cols + col
                if cell not in obstacles_set:
                    free_cells.append((row, col))

        if len(free_cells) == 0:
            return [], np.full((rows, cols), -1, dtype=np.int32)

        # Crear matriz de asignación (todas las celdas libres al robot 0)
        assignment_matrix = np.full((rows, cols), -1, dtype=np.int32)
        for row, col in free_cells:
            assignment_matrix[row, col] = 0

        # Ordenar celdas por proximidad al inicio (nearest neighbor greedy)
        start_row = start_cell // cols
        start_col = start_cell % cols
        current = (start_row, start_col)

        path = []
        remaining = set(free_cells)

        while remaining:
            if current in remaining:
                remaining.remove(current)

            if not remaining:
                break

            nearest = min(remaining, key=lambda c: abs(c[0] - current[0]) + abs(c[1] - current[1]))

            path.append((2*current[0], 2*current[1], 2*nearest[0], 2*nearest[1]))
            current = nearest
            remaining.remove(nearest)

        return path, assignment_matrix

    def find_entry_point(self, target_free_cells, all_subregions, cols):
        """
        Encuentra el punto de entrada óptimo en la sub-región objetivo
        desde las sub-regiones que contienen UAVs.
        """
        source_cells = []
        for sub in all_subregions:
            if sub['has_uav']:
                source_cells.extend(sub['free_cells'])

        if len(source_cells) == 0:
            return target_free_cells[0]

        min_distance = float('inf')
        best_entry = target_free_cells[0]

        for target_cell in target_free_cells:
            target_row = target_cell // cols
            target_col = target_cell % cols

            for source_cell in source_cells:
                source_row = source_cell // cols
                source_col = source_cell % cols

                distance = abs(target_row - source_row) + abs(target_col - source_col)

                if distance < min_distance:
                    min_distance = distance
                    best_entry = target_cell

        return best_entry

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
