import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import PointStamped
from vision_msgs.msg import Point2D

from std_msgs.msg import Int32MultiArray, MultiArrayDimension
from std_srvs.srv import Trigger

from multi_robot_planning_rms_msgs.srv import DarpPetition
from multi_robot_planning_rms_msgs.msg import Trajectory2D

class PlanningNode(Node):
    def __init__(self):
        super().__init__("planning_node")

        # Parámetros
        self.declare_parameter("agents.ids", [""])
        self.declare_parameter("cell_size", 2.0)
        self.declare_parameter("visited.update_period", 0.5)
        self.declare_parameter("tasks.min_x", 0)
        self.declare_parameter("tasks.max_x", 0)
        self.declare_parameter("tasks.min_y", 0)
        self.declare_parameter("tasks.max_y", 0)
        self.declare_parameter("tasks.obstacles_positions_x", [0.0])
        self.declare_parameter("tasks.obstacles_positions_y", [0.0])
        self.agent_ids = self.get_parameter("agents.ids").get_parameter_value().string_array_value
        self.original_agent_ids = list(self.agent_ids)  # Copia para mantener indices originales
        self.cell_size = self.get_parameter("cell_size").get_parameter_value().double_value
        self.visited_update_period = self.get_parameter("visited.update_period").get_parameter_value().double_value
        self.min_x = self.get_parameter("tasks.min_x").get_parameter_value().integer_value
        self.max_x = self.get_parameter("tasks.max_x").get_parameter_value().integer_value
        self.min_y = self.get_parameter("tasks.min_y").get_parameter_value().integer_value
        self.max_y = self.get_parameter("tasks.max_y").get_parameter_value().integer_value
        self.obstacles_positions_x = self.get_parameter("tasks.obstacles_positions_x").get_parameter_value().double_array_value
        self.obstacles_positions_y = self.get_parameter("tasks.obstacles_positions_y").get_parameter_value().double_array_value

        # Parámetros de simulación
        self.declare_parameter("simulation.enabled", False)
        self.declare_parameter("simulation.events_types", [""])
        self.declare_parameter("simulation.events_times", [0.0])
        self.declare_parameter("simulation.events_agents", [""])

        self.sim_enabled = self.get_parameter("simulation.enabled").get_parameter_value().bool_value
        self.sim_events = []
        self.sim_start_time = None
        
        if self.sim_enabled:
            types = self.get_parameter("simulation.events_types").get_parameter_value().string_array_value
            times = self.get_parameter("simulation.events_times").get_parameter_value().double_array_value
            agents = self.get_parameter("simulation.events_agents").get_parameter_value().string_array_value
            
            for t, tm, ag in zip(types, times, agents):
                if t:
                    self.sim_events.append({
                        "type": t,
                        "time": tm,
                        "agent": ag,
                        "fired": False
                    })
            self.sim_events.sort(key=lambda x: x["time"])
            self.sim_timer = self.create_timer(1.0, self.check_simulation_events)
            self.get_logger().info(f"Modo de simulacion activado con {len(self.sim_events)} eventos")

        self.last_remapped_zones = []  # Almacenar zonas base para visualización continua

        # Grid (cache)
        self.rows, self.cols = self.compute_rows_cols()
        self.traversed_positions = []
        self.visited_cells = set()

        # Timer para visualización constante de zonas y visitados
        self.vis_timer = self.create_timer(self.visited_update_period, self.publish_visualization_state)

        # QoS para trayectorias
        trajectory_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # QoS para zonas
        zones_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # QoS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Suscripciones de posiciones por agente
        self.position_subscribers = {}
        self.positions = {}  # agent_id -> PointStamped
        self.position_received = {}  # agent_id -> bool

        for agent_id in self.agent_ids:
            self.positions[agent_id] = None
            self.position_received[agent_id] = False
            self.position_subscribers[agent_id] = self.create_subscription(
                PointStamped,
                f"/{agent_id}/state/position",
                lambda msg, uid=agent_id: self.position_callback(msg, uid),
                qos_profile
            )

        # Publicadores de trayectorias por agente
        self.trajectory_publishers = {}
        for agent_id in self.agent_ids:
            self.trajectory_publishers[agent_id] = self.create_publisher(
                Trajectory2D, 
                f"/{agent_id}/planning/trajectory",
                trajectory_qos
            )

        # Publicador de zonas
        self.zones_pub = self.create_publisher(
            Int32MultiArray,
            "/planning/zones",
            zones_qos,
        )

        # Cliente de servicio DARP
        self.darp_client = self.create_client(DarpPetition, "darp_service")
        while not self.darp_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().info("Servicio DARP no disponible, esperando de nuevo...")

        self.plan_requested = False
        self.plan_done = False

        # Timer para solicitar la planificación inicial una vez que se reciben todas las posiciones
        self.timer = self.create_timer(0.5, self.maybe_request_initial_plan)

        # Servicio de replanning bajo demanda
        self.replan_srv = self.create_service(Trigger, "/planning/replan", self.replan_callback)

        self.get_logger().info("Nodo de planificacion iniciado. Esperando posiciones iniciales...")

    def position_callback(self, msg: PointStamped, agent_id: str):
        self.positions[agent_id] = msg
        self.position_received[agent_id] = True

        # Actualizar celdas visitadas
        if self.cell_size > 0:
            grid_x = int((msg.point.x - self.min_x) / self.cell_size)
            grid_y = int((msg.point.y - self.min_y) / self.cell_size)

            # Verificar limites
            if 0 <= grid_x < self.cols and 0 <= grid_y < self.rows:
                cell_index = (grid_x, grid_y)
                
                if cell_index not in self.visited_cells:
                    self.visited_cells.add(cell_index)
                    
                    # Calcular centro de la celda
                    center_x = self.min_x + (grid_x + 0.5) * self.cell_size
                    center_y = self.min_y + (grid_y + 0.5) * self.cell_size
                    
                    self.traversed_positions.append((center_x, center_y))
                    self.get_logger().debug(f"Nueva celda visitada: {cell_index} -> Centro: ({center_x:.2f}, {center_y:.2f})")

    def all_positions_received(self) -> bool:
        return all(self.position_received.get(agent_id, False) for agent_id in self.agent_ids)

    def maybe_request_initial_plan(self):
        if self.plan_done or self.plan_requested:
            return
        if not self.all_positions_received():
            return

        self.plan_requested = True
        self.request_darp(include_traversed=False)

    def request_darp(self, include_traversed: bool):
        if include_traversed:
            self.get_logger().info("Peticion de algoritmo DARP (replanning con visited)...")
        else:
            self.get_logger().info("Peticion de algoritmo DARP (planificacion inicial)...")
        req = DarpPetition.Request()
        req.min_x = int(self.min_x)
        req.max_x = int(self.max_x)
        req.min_y = int(self.min_y)
        req.max_y = int(self.max_y)
        req.visualization = False

        # Posiciones de los obstaculos
        for x, y in zip(self.obstacles_positions_x, self.obstacles_positions_y):
            p = Point2D()
            p.x = float(x)
            p.y = float(y)
            req.obstacle_points.append(p)

        # Posiciones iniciales de los agentes
        for agent_id in self.agent_ids:
            pos = self.positions.get(agent_id)
            if pos is None:
                self.get_logger().error(f"Posicion desconocida para agente {agent_id}, abortando peticion DARP")
                return
            p = Point2D()
            p.x = float(pos.point.x)
            p.y = float(pos.point.y)
            req.initial_positions.append(p)

        # Posiciones ya visitadas por cualquier agente
        if include_traversed:
            # Calcular las celdas actuales de todos los agentes activos para excluirlas de traversed
            current_agent_cells = set()
            for agent_id in self.agent_ids:
                pos = self.positions.get(agent_id)
                if pos:
                    grid_x = int((pos.point.x - self.min_x) / self.cell_size)
                    grid_y = int((pos.point.y - self.min_y) / self.cell_size)
                    # Convertir a coordenadas de centro de celda para comparar con traversed_positions
                    center_x = self.min_x + (grid_x + 0.5) * self.cell_size
                    center_y = self.min_y + (grid_y + 0.5) * self.cell_size
                    current_agent_cells.add((center_x, center_y))

            for x, y in self.traversed_positions:
                # Si la posición visitada corresponde a la posición actual de algún agente, ignorarla
                if (x, y) in current_agent_cells:
                    continue
                
                p = Point2D()
                p.x = float(x)
                p.y = float(y)
                req.traversed_points.append(p)

        future = self.darp_client.call_async(req)
        future.add_done_callback(self.darp_callback)

    def darp_callback(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f"Llamada al servicio DARP fallida: {e}")
            self.plan_requested = False
            return

        if not response.trajectories:
            self.get_logger().error("DARP devolvio trayectorias vacias. Manteniendo trayectorias anteriores...")
            self.plan_requested = False
            return

        self.get_logger().info(
            f"Solucion DARP recibida con {len(response.trajectories)} trayectorias"
        )

        # Cachear y procesar zones para visualización
        if response.zones:
            # Remapear zonas: DARP devuelve indices 1..N de la lista ACTIVA
            # Visualizacion espera indices 1..M de la lista ORIGINAL
            
            # Crear tabla de mapeo para eficiencia O(1) en el bucle
            zone_map = {0: 0}  # 0 es obstaculo/vacio
            
            for i, agent_id in enumerate(self.agent_ids):
                current_idx = i + 1  # Indice 1-based que devuelve DARP
                if agent_id in self.original_agent_ids:
                    # Mapear al indice original 1-based
                    original_idx = self.original_agent_ids.index(agent_id) + 1
                    zone_map[current_idx] = original_idx
                else:
                    zone_map[current_idx] = current_idx
            
            # Aplicar mapeo vectorizado (mucho mas rapido que buscar en cada iteracion)
            remapped_zones = [zone_map.get(val, val) for val in response.zones]
            
            # Guardar la base de zonas para el timer de visualización
            self.last_remapped_zones = remapped_zones
            # Forzar actualización inmediata
            self.publish_visualization_state()

        # Publicar trayectorias por agente basado en el orden de indices
        for i, traj in enumerate(response.trajectories):
            if i >= len(self.agent_ids):
                self.get_logger().warn(
                    f"Mas trayectorias que UAVs: Trayectoria {i} ignorada"
                )
                continue
            agent_id = self.agent_ids[i]
            msg = Trajectory2D()
            msg.points = traj.points
            msg.valid = True
            self.trajectory_publishers[agent_id].publish(msg)
            self.get_logger().info(
                f"Publicada trayectoria (len={len(msg.points)}) en /{agent_id}/planning/trajectory"
            )

        self.plan_done = True
        self.plan_requested = False

        if self.sim_enabled and self.sim_start_time is None:
            self.sim_start_time = self.get_clock().now()
            self.get_logger().info("Temporizador de simulacion iniciado")

    def publish_visualization_state(self):
        """Publica el estado actual de zonas y celdas visitadas."""
        # Si no hay zonas calculadas, usar ceros (todo obstáculo/desconocido)
        if not self.last_remapped_zones:
            if self.rows > 0 and self.cols > 0:
                current_zones = [0] * (self.rows * self.cols)
            else:
                return
        else:
            current_zones = list(self.last_remapped_zones)

        # Superponer celdas visitadas para visualización (color gris claro, valor -1)
        if self.cell_size > 0:
            for x, y in self.traversed_positions:
                grid_x = int((x - self.min_x) / self.cell_size)
                grid_y = int((y - self.min_y) / self.cell_size)
                
                # Verificar límites antes de calcular índice lineal
                if 0 <= grid_x < self.cols and 0 <= grid_y < self.rows:
                    idx = grid_y * self.cols + grid_x
                    if 0 <= idx < len(current_zones):
                        current_zones[idx] = -1

        self.publish_zones(current_zones)

    def replan_callback(self, request, response):
        if self.plan_requested:
            response.success = False
            response.message = "Replan rechazado: ya hay una petición DARP en curso"
            return response
        if not self.all_positions_received():
            response.success = False
            response.message = "Replan rechazado: faltan posiciones iniciales"
            return response

        self.plan_requested = True
        self.get_logger().info("Replan solicitado: llamando a DARP con visited como obstáculos")
        self.request_darp(include_traversed=True)
        response.success = True
        response.message = "Replan solicitado"
        return response

    def compute_rows_cols(self):
        extent_x = int(self.max_x) - int(self.min_x)
        extent_y = int(self.max_y) - int(self.min_y)
        cols = int(round(float(extent_x) / float(self.cell_size))) if self.cell_size > 0.0 else 0
        rows = int(round(float(extent_y) / float(self.cell_size))) if self.cell_size > 0.0 else 0
        return max(0, rows), max(0, cols)

    def publish_zones(self, zones_list):
        data = list(zones_list)

        zones_msg = Int32MultiArray()
        zones_msg.layout.dim = [
            MultiArrayDimension(label="rows", size=int(self.rows), stride=int(self.cols)),
            MultiArrayDimension(label="cols", size=int(self.cols), stride=1),
        ]
        zones_msg.layout.data_offset = 0
        zones_msg.data = data
        self.zones_pub.publish(zones_msg)

    def check_simulation_events(self):
        if not self.sim_enabled or self.sim_start_time is None:
            return

        elapsed = (self.get_clock().now() - self.sim_start_time).nanoseconds / 1e9

        for event in self.sim_events:
            if not event["fired"] and elapsed >= event["time"]:
                self.fire_event(event)
                event["fired"] = True

    def fire_event(self, event):
        self.get_logger().info(f"Activando evento de simulacion: {event['type']} para el agente {event['agent']} a los {event['time']} segundos")
        if event["type"] == "agent_failure":
            self.handle_agent_failure(event["agent"])

    def handle_agent_failure(self, agent_id):
        if agent_id not in self.agent_ids:
            return

        self.get_logger().warn(f"El agente {agent_id} ha fallado. Replanificando...")

        # 1. Eliminar del listado de agentes activos
        self.agent_ids.remove(agent_id)
        
        # 2. Limpiar estado de posicion recibida para el agente fallido
        if agent_id in self.position_received:
            self.position_received.pop(agent_id)

        # 3. Publicar trayectoria vacía para detener el agente y limpiar visualización
        empty_traj = Trajectory2D()
        empty_traj.valid = False
        self.trajectory_publishers[agent_id].publish(empty_traj)
        self.get_logger().info(f"Publicada trayectoria vacia para detener al agente {agent_id}")

        # 4. Solicitar replanificación
        if not self.plan_requested:
            self.plan_requested = True
            self.request_darp(include_traversed=True)

def main(args=None):
    rclpy.init(args=args)
    node = PlanningNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()


