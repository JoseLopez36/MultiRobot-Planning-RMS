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

        # Grid (cache)
        self.rows, self.cols = self.compute_rows_cols()

        # Estado de mapa visitado: celdas ya cubiertas por cualquier agente
        self.visited_mask = set()  # set[int] de índices flat (row*cols+col)
        self.static_obstacle_cells = self.compute_static_obstacle_cells()
        self.zones_current = None
        self.zones_previous = None
        self.last_published_zones = None  # list[int]

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

        # Timer para marcar celdas visitadas y refrescar zones
        self.visited_timer = self.create_timer(
            float(self.visited_update_period) if self.visited_update_period > 0.0 else 0.5,
            self.update_visited_from_positions,
        )

        # Servicio de replanning bajo demanda
        self.replan_srv = self.create_service(Trigger, "/planning/replan", self.replan_callback)

        self.get_logger().info("Nodo de planificacion iniciado. Esperando posiciones iniciales...")

    def position_callback(self, msg: PointStamped, agent_id: str):
        self.positions[agent_id] = msg
        self.position_received[agent_id] = True

    def all_positions_received(self) -> bool:
        return all(self.position_received.get(agent_id, False) for agent_id in self.agent_ids)

    def maybe_request_initial_plan(self):
        if self.plan_done or self.plan_requested:
            return
        if not self.all_positions_received():
            return

        self.plan_requested = True
        self.request_darp(include_visited=False)

    def request_darp(self, include_visited: bool):
        if include_visited:
            self.get_logger().info("Peticion de algoritmo DARP (replanning con visited)...")
        else:
            self.get_logger().info("Peticion de algoritmo DARP (planificacion inicial)...")
        req = DarpPetition.Request()
        req.min_x = int(self.min_x)
        req.max_x = int(self.max_x)
        req.min_y = int(self.min_y)
        req.max_y = int(self.max_y)
        req.visualization = False

        # Obstaculos desde parametros
        for x, y in zip(self.obstacles_positions_x, self.obstacles_positions_y):
            p = Point2D()
            p.x = float(x)
            p.y = float(y)
            req.obstacle_points.append(p)

        # Posiciones iniciales desde suscripciones
        for agent_id in self.agent_ids:
            pos = self.positions.get(agent_id)
            if pos is None:
                continue
            p = Point2D()
            p.x = float(pos.point.x)
            p.y = float(pos.point.y)
            req.initial_positions.append(p)

        # Celdas visitadas como puntos recorridos (solo para replanning)
        if include_visited:
            start_cells = set()
            for agent_id in self.agent_ids:
                pos = self.positions.get(agent_id)
                if pos is None:
                    continue
                cell = self.world_to_cell(float(pos.point.x), float(pos.point.y))
                if cell is not None:
                    start_cells.add(int(cell))

            for cell in sorted(self.visited_mask):
                if cell in start_cells:
                    continue
                if cell in self.static_obstacle_cells:
                    continue
                pt = self.cell_to_world_center(int(cell))
                if pt is None:
                    continue
                p = Point2D()
                p.x = float(pt[0])
                p.y = float(pt[1])
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

        # Cachear y publicar zones para visualización (visited = 0)
        if response.zones:
            self.zones_current = list(response.zones)
            self.apply_original_indices_to_zones_inplace(self.zones_current)
            self.apply_visited_to_zones_inplace(self.zones_current)
            self.publish_zones(self.zones_current)

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
            self.trajectory_publishers[agent_id].publish(msg)
            self.get_logger().info(
                f"Publicada trayectoria (len={len(msg.points)}) en /{agent_id}/planning/trajectory"
            )

        self.plan_done = True
        self.plan_requested = False

        if self.sim_enabled and self.sim_start_time is None:
            self.sim_start_time = self.get_clock().now()
            self.get_logger().info("Temporizador de simulacion iniciado")

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
        self.request_darp(include_visited=True)
        response.success = True
        response.message = "Replan solicitado"
        return response

    def compute_rows_cols(self):
        extent_x = int(self.max_x) - int(self.min_x)
        extent_y = int(self.max_y) - int(self.min_y)
        cols = int(round(float(extent_x) / float(self.cell_size))) if self.cell_size > 0.0 else 0
        rows = int(round(float(extent_y) / float(self.cell_size))) if self.cell_size > 0.0 else 0
        return max(0, rows), max(0, cols)

    def world_to_cell(self, x: float, y: float):
        if self.rows <= 0 or self.cols <= 0 or self.cell_size <= 0.0:
            return None
        gx = int(math.floor((x - float(self.min_x)) / float(self.cell_size)))
        gy = int(math.floor((y - float(self.min_y)) / float(self.cell_size)))
        gx = max(0, min(self.cols - 1, gx))
        gy = max(0, min(self.rows - 1, gy))
        return int(gy * self.cols + gx)

    def cell_to_world_center(self, cell: int):
        if self.rows <= 0 or self.cols <= 0 or self.cell_size <= 0.0:
            return None
        if cell < 0 or cell >= (self.rows * self.cols):
            return None
        r = int(cell // self.cols)
        c = int(cell % self.cols)
        x = float(self.min_x) + (float(c) + 0.5) * float(self.cell_size)
        y = float(self.min_y) + (float(r) + 0.5) * float(self.cell_size)
        return (x, y)

    def compute_static_obstacle_cells(self):
        cells = set()
        for x, y in zip(self.obstacles_positions_x, self.obstacles_positions_y):
            cell = self.world_to_cell(float(x), float(y))
            if cell is not None:
                cells.add(int(cell))
        return cells

    def apply_original_indices_to_zones_inplace(self, zones_list):
        """
        Remapea los IDs de zona (1..N) devueltos por DARP (basados en self.agent_ids actual)
        a los IDs originales (1..M) basados en self.original_agent_ids.
        Esto preserva los colores en el nodo de visualización.
        """
        if not zones_list or not self.agent_ids:
            return

        # Crear mapa: {id_darp (1..N) -> id_original (1..M)}
        # id_darp = i + 1, donde i es indice en self.agent_ids
        # id_original = j + 1, donde j es indice en self.original_agent_ids
        mapping = {}
        for i, agent_id in enumerate(self.agent_ids):
            if agent_id in self.original_agent_ids:
                orig_idx = self.original_agent_ids.index(agent_id)
                mapping[i + 1] = orig_idx + 1
        
        # Aplicar mapeo
        for k in range(len(zones_list)):
            val = zones_list[k]
            if val > 0:  # Si es una zona asignada (no obstaculo 0, no visitado -1)
                if val in mapping:
                    zones_list[k] = mapping[val]

    def apply_visited_to_zones_inplace(self, zones_list):
        if zones_list is None:
            return
        for cell in self.visited_mask:
            idx = int(cell)
            if idx < 0 or idx >= len(zones_list):
                continue
            # Marcar como visitado (-1)
            zones_list[idx] = -1

    def publish_zones(self, zones_list):
        if zones_list is None:
            return
        if self.rows <= 0 or self.cols <= 0:
            return
        expected = int(self.rows * self.cols)
        data = list(zones_list[:expected]) if len(zones_list) >= expected else list(zones_list)

        zones_msg = Int32MultiArray()
        zones_msg.layout.dim = [
            MultiArrayDimension(label="rows", size=int(self.rows), stride=int(self.rows * self.cols)),
            MultiArrayDimension(label="cols", size=int(self.cols), stride=int(self.cols)),
        ]
        zones_msg.layout.data_offset = 0
        zones_msg.data = data
        self.zones_pub.publish(zones_msg)
        self.last_published_zones = list(data)

    def update_visited_from_positions(self):
        changed = False
        for agent_id in self.agent_ids:
            pos = self.positions.get(agent_id)
            if pos is None:
                continue
            cell = self.world_to_cell(float(pos.point.x), float(pos.point.y))
            if cell is None:
                continue
            if cell in self.static_obstacle_cells:
                continue
            if cell not in self.visited_mask:
                self.visited_mask.add(int(cell))
                changed = True

        if not changed:
            return

        # Si ya tenemos zones, aplicar visited y republicar
        if self.zones_current is not None:
            self.apply_visited_to_zones_inplace(self.zones_current)
            self.publish_zones(self.zones_current)

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

        # 1. Agregar la posicion actual a los obstaculos estaticos
        pos = self.positions.get(agent_id)
        if pos:
            cell = self.world_to_cell(pos.point.x, pos.point.y)
            if cell is not None:
                self.static_obstacle_cells.add(int(cell))
                self.get_logger().info(f"Ubicacion del agente {agent_id} (celda {cell}) agregada a los obstaculos estaticos")

        # 2. Eliminar del listado de agentes activos
        self.agent_ids.remove(agent_id)
        
        # 3. Limpiar estado de posicion recibida para el agente fallido
        if agent_id in self.position_received:
            self.position_received.pop(agent_id)

        # 4. Publicar trayectoria vacía para detener el agente y limpiar visualización
        empty_traj = Trajectory2D()
        empty_traj.points = []
        self.trajectory_publishers[agent_id].publish(empty_traj)
        self.get_logger().info(f"Publicada trayectoria vacia para detener al agente {agent_id}")

        # 5. Solicitar replanificación
        if not self.plan_requested:
            self.plan_requested = True
            self.request_darp(include_visited=True)

def main(args=None):
    rclpy.init(args=args)
    node = PlanningNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()


