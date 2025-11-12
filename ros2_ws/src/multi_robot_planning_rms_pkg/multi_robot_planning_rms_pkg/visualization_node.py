#!/usr/bin/env python

import rclpy
import numpy as np
from rclpy.node import Node
from visualization_msgs.msg import Marker

class VisualizationNode(Node):

    def __init__(self):
        super().__init__('visualization_node')

        # Log
        self.get_logger().info("Iniciando nodo de visualizacion...")

        # Crear suscriptores

        # Crear publicadores

        # Obtener parámetros del archivo de configuración
        update_rate = self.declare_parameter('update_rate', 20.0).value

        # Crear timer para la actualización de la visualización
        self.dt = 1.0 / update_rate
        self.visualization_timer = self.create_timer(self.dt, self.update)
    
        # Log
        self.get_logger().info("Nodo de visualizacion iniciado")

    def update(self):
        # Nada que hacer
        pass

def main(args=None):
    rclpy.init(args=args)
    visualization_node = VisualizationNode()
    rclpy.spin(visualization_node)
    visualization_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()