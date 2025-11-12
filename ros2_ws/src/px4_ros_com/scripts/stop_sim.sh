#!/bin/bash
cd ~/ws_sensor_combined

echo "Deteniendo ROS2, PX4 y Gazebo..."
pkill -f ros2
pkill -f MicroXRCEAgent
pkill -f px4
pkill -f gazebo
pkill -f gz
echo "Todos los procesos cerrados."
