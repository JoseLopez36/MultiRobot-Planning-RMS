# MultiRobot-Planning-RMS

<div align="center">
  <img src="https://img.shields.io/badge/ROS2-Humble-blue" alt="ROS2 Humble">
  <img src="https://img.shields.io/badge/Gazebo-Garden-green" alt="Gazebo Garden">
  <img src="https://img.shields.io/badge/PX4-v1.15.0-orange" alt="PX4 v1.15.0">
</div>

## 📋 Introducción
Este repositorio contiene la implementación de un sistema de planificación multi-robot...

## 🔧 Requisitos
- Ubuntu 20.04, 22.04 o 24.04
- Docker

## 🚀 Instalación

### 1. Clonar este repositorio
En cualquier carpeta accesible:
```bash
git clone https://github.com/JoseLopez36/MultiRobot-Planning-RMS.git
```

### 2. Compilar imagen Docker
```bash
/path/to/MultiRobot-Planning-RMS/docker/build.sh
```

### 3. Abrir contenedor
```bash
/path/to/MultiRobot-Planning-RMS/docker/start.sh
```

> **Nota**: Esto creará un volumen compartido para trabajar. Este mismo script se puede utilizar para conectar un terminal a un contenedor en ejecución.

### 4. Clonar y compilar este repositorio dentro del contenedor
Una vez en el contenedor:
```bash
# Clonar repositorio
git clone https://github.com/JoseLopez36/MultiRobot-Planning-RMS.git

# Compilar paquete principal de ROS2
cd ~/shared_volume/MultiRobot-Planning-RMS/ros2_ws/
colcon build
source install/setup.bash

# Compilar mensajes del Plugin UWB-Beacon
cd ~/shared_volume/MultiRobot-Planning-RMS/gz_px4/gz_uwb_beacon_msgs
colcon build
source install/setup.bash

# Compilar Plugin UWB-Beacon
cd ~/shared_volume/MultiRobot-Planning-RMS/gz_px4/gz_uwb_beacon_plugin
colcon build
source install/setup.bash
```

### 5. Clonar PX4 Autopilot dentro del contenedor
```bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot/
git checkout v1.15.0
make clean
make distclean
make submodulesclean
```

### 6. Configurar PX4 Autopilot
```bash
# Copiar modelos
cp -r ~/shared_volume/MultiRobot-Planning-RMS/gz_px4/models ~/shared_volume/PX4-Autopilot/Tools/simulation/gz

# Copiar mundos
cp -r ~/shared_volume/MultiRobot-Planning-RMS/gz_px4/worlds ~/shared_volume/PX4-Autopilot/Tools/simulation/gz

# Copiar configuración de aeronaves
cp -r ~/shared_volume/MultiRobot-Planning-RMS/gz_px4/airframes ~/shared_volume/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix

# Copiar CMakeLists de gz bridge
cp ~/shared_volume/MultiRobot-Planning-RMS/gz_px4/CMakeLists.txt ~/shared_volume/PX4-Autopilot/src/modules/simulation/gz_bridge
```

## 🎮 Uso

### 1. Configurar variables de entorno del Plugin de Gazebo
```bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH:/home/user/shared_volume/MultiRobot-Planning-RMS/gz_px4/gz_uwb_beacon_plugin/install/gz_uwb_beacon_plugin/lib
```

> **Importante**: Esto es necesario para que Gazebo pueda encontrar el Plugin.

### 2. Iniciar simulación de Gazebo con PX4 Autopilot
```bash
cd ~/shared_volume/PX4-Autopilot/
make px4_sitl gz_x500_mod_warehouse
```

> **Nota**: "warehouse" puede sustituirse por otros mundos disponibles como "default" o "lawn".

### 3. Ejecutar MicroXRCEAgent
```bash
MicroXRCEAgent udp4 -p 8888
```

> **Nota**: Esto conecta los mensajes de PX4 con ROS2, haciendo accesibles dichos mensajes en los nodos del paquete principal de ROS2.

### 4. Iniciar el paquete de localización EIF
```bash
cd ~/shared_volume/MultiRobot-Planning-RMS/ros2_ws/
source install/setup.bash
ros2 launch multi_robot_planning_rms_pkg run.launch.py
```

### 5. (Opcional) Iniciar RViz2
```bash
cd ~/shared_volume/MultiRobot-Planning-RMS/ros2_ws/
source install/setup.bash
ros2 launch multi_robot_planning_rms_pkg rviz.launch.py
```
