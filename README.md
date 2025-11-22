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
En cualquier carpeta:
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
# Preparar el volumen compartido
sudo chown -R user:user ~/shared_volume
cd ~/shared_volume

# Clonar repositorio
git clone https://github.com/JoseLopez36/MultiRobot-Planning-RMS.git

# Compilar paquete principal de ROS2
cd ~/shared_volume/MultiRobot-Planning-RMS/ros2_ws/
colcon build
source install/setup.bash
```

### 5. Clonar PX4 Autopilot dentro del contenedor
```bash
cd ~/shared_volume
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

# Compilar PX4 SITL
cd ~/shared_volume/PX4-Autopilot/
make px4_sitl
```

## 🎮 Lanzar instancias de PX4 (sin script)

### 1. Iniciar simulación de Gazebo con PX4 Autopilot
```bash
PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500_mod PX4_GZ_WORLD=warehouse ./build/px4_sitl_default/bin/px4 -i 1
```

### 2. Iniciar sucesivas simulaciones
```bash
PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="0,1" PX4_SIM_MODEL=gz_x500_mod PX4_GZ_WORLD=warehouse ./build/px4_sitl_default/bin/px4 -i 2
```

> **Nota**: "warehouse" puede sustituirse por otros mundos disponibles como "default" o "lawn".

### 3. Ejecutar MicroXRCEAgent
```bash
MicroXRCEAgent udp4 -p 8888
```

> **Nota**: Esto conecta los mensajes de PX4 con ROS2, haciendo accesibles dichos mensajes en los nodos del paquete principal de ROS2. Se conecta a todas las instancias de PX4 existentes.

## 🎮 Lanzar instancias de PX4 (con script)

```bash
source ~/shared_volume/MultiRobot-Planning-RMS/ros2_ws/install/setup.bash
~/shared_volume/MultiRobot-Planning-RMS/tools/launch_px4_instances.py 2
```

## 🎮 Lanzar nodos de ROS2

```bash
source ~/shared_volume/MultiRobot-Planning-RMS/ros2_ws/install/setup.bash
ros2 launch multi_robot_planning_rms_pkg run.launch.py
```

> **Nota**: Esto ejecuto todo lo anterior y, además, activa control offboard en cada instancia.

## 🛠️ Herramientas y Scripts

### Scripts Docker
- **`docker/build.sh`**: Compila la imagen Docker con ROS2 Humble, Gazebo Garden y herramientas de desarrollo PX4.
- **`docker/start.sh`**: Inicia el contenedor Docker y crea un volumen compartido entre el host y el contenedor.
- **`docker/kill.sh`**: Detiene y elimina el contenedor Docker en ejecución.

### Herramientas de Lanzamiento
- **`tools/launch_px4_instances.py`**: Script Python que automatiza el lanzamiento de múltiples instancias de PX4. Lee las posiciones iniciales desde `config/px4_vehicles.yaml`, inicia MicroXRCEAgent y los nodos de control offboard para cada instancia. Maneja la limpieza de procesos al recibir Ctrl+C.

## ⚙️ Configuraciones

### Archivos de Configuración Global
- **`docker/config.env`**: Variables de entorno para Docker (nombre del contenedor, imagen, configuración de red y rutas del volumen compartido).
- 
- **`config/px4_vehicles.yaml`**: Define las posiciones iniciales de los vehículos, configuración de PX4 (autostart, modelo, mundo) y parámetros del agente MicroXRCE-DDS (puerto, protocolo).

### Configuraciones ROS2
- **`ros2_ws/src/multi_robot_planning_rms_pkg/config/launch.yaml`**: Configuración de lanzamiento que permite habilitar/deshabilitar nodos y ajustar niveles de log (debug, info, warn, error, fatal).

## 📁 Estructura del Proyecto

- **`config/`**: Archivos de configuración global del proyecto.
- **`docker/`**: Scripts y archivos Dockerfile para el entorno de desarrollo containerizado.
- **`gz_px4/`**: Integración Gazebo-PX4:
  - `models/`: Modelos 3D de drones (x500_mod), entorno (Warehouse), estanterías y objetos móviles.
  - `worlds/`: Archivos de mundo de simulación (warehouse.sdf).
  - `airframes/`: Configuraciones de aeronaves para PX4.
- **`ros2_ws/`**: Workspace de ROS2:
  - `src/multi_robot_planning_rms_msgs/`: Definiciones de mensajes para el proyecto.
  - `src/multi_robot_planning_rms_pkg/`: Paquete principal con nodos del proyecto.
  - `src/px4_msgs/`: Definiciones de mensajes de PX4.
  - `src/px4_ros_com/`: Bridge de comunicación PX4-ROS2 y ejemplos de control offboard.
- **`tools/`**: Scripts de utilidad y automatización.