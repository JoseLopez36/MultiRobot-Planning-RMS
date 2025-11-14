#!/bin/bash
# Obtener el archivo de configuración de entorno
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
source "$SCRIPT_DIR/config.env"

BASE_IMAGE="mzahana/px4-dev-simulation-ubuntu22"

# Asegurarse de que la imagen base de PX4 + ROS2 exista localmente
if ! docker image inspect "$BASE_IMAGE" > /dev/null 2>&1; then
    echo "La imagen base '$BASE_IMAGE' no existe localmente. Construyéndola desde 'px4_ros2_humble'..."
    if cd "$SCRIPT_DIR/px4_ros2_humble/docker" && make px4-dev-simulation-ubuntu22; then
        echo "Imagen base '$BASE_IMAGE' construida correctamente."
    else
        echo "Error al construir la imagen base '$BASE_IMAGE'."
        exit 1
    fi
    cd "$SCRIPT_DIR"
fi

# Construir la imagen de Docker
echo "Construyendo la imagen de Docker '$IMAGE_NAME'..."
cd "$SCRIPT_DIR"
if docker build -t "$IMAGE_NAME" .; then
    echo "Imagen '$IMAGE_NAME' construida correctamente"
else
    echo "Error al construir la imagen de Docker"
    exit 1
fi