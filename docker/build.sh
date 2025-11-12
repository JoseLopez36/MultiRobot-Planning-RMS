#!/bin/bash
# Obtener el archivo de configuración de entorno
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
source "$SCRIPT_DIR/config.env"

# Construir la imagen de Docker
echo "Construyendo la imagen de Docker '$IMAGE_NAME'..."
cd $SCRIPT_DIR/px4_ros2_humble/docker
if docker build -t $IMAGE_NAME --file Dockerfile_simulation-ubuntu22 .; then
    echo "Imagen '$IMAGE_NAME' construida correctamente"
else
    echo "Error al construir la imagen de Docker"
    exit 1
fi