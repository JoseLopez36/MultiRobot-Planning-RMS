#!/bin/bash
# Obtener el archivo de configuración de entorno
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
source "$SCRIPT_DIR/config.env"

# Obtener el contenedor en ejecución
RUNNING_CONTAINER=$(docker ps -aq -f "name=${CONTAINER_NAME}")

# Verificar si el contenedor está en ejecución y detenerlo
if [ -n "$RUNNING_CONTAINER" ]; then
    echo "Deteniendo contenedor: $CONTAINER_NAME"
    docker stop $RUNNING_CONTAINER
    echo "Eliminando contenedor: $CONTAINER_NAME"
    docker rm $RUNNING_CONTAINER
else
    echo "No hay contenedores en ejecucion con el nombre: $CONTAINER_NAME"
fi
