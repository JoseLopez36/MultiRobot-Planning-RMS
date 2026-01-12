#
# MultiRobot-Planning-RMS project Dockerfile
# Builds on top of the PX4 + ROS 2 Humble simulation image and adds
# project-specific dependencies.
#

FROM mzahana/px4-dev-simulation-ubuntu22

LABEL maintainer="MultiRobot-Planning-RMS"

# Ensure we are root for package installation (the base image may switch users)
USER root

# Install additional system dependencies
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
        libxcb-cursor0 \
        ros-humble-vision-msgs \
        tmux \
    && apt-get clean && apt-get autoremove -y && \
    rm -rf /var/lib/apt/lists/*

# Install Python dependencies for this project
RUN pip3 install --no-cache-dir PySide6

# Install DARP dependencies
RUN pip3 install --no-cache-dir --upgrade \
    numpy==1.26.4 \
    scipy \
    scikit-learn \
    opencv-python \
    pygame \
    numba

# Disable pygame audio warnings
ENV SDL_AUDIODRIVER=dummy

# Install tmux
RUN apt-get update && apt-get install -y tmux

# Default workdir matches the base image conventions
WORKDIR /home/user

CMD ["/bin/bash"]


