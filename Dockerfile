# ============================================================
# Isaac Sim 5.1.0 + Isaac Lab v2.3.2
# Minimal image for running robot simulation scripts.
# ============================================================
FROM nvcr.io/nvidia/isaac-sim:5.1.0

# 5.1 base image runs as non-root by default; apt-get needs root
USER root

ENV ACCEPT_EULA=Y \
    DEBIAN_FRONTEND=noninteractive \
    TERM=xterm

# System deps
RUN apt-get update && apt-get install -y --no-install-recommends \
    git curl ffmpeg libgl1 libglib2.0-0 \
    pcmanfm xdg-utils \
    && rm -rf /var/lib/apt/lists/*

# Isaac Lab v2.3.2 (provides BaseSample, task framework, RL utilities)
RUN git clone --depth 1 --branch v2.3.2 \
        https://github.com/isaac-sim/IsaacLab.git /opt/isaaclab && \
    ln -s /isaac-sim /opt/isaaclab/_isaac_sim && \
    cd /opt/isaaclab && \
    ./isaaclab.sh --install && \
    /isaac-sim/python.sh -m pip install --no-build-isolation \
        -e /opt/isaaclab/source/isaaclab 2>/dev/null || true

WORKDIR /workspace
ENV PYTHONPATH="/workspace:${PYTHONPATH:-}"

ENTRYPOINT []
CMD ["/bin/bash"]
