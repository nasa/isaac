
ARG UBUNTU_VERSION=20.04
ARG REMOTE=ghcr.io/nasa

FROM ${REMOTE}/isaac:latest-ubuntu${UBUNTU_VERSION}

RUN apt-get update \
  && apt-get install -y --no-install-recommends \
    hugin \
    python3-pip \
  && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir --upgrade pip \
  && pip3 install --no-cache-dir snakemake

RUN echo 'source "/src/isaac/devel/setup.bash"\nexport ASTROBEE_CONFIG_DIR="/src/astrobee/src/astrobee/config"' >> "${HOME}/.bashrc"
