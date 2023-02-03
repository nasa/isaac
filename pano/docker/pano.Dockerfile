
ARG UBUNTU_VERSION=20.04
ARG REMOTE=ghcr.io/nasa

FROM ${REMOTE}/isaac:latest-ubuntu${UBUNTU_VERSION}

# default-jre: Java runtime needed for minifying Pannellum web files
# hugin: pano stitching tools (and hsi Python interface)
# libvips-tools: convert images to multires, zoomable in OpenSeaDragon
# python3-pip: for installing Python packages later in this Dockerfile
RUN apt-get update \
  && apt-get install -y --no-install-recommends \
    default-jre \
    hugin \
    libvips-tools \
    python3-pip \
  && rm -rf /var/lib/apt/lists/*

# pandas: pulled in as pyshtools dependency but install breaks if not mentioned explicitly (?)
# pyshtools: used during Pannellum multires generation
# snakemake: modern build system based on Python, manages stitching workflows
RUN pip3 install --no-cache-dir --upgrade pip \
  && pip3 install --no-cache-dir \
    pandas \
    pyshtools \
    snakemake

# pannellum: library for viewing/navigating panorama tours
RUN mkdir -p /opt \
  && cd /opt \
  && git clone --quiet --depth 1 --branch standalone_load_event --single-branch --no-tags https://github.com/trey0/pannellum.git \
  && cd /opt/pannellum/utils/build \
  && python3 build.py

# openseadragon: library for viewing high-res SciCam source images efficiently with zoom
RUN cd /tmp \
  && wget --quiet https://github.com/openseadragon/openseadragon/releases/download/v4.0.0/openseadragon-bin-4.0.0.tar.gz \
  && tar xfz openseadragon-bin-4.0.0.tar.gz \
  && rm openseadragon-bin-4.0.0.tar.gz \
  && mv openseadragon-bin-4.0.0 /opt/openseadragon

RUN echo 'source "/src/isaac/devel/setup.bash"\nexport ASTROBEE_CONFIG_DIR="/src/astrobee/src/astrobee/config"' >> "${HOME}/.bashrc"

# Enables rosrun for pano packages. Can likely take this out
# once new pano folder is merged into develop and official docker
# images are updated.
RUN echo 'export ROS_PACKAGE_PATH="${ROS_PACKAGE_PATH}:/src/isaac/src/pano/pano_stitch::/src/isaac/src/pano/pano_view"' >> "${HOME}/.bashrc"
