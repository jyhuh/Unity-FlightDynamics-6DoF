# Licensed under the Apache License, Version 2.0.
# Container 1: Independent PX4-Autopilot SITL Node (Ubuntu 22.04 LTS)
# Eliminates macOS host Python 3.14 / CMake 4.x toolchain incompatibilities.
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    ninja-build \
    git \
    ca-certificates \
    pkg-config \
    libeigen3-dev \
    libxml2-dev \
    libxml2-utils \
    python3 \
    python3-dev \
    python3-pip \
    python3-venv \
    bc \
    xxd \
    file \
    rsync \
    unzip \
    zip \
    iproute2 \
    iputils-ping \
    netcat-openbsd \
    && rm -rf /var/lib/apt/lists/*

# Install full PX4 v1.15 Python build & uXRCE-DDS code generation requirements
RUN pip3 install --no-cache-dir --upgrade pip setuptools wheel && \
    pip3 install --no-cache-dir \
    argcomplete \
    cerberus \
    "empy==3.3.4" \
    future \
    jinja2 \
    jsonschema \
    kconfiglib \
    lxml \
    numpy \
    nunavut \
    packaging \
    pkgconfig \
    psutil \
    pygments \
    pymavlink \
    pyros-genmsg \
    pyserial \
    pyulog \
    pyyaml \
    requests \
    six \
    sympy \
    toml

WORKDIR /workspace

COPY entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["run"]
