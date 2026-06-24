FROM ubuntu:noble-20251013

ARG TARGETPLATFORM
ARG TARGETARCH

ARG QEMU_CPU

SHELL ["/bin/bash", "-c"]

# Upgrade OS
RUN apt-get update -q && \
    DEBIAN_FRONTEND=noninteractive apt-get upgrade -y && \
    apt-get autoclean && \
    apt-get autoremove && \
    rm -rf /var/lib/apt/lists/*

# Install Ubuntu XFCE desktop
RUN apt-get update -q && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
        xfce4 xfce4-session xfce4-terminal supervisor dbus-x11 && \
    apt-get autoclean && \
    apt-get autoremove && \
    rm -rf /var/lib/apt/lists/*

# Add Utility Packages
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
        tigervnc-standalone-server tigervnc-common \
        supervisor wget curl gosu git sudo python3-full python3-pip \
        tini build-essential vim sudo lsb-release locales \
        bash-completion tzdata tmux tar xz-utils gnupg2 lsb-release \ 
        qt6-qpa-plugins xcb xcb-proto libgl1-mesa-dev \
        libavcodec-dev libavformat-dev libswscale-dev && \
    apt-get autoclean && \
    apt-get autoremove && \
    rm -rf /var/lib/apt/lists/*

# noVNC and Websockify
RUN git clone https://github.com/AtsushiSaito/noVNC.git -b add_clipboard_support /usr/lib/novnc
RUN pip install --no-cache-dir --break-system-packages git+https://github.com/novnc/websockify.git@v0.10.0
RUN ln -s /usr/lib/novnc/vnc.html /usr/lib/novnc/index.html

# Set remote resize function enabled by default
RUN sed -i "s/UI.initSetting('resize', 'off');/UI.initSetting('resize', 'remote');/g" /usr/lib/novnc/app/ui.js

# Install Firefox
RUN wget -q https://packages.mozilla.org/apt/repo-signing-key.gpg \
    -O /etc/apt/keyrings/packages.mozilla.org.asc && \
    echo "deb [signed-by=/etc/apt/keyrings/packages.mozilla.org.asc] https://packages.mozilla.org/apt mozilla main" \
    | tee -a /etc/apt/sources.list.d/mozilla-apt.list && \
    echo "Package: *" | tee /etc/apt/preferences.d/mozilla > /dev/null && \
    echo "Pin: origin packages.mozilla.org" | tee -a /etc/apt/preferences.d/mozilla > /dev/null && \
    echo "Pin-Priority: 1000" | tee -a /etc/apt/preferences.d/mozilla > /dev/null && \
    apt-get update -q && \
    apt-get remove -y firefox && \
    apt-get install -y \
    firefox && \
    apt-get autoclean && \
    apt-get autoremove && \
    rm -rf /var/lib/apt/lists/*

# Install VSCodium
RUN wget https://gitlab.com/paulcarroty/vscodium-deb-rpm-repo/raw/master/pub.gpg \
    -O /usr/share/keyrings/vscodium-archive-keyring.asc && \
    echo 'deb [ signed-by=/usr/share/keyrings/vscodium-archive-keyring.asc ] https://paulcarroty.gitlab.io/vscodium-deb-rpm-repo/debs vscodium main' \
    | tee /etc/apt/sources.list.d/vscodium.list && \
    apt-get update -q && \
    apt-get install -y codium && \
    apt-get autoclean && \
    apt-get autoremove && \
    rm -rf /var/lib/apt/lists/*

ENV ROS_DISTRO jazzy
ARG INSTALL_PACKAGE=ros-base

RUN apt-get update -q && \
    apt-get install -y curl gnupg2 lsb-release && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt-get update -q && \
    apt-get install -y ros-${ROS_DISTRO}-${INSTALL_PACKAGE} \
    python3-argcomplete \
    python3-colcon-common-extensions \
    python3-rosdep python3-vcstool && \
    rosdep init && \
    rosdep update && \
    rm -rf /var/lib/apt/lists/*

# Install CoppeliaSim
RUN apt-get update -q && \
    wget https://downloads.coppeliarobotics.com/V4_10_0_rev0/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu24_04.tar.xz && \
    apt-get update -q && \
    apt-get install -y \
    python3-argcomplete && \
    pip install --break-system-packages pyzmq cbor2 && \
    rm -rf /var/lib/apt/lists/*

# Unpacking the CoppeliaSim archive
RUN tar -xvf CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu24_04.tar.xz && \
    rm CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu24_04.tar.xz

COPY coppelia-logo.png /coppelia-logo.png

# Enable apt-get completion after running `apt-get update` in the container
RUN rm /etc/apt/apt.conf.d/docker-clean

COPY ./entrypoint.sh /
ENTRYPOINT [ "/bin/bash", "-c", "/entrypoint.sh" ]

ENV USER ubuntu
ENV PASSWD ubuntu
