FROM ros:jazzy-ros-base-noble

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git sudo curl ca-certificates bash-completion \
    python3-colcon-common-extensions python3-rosdep python3-pytest \
    python3-serial python3-numpy python3-dev libeigen3-dev \
    mesa-utils x11-apps libgl1-mesa-dri \
    ros-jazzy-desktop ros-jazzy-moveit \
    ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
    ros-jazzy-ros-gz ros-jazzy-gz-ros2-control \
    ros-jazzy-rtabmap-ros ros-jazzy-pcl-ros ros-jazzy-pcl-conversions \
    && rm -rf /var/lib/apt/lists/*

# Phoenix 6 C++ SDK, including the phoenix6 CMake package.
ARG PHOENIX_YEAR=2026
RUN curl -fsSL --compressed https://deb.ctr-electronics.com/ctr-pubkey.gpg \
        -o /usr/share/keyrings/ctr-pubkey.gpg \
    && curl -fsSL --compressed "https://deb.ctr-electronics.com/ctr${PHOENIX_YEAR}.list" \
        -o /etc/apt/sources.list.d/ctr.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends phoenix6 \
    && ldconfig \
    && rm -rf /var/lib/apt/lists/*

# Ubuntu Noble may already reserve UID 1000 for the ubuntu account.
RUN if id ubuntu &>/dev/null; then userdel -r ubuntu; fi \
    && groupadd --gid 1000 vscode \
    && useradd --uid 1000 --gid vscode --create-home --shell /bin/bash vscode \
    && echo 'vscode ALL=(ALL) NOPASSWD:ALL' > /etc/sudoers.d/vscode \
    && chmod 0440 /etc/sudoers.d/vscode
RUN printf '\nsource /opt/ros/jazzy/setup.bash\nif [ -f /workspaces/urc-robot/install/setup.bash ]; then source /workspaces/urc-robot/install/setup.bash; fi\n' >> /home/vscode/.bashrc
ENV DEBIAN_FRONTEND=dialog
USER vscode
WORKDIR /workspaces/urc-robot
CMD ["sleep", "infinity"]
