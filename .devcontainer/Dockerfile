FROM osrf/ros:humble-desktop

SHELL ["/bin/bash", "-c"]

ENV RCUTILS_COLORIZED_OUTPUT=1

# Install dependencies
RUN apt update && \
    apt install -y \
    build-essential \
    cmake \
    git \
    curl lsb-release gnupg \
    libbullet-dev \
    python3-colcon-common-extensions \
    python3-flake8 \
    python3-pip \
    python3-pytest-cov \
    python3-rosdep \
    python3-setuptools \
    python3-vcstool \
    python3-colcon-clean \
    wget \
    clang-format

# Install Python packages
RUN python3 -m pip install -U \
    argcomplete \
    flake8-blind-except \
    flake8-builtins \
    flake8-class-newline \
    flake8-comprehensions \
    flake8-deprecated \
    flake8-docstrings \
    flake8-import-order \
    flake8-quotes \
    pytest-repeat \
    pytest-rerunfailures \
    pytest \
    kconfiglib \
    jsonschema \
    jinja2 \
    pyros-genmsg \
    future \
    numpy==1.22 \
    quaternionic


# Install Gazebo Harmonic - Default Gazebo version in ROS2 Humble is Fortress
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN apt-get update
RUN apt-get install -y ros-humble-ros-gzharmonic
RUN apt install -y \
    ros-humble-turtle-tf2-py \
    ros-humble-tf2-tools \
    ros-dev-tools \
    ros-humble-plotjuggler-ros

RUN apt-get update
RUN apt-get upgrade -y

# Create developer user with root-like privileges
RUN useradd -m -d /home/developer -s /bin/bash -u 1001 -G sudo developer && \
    echo 'developer:developer' | chpasswd && \
    echo "developer ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/developer && \
    chmod 0440 /etc/sudoers.d/developer
USER developer

# Install PX4-Autopilot
RUN git clone https://github.com/PX4/PX4-Autopilot.git ~/PX4-Autopilot --recursive -b release/1.15 && \
    echo developer | sudo -S bash /home/developer/PX4-Autopilot/Tools/setup/ubuntu.sh

# Copy workspace
COPY . /home/developer/ros2_ws
WORKDIR /home/developer/ros2_ws

# Add ROS2 setup to bashrc
RUN echo ". /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc \
    && echo ". /home/developer/ros2_ws/install/setup.bash" >> ~/.bashrc

CMD [ "sleep", "infinity" ]
