# Escolha a imagem base
FROM ubuntu:20.04

# Definir a variável de ambiente não interativa
ENV DEBIAN_FRONTEND=noninteractive

# Atualizar pacotes e instalar dependências básicas
RUN apt-get update && apt-get install -y \
    sudo \
    curl \
    wget \
    git \
    lsb-release \
    gnupg2 \
    build-essential \
    python3-pip

RUN apt-get upgrade

# Adicionar o repositório do ROS Noetic e a chave GPG
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
    && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
    && apt-get update \
    && apt-get install -y ros-noetic-desktop-full

# Instalar rosdep
RUN apt-get install -y python3-rosdep

ENV GAZEBO_MODEL_PATH="/usr/share/gazebo-11/models:~/catkin_ws/src/cbr_2024/cbr/models"

# Inicializar rosdep
RUN rosdep init && rosdep update

# Atualizar pacotes e instalar dependências básicas
RUN apt-get update && apt-get install -y \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    libopencv-dev \
    python3-opencv \
    && rm -rf /var/lib/apt/lists/*

# Instalar vcstool via pip
RUN pip3 install vcstool

# Criar e configurar o workspace do Catkin
RUN mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/ && /bin/bash -c "source /opt/ros/noetic/setup.bash; catkin_make"

# Testar a conexão SSH
RUN ssh -T git@github.com || true

# Clonar o repositório do Swarm in Blocks
RUN cd ~/catkin_ws/src \
    && git clone https://github.com/Grupo-SEMEAR-USP/swarm_in_blocks.git ~/catkin_ws/src/swarm_in_blocks

# Clonar os repositórios um por vez
RUN cd ~/catkin_ws/src \
    && git clone --depth 1 https://github.com/CopterExpress/clover.git
RUN cd ~/catkin_ws/src \
    && git clone --depth 1 https://github.com/CopterExpress/ros_led.git
RUN cd ~/catkin_ws/src \
    && git clone --depth 1 https://github.com/ethz-asl/mav_comm.git

# Instalar o pacote de cores para bash
RUN apt-get update && apt-get install -y \
    bash-completion \
    nano

# Habilitar cores no bash e no comando ls
RUN echo "export LS_OPTIONS='--color=auto'" >> ~/.bashrc \
    && echo "eval \"\$(dircolors)\"" >> ~/.bashrc \
    && echo "alias ls='ls \$LS_OPTIONS'" >> ~/.bashrc \
    && echo "alias ll='ls \$LS_OPTIONS -l'" >> ~/.bashrc \
    && echo "alias l='ls \$LS_OPTIONS -lA'" >> ~/.bashrc \
    && echo "PS1='\\[\\033[01;34m\\]\\u@\\h \\[\\033[01;32m\\]\\w\\[\\033[00m\\]\\$ '" >> ~/.bashrc

# Instalar dependências Python adicionais
RUN sudo /usr/bin/python3 -m pip install -r ~/catkin_ws/src/clover/clover/requirements.txt

# Clonar os fontes do PX4 e configurar links simbólicos
RUN git clone --recursive --depth 1 --branch v1.12.3 https://github.com/PX4/PX4-Autopilot.git ~/PX4-Autopilot \
    && ln -s ~/PX4-Autopilot ~/catkin_ws/src/ \
    && ln -s ~/PX4-Autopilot/Tools/sitl_gazebo ~/catkin_ws/src/ \
    && ln -s ~/PX4-Autopilot/mavlink ~/catkin_ws/src/

# Instalar as dependências do PX4 com o script ubuntu.sh
RUN cd ~/catkin_ws/src/PX4-Autopilot/Tools/setup && sudo ./ubuntu.sh --no-nuttx

# Instalar mais pacotes Python necessários
RUN pip3 install --user toml

# Adicionar o airframe do Clover ao PX4
RUN ln -s ~/catkin_ws/src/clover/clover_simulation/airframes/* ~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/

# Instalar os datasets do geographiclib necessários para o Mavros
RUN sudo apt-get install -y ros-noetic-mavros ros-noetic-mavros-extras ros-noetic-mavros-msgs
RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh \
    && sudo bash ./install_geographiclib_datasets.sh \
    && mkdir -p /usr/share/GeographicLib/geoids/
    
# Instalar pygeodesy
RUN pip3 install pygeodesy

RUN pip uninstall opencv-contrib-python opencv-python
RUN pip install opencv-contrib-python==4.6.0.66
RUN pip install colorful
RUN pip install pyzbar
    
# Compilar o workspace
RUN cd ~/catkin_ws && /bin/bash -c "source ~/catkin_ws/devel/setup.bash; catkin_make"

# Configurar ambiente ROS
RUN echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

# Comando padrão
CMD ["/bin/bash"]
