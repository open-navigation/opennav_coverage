# ROS 2 Humble FULL — nav2 + Gazebo Classic + TB3 sim (demo'da robotu yürütmek için)
#
# .devcontainer/humble (yalnız opennav build deps) üzerine TAM çalışma ortamı ekler.
# Hedef branch: humble-v2 (demo + coverage_demo_launch.py orada). Açmadan önce host'ta:
#   git checkout humble-v2
#
# Humble sim'i Jazzy'den FARKLI — Gazebo CLASSIC (gz sim DEĞİL):
#   coverage_demo_launch.py → gzserver + libgazebo_ros_init/factory + gazebo_ros
#   spawn_entity.py; model/urdf nav2_bringup/worlds/waffle.model'den (kendi içinde,
#   turtlebot3_gazebo modeli/env GEREKMEZ). gzclient kapalı → robot RViz'de izlenir.
#
# Bu yüzden jazzy-full'daki ros-gz / xacro / nav2-minimal-tb3-sim BURADA YOK; yerine
# gazebo-ros-pkgs (Gazebo Classic) var. F2C v2 yine apt'ten (ros-humble-fields2cover).
ARG ROS_DISTRO=humble
FROM osrf/ros:${ROS_DISTRO}-desktop
ARG ROS_DISTRO

ENV DEBIAN_FRONTEND=noninteractive

# --- Tek apt katmanı (cache'lenir): opennav build deps + tam nav2 + Gazebo Classic ---
# opennav build deps (humble/Dockerfile ile birebir):
#   fields2cover / nav2-util / nav2-core / nav2-behavior-tree / nav2-msgs / tinyxml2-vendor
#   (backported_bt_navigator repo içi kaynak paket → deps'i nav2-behavior-tree + navigation2)
# Çalışma ortamı ek paketleri:
#   navigation2       : nav2_controller/planner/bt_navigator/behavior_tree (opennav_coverage_bt + demo runtime)
#   nav2-bringup      : lifecycle_manager + rviz_launch.py + worlds/waffle.model + urdf/turtlebot3_waffle.urdf
#   gazebo-ros-pkgs   : Gazebo Classic (gzserver) + libgazebo_ros_init/factory + spawn_entity.py
#   robot-state-publisher / rviz2 : desktop imajında zaten var
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake git sudo \
      python3-colcon-common-extensions \
      ros-${ROS_DISTRO}-fields2cover \
      ros-${ROS_DISTRO}-nav2-util \
      ros-${ROS_DISTRO}-nav2-core \
      ros-${ROS_DISTRO}-nav2-behavior-tree \
      ros-${ROS_DISTRO}-nav2-msgs \
      ros-${ROS_DISTRO}-tinyxml2-vendor \
      ros-${ROS_DISTRO}-navigation2 \
      ros-${ROS_DISTRO}-nav2-bringup \
      ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    && rm -rf /var/lib/apt/lists/*

# --- Bind-mount izinleri için root olmayan kullanıcı (humble/Dockerfile ile aynı) ---
# Ubuntu 22.04 (Jammy) tabanında UID/GID 1000 boşta → jazzy'deki rename hilesine gerek yok.
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=${USER_UID}
RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

RUN mkdir -p /ros_ws/src /ros_ws/build /ros_ws/install /ros_ws/log \
    && chown -R ${USER_UID}:${USER_GID} /ros_ws

# --- Her shell'de ROS + workspace overlay'i source et ---
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USERNAME}/.bashrc \
    && echo "[ -f /ros_ws/install/setup.bash ] && source /ros_ws/install/setup.bash" >> /home/${USERNAME}/.bashrc

USER ${USERNAME}
WORKDIR /ros_ws

# ============================================================================
# CONTAINER İÇİNDE KOŞULACAK ADIMLAR (ilk açılışta bir kez)
# ============================================================================
# 1) Tüm opennav paketlerini derle (deps apt'te → hızlı; backported_bt_navigator kaynaktan):
#    colcon build --symlink-install
#    source install/setup.bash
#
# 2) Host'ta bir kez (RViz için):  xhost +local:docker
#
# 3) Robotu sim'de yürüt (headless Gazebo Classic + Nav2 + RViz + otomatik coverage goal):
#    ros2 launch opennav_coverage_demo coverage_demo_launch.py
#
#    Başarı: robot boustrophedon satırlarını RViz'de baştan sona tarar, controller
#    log'unda "Resulting plan has 0 poses in it." ÇIKMAZ.
#    (gzclient kapalı; Gazebo GUI istersen launch'ta start_gazebo_client_cmd'i aç.)
# ============================================================================
