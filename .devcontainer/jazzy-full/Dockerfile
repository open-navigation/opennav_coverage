# ROS 2 Jazzy FULL — nav2 + Gazebo + TB3 sim (demo'da robotu yürütmek için)
#
# .devcontainer/jazzy (yalnız opennav build deps) üzerine TAM çalışma ortamı ekler:
#   - navigation2 (nav2_core dahil → opennav_coverage_bt/navigator derlenir)
#   - nav2-bringup (lifecycle_manager, rviz_launch, params)
#   - nav2-minimal-tb3-sim + ros-gz (Gazebo Harmonic) → coverage_demo_launch.py
#   - xacro, robot-state-publisher
#
# Jazzy'de bunların HEPSİ apt'te var → rolling-full'un aksine kaynaktan build YOK.
# F2C v2 yine apt'ten (ros-jazzy-fields2cover), jazzy/Dockerfile ile aynı yöntem.
ARG ROS_DISTRO=jazzy
FROM osrf/ros:${ROS_DISTRO}-desktop
ARG ROS_DISTRO

ENV DEBIAN_FRONTEND=noninteractive

# --- Tek apt katmanı (cache'lenir): opennav build deps + tam nav2 + sim ---
# opennav build deps (jazzy/Dockerfile ile birebir):
#   fields2cover / nav2-util / nav2-msgs / tinyxml2-vendor / libgdal-dev / libtinyxml2-dev
# Çalışma ortamı ek paketleri:
#   navigation2          : nav2_core + controller/planner/bt_navigator/behavior_tree (opennav_coverage_bt için)
#   nav2-bringup         : lifecycle_manager + rviz_launch.py + demo'nun kullandığı bringup parçaları
#   nav2-minimal-tb3-sim : Waffle SDF/URDF + spawn_tb3.launch.py + world modelleri
#   ros-gz               : Gazebo Harmonic + ros_gz_sim (gz_sim.launch.py) köprüsü
#   xacro                : demo world_sdf_xacro / robot_sdf için
#   rviz2                : desktop imajında zaten var (path görselleştirme)
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake git sudo \
      python3-colcon-common-extensions \
      libgdal-dev libtinyxml2-dev \
      ros-${ROS_DISTRO}-fields2cover \
      ros-${ROS_DISTRO}-nav2-util \
      ros-${ROS_DISTRO}-nav2-msgs \
      ros-${ROS_DISTRO}-tinyxml2-vendor \
      ros-${ROS_DISTRO}-navigation2 \
      ros-${ROS_DISTRO}-nav2-bringup \
      ros-${ROS_DISTRO}-nav2-minimal-tb3-sim \
      ros-${ROS_DISTRO}-ros-gz \
      ros-${ROS_DISTRO}-xacro \
    && rm -rf /var/lib/apt/lists/*

# --- Bind-mount izinleri için root olmayan kullanıcı (jazzy/Dockerfile ile aynı) ---
# osrf/ros:jazzy-desktop (Ubuntu Noble) UID/GID 1000'i "ubuntu"ya vermiş → çakışmayı
# yeniden adlandırarak çöz.
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=${USER_UID}
RUN if getent passwd ${USER_UID} >/dev/null; then \
      existing_user=$(getent passwd ${USER_UID} | cut -d: -f1); \
      existing_group=$(getent group ${USER_GID} | cut -d: -f1); \
      [ "${existing_group}" != "${USERNAME}" ] && groupmod -n ${USERNAME} ${existing_group}; \
      [ "${existing_user}" != "${USERNAME}" ] && usermod -l ${USERNAME} -d /home/${USERNAME} -m ${existing_user}; \
    else \
      groupadd --gid ${USER_GID} ${USERNAME} \
      && useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME}; \
    fi \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

RUN mkdir -p /ros_ws/src /ros_ws/build /ros_ws/install /ros_ws/log \
    && chown -R ${USER_UID}:${USER_GID} /ros_ws

# --- Her shell'de ROS + workspace overlay + sim ortam değişkenleri ---
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USERNAME}/.bashrc \
    && echo "[ -f /ros_ws/install/setup.bash ] && source /ros_ws/install/setup.bash" >> /home/${USERNAME}/.bashrc

USER ${USERNAME}
WORKDIR /ros_ws

# ============================================================================
# CONTAINER İÇİNDE KOŞULACAK ADIMLAR (ilk açılışta bir kez)
# ============================================================================
# 1) Tüm opennav paketlerini derle (deps apt'te → hepsi hızlı, kaynak build yok):
#    colcon build --symlink-install
#    source install/setup.bash
#
# 2) Host'ta bir kez (RViz/Gazebo GUI için):  xhost +local:docker
#
# 3) Robotu sim'de yürüt (Gazebo + Nav2 + RViz + otomatik coverage goal):
#    ros2 launch opennav_coverage_demo coverage_demo_launch.py
#
#    Başarı: robot boustrophedon satırlarını tarar, controller log'unda
#    "Resulting plan has 0 poses in it." ÇIKMAZ.
# ============================================================================
