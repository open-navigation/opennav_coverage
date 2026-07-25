# ROS 2 Humble + Fields2Cover v2 — izole geliştirme ortamı
#
# Bu konfigürasyon ../Dockerfile (rolling/main) ile AYNI repo checkout'unu
# kullanır ama farklı bir branch hedefler: Feature/umutc/fields2cover2
# (fields2cover2-main DEĞİL). O branch'e geçmeden bu devcontainer'ı açmayın:
#   git checkout Feature/umutc/fields2cover2
#
# Humble branch main'den farklı: nav2_ros_common YOK; nav2_core +
# nav2_behavior_tree var (backported_bt_navigator paketi repo içinde kaynak
# olarak geliyor, apt'ten kurulmuyor). C++17 (CMakeLists'lerde sabit).
#
# Fields2Cover v2 humble apt deposunda mevcut (ros-humble-fields2cover) —
# rolling/jazzy'deki gibi kaynaktan derlemeye gerek yok.
ARG ROS_DISTRO=humble
FROM osrf/ros:${ROS_DISTRO}-desktop
ARG ROS_DISTRO

ENV DEBIAN_FRONTEND=noninteractive

# --- Tüm sistem bağımlılıkları tek apt katmanında (cache'lenir) ---
# osrf/ros:humble-desktop zaten rclcpp / rclcpp_action / rclcpp_lifecycle /
# std_msgs / nav_msgs / geometry_msgs / builtin_interfaces / tf2_ros /
# pluginlib / std_srvs içerir.
#
#  - ros-${ROS_DISTRO}-fields2cover : F2C v2, apt'ten (kaynaktan derleme yok).
#  - ros-${ROS_DISTRO}-nav2-util / nav2-core / nav2-behavior-tree / nav2-msgs :
#    opennav_coverage + backported_bt_navigator'ın ROS bağımlılıkları.
#  - ros-${ROS_DISTRO}-tinyxml2-vendor : row_coverage'ın find_package(TinyXML2) +
#    tinyxml2::tinyxml2 ihtiyacı (Ubuntu libtinyxml2-dev CMake config içermez).
#  - build-essential/cmake/git/sudo : workspace paketlerini colcon ile derlemek
#    için.
#
# Yeni bir bağımlılık gerekirse bu satıra ekle ve "Rebuild Container".
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake git sudo \
      ros-${ROS_DISTRO}-fields2cover \
      ros-${ROS_DISTRO}-nav2-util \
      ros-${ROS_DISTRO}-nav2-core \
      ros-${ROS_DISTRO}-nav2-behavior-tree \
      ros-${ROS_DISTRO}-nav2-msgs \
      ros-${ROS_DISTRO}-tinyxml2-vendor \
    && rm -rf /var/lib/apt/lists/*

# --- Bind-mount dosya izinleri için root olmayan kullanıcı ---
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=${USER_UID}
RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

# --- colcon workspace kökünü kullanıcıya ait yap ---
# /ros_ws'i Docker mount sırasında root oluşturur; ros kullanıcısı build/install/log
# yazabilsin diye sahipliği veriyoruz (yoksa 'sudo colcon build'e mecbur kalınır).
RUN mkdir -p /ros_ws && chown ${USER_UID}:${USER_GID} /ros_ws

# --- Her shell'de ROS + workspace overlay'i source et ---
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USERNAME}/.bashrc \
    && echo "[ -f /ros_ws/install/setup.bash ] && source /ros_ws/install/setup.bash" >> /home/${USERNAME}/.bashrc

USER ${USERNAME}
