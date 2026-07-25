# ROS 2 Jazzy (LTS) + Fields2Cover v2 — izole geliştirme ortamı
#
# Feature/umutc/fields2cover2-jazzy branch'ini hedefler: nav2_util tabanlı
# (nav2_ros_common yok, C++17, jazzy'nin orijinal API'si).
#
# F2C v2, ROS apt reposundan (ros-jazzy-fields2cover) kurulur; humble'daki
# ros-humble-fields2cover = 2.0.0 ile aynı yöntem. Paket bulunamazsa
# kaynak build'e dönülmeli (Dockerfile'ı ../rolling/Dockerfile'a bakarak düzenle).
ARG ROS_DISTRO=jazzy
FROM osrf/ros:${ROS_DISTRO}-desktop
ARG ROS_DISTRO

ENV DEBIAN_FRONTEND=noninteractive

# --- Tüm bağımlılıklar tek apt katmanında (cache'lenir) ---
# osrf/ros:jazzy-desktop zaten rclcpp / rclcpp_action / rclcpp_lifecycle /
# rclcpp_components / std_msgs / visualization_msgs / nav_msgs / geometry_msgs /
# builtin_interfaces / tf2_ros / rosidl_default_generators içerir.
#
#  - ros-jazzy-fields2cover : F2C v2 (CMake config + lib + GDAL/Eigen/TBB/GEOS
#    bağımlılıklarını kendisi çeker). package.xml'deki <depend>fields2cover</depend>
#    rosdep ile buna çözülür.
#  - ros-jazzy-nav2-util / nav2-msgs : opennav_coverage'ın ROS bağımlılıkları
#    (jazzy'de nav2_ros_common yok, nav2_util kullanılıyor).
#  - ros-jazzy-tinyxml2-vendor : row_coverage'ın find_package(TinyXML2) +
#    tinyxml2::tinyxml2 ihtiyacı (Ubuntu Noble'daki libtinyxml2-dev CMake config
#    içermiyor).
#  - libgdal-dev : opennav CMakeLists'teki find_package(GDAL) için (CMake config).
#  - libtinyxml2-dev : FindTinyXML2 modülünün bulduğu sistem header+lib.
#  - build-essential/cmake/git/sudo : workspace paketlerini colcon ile derlemek için.
#
# apt-F2C ile çıkan 2 ek build sorunu — CMakeLists'te çözüldü (humble'da doğrulandı,
# jazzy'de de aynı):
#   (1) F2C target adı apt'te Fields2Cover::Fields2Cover (namespaced), kaynakta
#       Fields2Cover → if(TARGET ...) ile seçilir.
#   (2) apt F2C include'ları /opt/ros/jazzy/include'da → -Wshadow -Werror kırar →
#       Fields2Cover_INCLUDE_DIR SYSTEM ile işaretlendi.
#
# Yeni bir bağımlılık gerekirse bu satıra ekle ve "Rebuild Container".
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake git sudo \
      libgdal-dev libtinyxml2-dev \
      ros-${ROS_DISTRO}-fields2cover \
      ros-${ROS_DISTRO}-nav2-util \
      ros-${ROS_DISTRO}-nav2-msgs \
      ros-${ROS_DISTRO}-tinyxml2-vendor \
    && rm -rf /var/lib/apt/lists/*

# --- Bind-mount dosya izinleri için root olmayan kullanıcı ---
# osrf/ros:jazzy-desktop (Ubuntu Noble tabanlı) UID/GID 1000'i zaten "ubuntu"
# kullanıcısına vermiş durumda → groupadd/useradd çakışır. Varsa yeniden adlandır.
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

# --- colcon workspace kökünü kullanıcıya ait yap ---
RUN mkdir -p /ros_ws && chown ${USER_UID}:${USER_GID} /ros_ws

# --- Her shell'de ROS + workspace overlay'i source et ---
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USERNAME}/.bashrc \
    && echo "[ -f /ros_ws/install/setup.bash ] && source /ros_ws/install/setup.bash" >> /home/${USERNAME}/.bashrc

USER ${USERNAME}
