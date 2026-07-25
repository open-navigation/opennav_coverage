# ROS 2 Rolling (Lyrical) + Fields2Cover v2 — izole geliştirme ortamı
#
# main branch'i hedefler (humble değil): C++20, nav2_util + nav2_ros_common
# birlikte, behaviortree_cpp (v3 değil).
#
# F2C v2 KAYNAKTAN derlenir (apt'te ros-rolling-fields2cover olup olmadığı
# doğrulanamadığı için humble'daki apt kısayolu burada kullanılmıyor).
# Fields2Cover/CMakeLists.txt incelendi: TinyXML2/GDAL>=3.0/Eigen3/geos_c
# sistemde bulunmalı; or-tools/steering_functions/matplot/nlohmann_json
# F2C'nin kendi CMake'i (FetchContent ile, build sırasında internet ister)
# tarafından otomatik indirilip statik gömülüyor — elle kurulmaz.
ARG ROS_DISTRO=rolling
FROM osrf/ros:${ROS_DISTRO}-desktop

ENV DEBIAN_FRONTEND=noninteractive

# --- Tüm sistem bağımlılıkları tek apt katmanında (cache'lenir) ---
# osrf/ros:rolling-desktop zaten rclcpp / rclcpp_action / rclcpp_lifecycle /
# rclcpp_components / std_msgs / visualization_msgs / nav_msgs / geometry_msgs /
# builtin_interfaces / tf2_ros / rosidl_default_generators içerir.
#
#  - libgdal-dev / libeigen3-dev / libgeos-dev / libtinyxml2-dev : F2C'nin
#    CMakeLists'indeki find_package(GDAL/Eigen3/TinyXML2) + -lgeos_c için.
#  - ca-certificates / wget : F2C'nin build sırasında FetchContent ile
#    or-tools/steering_functions/matplot/json çekmesi için.
#  - ros-rolling-tinyxml2-vendor : row_coverage'ın find_package(TinyXML2) +
#    tinyxml2::tinyxml2 ihtiyacı (Ubuntu libtinyxml2-dev CMake config içermez).
#  - build-essential/cmake/git/sudo : workspace paketlerini colcon ile derlemek
#    ve F2C'yi kaynaktan kurmak için.
#  - python3-colcon-common-extensions : nav2_util/nav2_msgs/nav2_ros_common/nav2_common'ı
#    aşağıda kaynaktan derlemek için (bu paketler rolling rosdistro release havuzunda
#    henüz yok, apt'te bulunamıyor).
#  - ros-rolling-angles/backward-ros/bond/bondcpp/geographic-msgs/map-msgs/
#    unique-identifier-msgs/rcl-interfaces/tf2-msgs/ament-index-cpp : yukarıdaki
#    nav2 paketlerinin ROS bağımlılıkları. rosdep yerine elle listelendi çünkü
#    rosdep'in rolling→Ubuntu eşlemesi şu an "resolute"yu hedefliyor (bizim image
#    "noble" tabanlı) ve "No definition ... for OS version [noble]" hatası veriyor —
#    paketlerin kendisi apt'te mevcut, sadece rosdep key çözümlemesi bozuk.
#
# Yeni bir bağımlılık gerekirse bu satıra ekle ve "Rebuild Container".
# "apt-get upgrade" şart: rolling apt deposu sürekli ileri akıyor ama
# osrf/ros:rolling-desktop imajı eski bir snapshot — bugün taze kurulan
# ros-rolling-geographic-msgs gibi paketler, imajın içinde önceden gömülü
# olan rosidl/rclcpp paketlerinden daha yeni bir ABI'ye karşı derlenmiş
# olabiliyor (örn. "rosidl_buffer" target'ı eksik hatası). Upgrade ile
# tüm ros-rolling-* paketleri aynı snapshot'a eşitlenir.
RUN apt-get update && apt-get dist-upgrade -y \
    && apt-get install -y --no-install-recommends \
      build-essential cmake git sudo ca-certificates wget \
      libgdal-dev libeigen3-dev libgeos-dev libtinyxml2-dev \
      ros-rolling-tinyxml2-vendor \
      python3-colcon-common-extensions \
      ros-rolling-angles ros-rolling-backward-ros ros-rolling-bond \
      ros-rolling-bondcpp ros-rolling-geographic-msgs ros-rolling-map-msgs \
      ros-rolling-unique-identifier-msgs ros-rolling-rcl-interfaces \
      ros-rolling-tf2-msgs ros-rolling-ament-index-cpp \
      ros-rolling-rosidl-buffer \
    && rm -rf /var/lib/apt/lists/*

# --- nav2_common / nav2_msgs / nav2_util / nav2_ros_common'ı kaynaktan ayrı bir overlay'e derle ---
# rolling rosdistro release havuzunda navigation2 paketleri yok (distribution.yaml'da
# sadece nav2_minimal_* var) → apt'ten kurulamıyor, ros-navigation/navigation2'den
# ilgili 4 paketi (sparse-checkout ile, tüm repoyu klonlamadan) colcon ile
# /opt/nav2_ws altına derleyip kuruyoruz. ROS bağımlılıkları yukarıda apt ile
# zaten kurulu olduğundan rosdep'e ihtiyaç yok.
#
# NOT: opennav_coverage_bt / opennav_coverage_navigator'ın gerektirdiği BT yığını
# (nav2_core, nav2_behavior_tree, nav2_bt_navigator) bilerek KURULMUYOR. Bunlar
# rolling apt'te yok ve kaynaktan derlemek nav2_core→nav2_costmap_2d üzerinden
# nav2'nin büyük kısmını çekerdi (uzun, kırılgan). Ayrıca bu paketler upstream'de
# main-branch nav2'ye (nav2::LifecycleNode) migrate edildiğinden jazzy apt
# sürümüyle de çalışmıyor → bt/navigator iki container'da da kapsam dışı.
# Migration burada çekirdek paketlerle doğrulanır: opennav_coverage +
# opennav_row_coverage.
ARG NAV2_BRANCH=main
RUN mkdir -p /opt/nav2_ws/src /tmp/navigation2 \
    && cd /tmp/navigation2 \
    && git init -q \
    && git remote add origin https://github.com/ros-navigation/navigation2.git \
    && git config core.sparseCheckout true \
    && git sparse-checkout set --no-cone nav2_common nav2_msgs nav2_util nav2_ros_common \
    && git fetch --depth 1 --filter=blob:none origin ${NAV2_BRANCH} \
    && git checkout FETCH_HEAD \
    && cp -r nav2_common nav2_msgs nav2_util nav2_ros_common /opt/nav2_ws/src/ \
    && rm -rf /tmp/navigation2 \
    && . /opt/ros/${ROS_DISTRO}/setup.sh \
    && cd /opt/nav2_ws \
    && colcon build --merge-install --install-base /opt/nav2_ws/install \
      --cmake-args -DBUILD_TESTING=OFF \
    && rm -rf /opt/nav2_ws/build /opt/nav2_ws/log

# --- Fields2Cover v2.0.0'ı kaynaktan /usr/local'e derle ---
# BUILD_PYTHON/BUILD_TUTORIALS/BUILD_TESTING kapalı: swig/gtest/gnuplot gibi
# opennav_coverage için gereksiz ek bağımlılıkları (ve build süresini) önler.
ARG F2C_VERSION=v2.0.0
RUN git clone --branch ${F2C_VERSION} --depth 1 \
      https://github.com/Fields2Cover/Fields2Cover.git /tmp/Fields2Cover \
    && cmake -S /tmp/Fields2Cover -B /tmp/Fields2Cover/build \
      -DCMAKE_BUILD_TYPE=Release \
      -DBUILD_PYTHON=OFF \
      -DBUILD_TUTORIALS=OFF \
      -DBUILD_TESTING=OFF \
    && cmake --build /tmp/Fields2Cover/build -j$(nproc) \
    && cmake --install /tmp/Fields2Cover/build \
    && ldconfig \
    && rm -rf /tmp/Fields2Cover

# --- Bind-mount dosya izinleri için root olmayan kullanıcı ---
# Ubuntu 24.04 (noble) tabanlı imajlar artık varsayılan olarak uid/gid 1000'de
# "ubuntu" kullanıcısı içeriyor → "ros" kullanıcısını aynı uid/gid'de oluşturmadan
# önce onu siliyoruz (yoksa groupadd/useradd "already exists" hatası verir).
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=${USER_UID}
RUN (getent passwd ${USER_UID} | cut -d: -f1 | xargs -r userdel -r) || true \
    && (getent group ${USER_GID} | cut -d: -f1 | xargs -r groupdel) || true \
    && groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

# --- colcon workspace kökünü kullanıcıya ait yap ---
# /ros_ws'i Docker mount sırasında root oluşturur; ros kullanıcısı build/install/log
# yazabilsin diye sahipliği veriyoruz (yoksa 'sudo colcon build'e mecbur kalınır).
RUN mkdir -p /ros_ws && chown ${USER_UID}:${USER_GID} /ros_ws

# --- Her shell'de ROS + nav2 overlay'i + workspace overlay'i source et ---
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USERNAME}/.bashrc \
    && echo "source /opt/nav2_ws/install/setup.bash" >> /home/${USERNAME}/.bashrc \
    && echo "[ -f /ros_ws/install/setup.bash ] && source /ros_ws/install/setup.bash" >> /home/${USERNAME}/.bashrc

USER ${USERNAME}
