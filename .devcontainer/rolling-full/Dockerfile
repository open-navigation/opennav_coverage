# syntax=docker/dockerfile:1
# ROS 2 Rolling (Ubuntu 26.04 / resolute) — GENIS KAPSAM build ortami (CI'in yerel kopyasi)
ARG ROS_DISTRO=rolling
FROM ghcr.io/ros-tooling/setup-ros-docker/setup-ros-docker-ubuntu-resolute-ros-rolling-ros-base:master
ARG ROS_DISTRO=rolling

ENV DEBIAN_FRONTEND=noninteractive
ENV CMAKE_POLICY_VERSION_MINIMUM=3.5

# BuildKit apt cache mount'larinin kalici olmasini sagla
RUN rm -f /etc/apt/apt.conf.d/docker-clean \
    && echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' > /etc/apt/apt.conf.d/keep-cache

# --- Temel derleme araclari ---
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake git sudo ca-certificates wget \
      python3-colcon-common-extensions python3-vcstool python3-rosdep

# --- Rosdep ilk kurulumu (Zaten varsa hata vermemesi icin korumali) ---
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi

# --- Kullanici izinleri ---
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=${USER_UID}
RUN (getent passwd ${USER_UID} | cut -d: -f1 | xargs -r userdel -r) || true \
    && (getent group ${USER_GID} | cut -d: -f1 | xargs -r groupdel) || true \
    && groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

RUN mkdir -p /ros_ws/src /ros_ws/build /ros_ws/install /ros_ws/log \
    && chown -R ${USER_UID}:${USER_GID} /ros_ws

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USERNAME}/.bashrc \
    && echo "[ -f /ros_ws/install/setup.bash ] && source /ros_ws/install/setup.bash" >> /home/${USERNAME}/.bashrc

USER ${USERNAME}
WORKDIR /ros_ws

# --- Aktiflestirilen Ilk 5 Interaktif Adim ---
# --- Aktiflestirilen Ilk 5 Interaktif Adim ---
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    sudo apt-get update && sudo apt-get install -y --no-install-recommends \
      libgdal-dev libeigen3-dev libgeos-dev libtinyxml2-dev libompl-dev \
      ros-rolling-rmw-cyclonedds-cpp ros-rolling-tinyxml2-vendor \
      ros-rolling-backward-ros ros-rolling-ament-cmake-vendor-package

# # Not: vcs import icin opennav_coverage reposunun container'a onceden bind-mount veya COPY ile verilmis olmasi gerekir.
# vcs import'u BuildKit'in kalici cache mount'una yapiyoruz: bu klasor normal
# layer cache'inden bagimsizdir, ustteki katmanlar degisip cache bozulsa bile
# (ornegin apt paketi eklemek) burasi diskte kalir -> vcs import zaten var olan
# repo'larda tam klon yerine hizli fetch/checkout yapar. Sonra kalici katmana
# kopyaliyoruz (cache mount'un kendisi image'a gommez).
RUN --mount=type=cache,target=/home/ros/.cache/vcs-deps,uid=1000,gid=1000,sharing=locked \
    --mount=type=bind,source=.,target=/ros_ws/src/opennav_coverage \
    mkdir -p src/deps /home/ros/.cache/vcs-deps && \
    vcs import /home/ros/.cache/vcs-deps < /ros_ws/src/opennav_coverage/deps.repos && \
    cp -a /home/ros/.cache/vcs-deps/. src/deps/



RUN --mount=type=bind,source=.,target=/ros_ws/src/opennav_coverage \
    if [ -f src/deps/navigation2/nav2_common/cmake/nav2_package.cmake ]; then \
      sed -i 's/ -Werror//' src/deps/navigation2/nav2_common/cmake/nav2_package.cmake; \
    fi

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    --mount=type=bind,source=.,target=/ros_ws/src/opennav_coverage \
    rosdep update --rosdistro rolling && \
    rosdep install --from-paths src --ignore-src --rosdistro rolling -r -y || true

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    sudo apt-get remove -y ros-rolling-behaviortree-cpp ros-rolling-ompl || true

# ============================================================================
# KALAN INTERAKTIF ADIMLAR (Container icinde kosulacaklar)
# ============================================================================
# # 6) Genis kapsam build:
# colcon build --packages-up-to \
#   nav2_bt_navigator nav2_behavior_tree opennav_coverage opennav_coverage_bt \
#   opennav_coverage_demo opennav_coverage_msgs opennav_coverage_navigator \
#   opennav_row_coverage --packages-skip slam_toolbox --event-handlers console_cohesion+
#
# # 7) Test:
# colcon test --packages-select opennav_* --event-handlers console_cohesion+ && colcon test-result --verbose
# ============================================================================