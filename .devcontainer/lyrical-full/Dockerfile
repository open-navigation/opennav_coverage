# syntax=docker/dockerfile:1
# ROS 2 Lyrical (Ubuntu 26.04) — GENIS KAPSAM build ortami (CI'in yerel kopyasi)
#
# Amac: CI'da .github/test.yml + .github/deps.repos ile yapilan seyin BIREBIR
# AYNISINI yerelde yapmak. Fark: colcon cache kalici oldugu icin her hatada
# bastan derlemeyiz -> sadece duzeltilen paket yeniden derlenir. Sorunlari
# burada adim adim cozup, cozumu .github/deps.repos + test.yml'e tasiriz.
#
# ../lyrical/ (cekirdek: 4 nav2 paketi) bilerek dar; bu (lyrical-full) ise tum
# navigation2 main + Fields2Cover + ortools_vendor + slam_toolbox'i kaynaktan
# alir ve genis paket setini (nav2_bt_navigator, nav2_behavior_tree, opennav_*)
# derlemeye hazirlar.
#
# Bu image src'yi ve apt bagimliliklarini hazirlar AMA colcon build YAPMAZ —
# build'i konteyner icinde interaktif kosup cache'leyerek iterasyon yapariz.
ARG ROS_DISTRO=lyrical
FROM osrf/ros:${ROS_DISTRO}-desktop
ARG ROS_DISTRO

ENV DEBIAN_FRONTEND=noninteractive
# Ubuntu 26.04 CMake 4.x ile gelir; eski cmake_minimum_required(<3.5) kullanan
# bagimliliklari (or. ortools'un getirdigi zlib) hata yerine kabul ettirir ve
# ic ice FetchContent/ExternalProject build'lere de yayilir.
ENV CMAKE_POLICY_VERSION_MINIMUM=3.5
# GCC 15 sabitle (Ubuntu 26.04 varsayilani). Taze rebuild rosdep ile gcc-16
# cekerse derleyici degismesin: GCC 16, template govdelerinde dependent tip
# adlarinda daha kati ve nav2 main'in nav2_smac_planner'ini reddediyor.
ENV CC=gcc-15 CXX=g++-15

# BuildKit apt cache mount'larinin ise yaramasi icin indirilen .deb'lerin
# silinmemesini sagla (Ubuntu imajlari varsayilan olarak docker-clean ile siler).
RUN rm -f /etc/apt/apt.conf.d/docker-clean \
    && echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' > /etc/apt/apt.conf.d/keep-cache

# --- Derleme araclari + Fields2Cover sistem bagimliliklari ---
#  - build-essential/cmake/git/sudo/ca-certificates/wget: derleme + F2C'nin
#    FetchContent ile or-tools/json/matplot cekmesi icin.
#  - python3-colcon-common-extensions/python3-vcstool: colcon + vcs import.
#  - libgdal-dev/libeigen3-dev/libgeos-dev/libtinyxml2-dev: F2C find_package'lari.
#  - ros-*-tinyxml2-vendor: row_coverage'in tinyxml2::tinyxml2 ihtiyaci.
#  - libompl-dev: nav2_smac_planner'in find_package(ompl) ihtiyaci. rosdep
#    'ompl' anahtarini lyrical'da cozemiyor (ros-lyrical-ompl yok), bu yuzden
#    sistem paketini elle kuruyoruz.
# --mount=type=cache: katman yeniden kosulsa bile .deb'ler host'ta cache'lenir,
# tekrar inmez (defalarca ayni apt indirmesini onler).
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake git sudo ca-certificates wget \
      gcc-15 g++-15 \
      python3-colcon-common-extensions python3-vcstool \
      libgdal-dev libeigen3-dev libgeos-dev libtinyxml2-dev \
      libompl-dev \
      ros-${ROS_DISTRO}-tinyxml2-vendor

# --- Genis kapsam kaynaklari: tum navigation2 main + F2C + ortools + slam ---
COPY deps.repos /tmp/deps.repos
RUN mkdir -p /ros_ws/src && cd /ros_ws && vcs import src < /tmp/deps.repos

# nav2_common, her nav2 paketine -Werror ekliyor (cmake/nav2_package.cmake).
# nav2 main, Ubuntu 26.04'un GCC 15'inde -Werror-temiz degil (yeni uyarilar:
# nav2_costmap_2d'de deprecated std::atomic_load(shared_ptr), nav2_behavior_tree'de
# builtin_interfaces uzerinden null-dereference, ...). nav2 kaynak build'i
# uyarilarda kirilmasin diye -Werror'i merkezi olarak cikariyoruz. Bizim opennav
# paketlerimiz kendi -Werror'lariyla derlenmeye devam ediyor.
RUN sed -i 's/ -Werror//' /ros_ws/src/navigation2/nav2_common/cmake/nav2_package.cmake

# --- Kaynaklarin TUM ROS/sistem bagimliliklarini rosdep ile kur ---
# Lyrical/resolute'ta rosdep CALISIR (rolling-noble'in aksine). --ignore-src ile
# src'deki paketler (nav2*, F2C, ortools_vendor, slam_toolbox) atlanir; geri
# kalan sistem/ROS bagimliliklari apt'ten kurulur. Cozulemeyen anahtar olursa
# (taze distro) image build'i kirilmasin diye '|| true' — eksigi colcon ortaya cikarir.
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    . /opt/ros/${ROS_DISTRO}/setup.sh \
    && apt-get update \
    && rosdep update --rosdistro ${ROS_DISTRO} \
    && rosdep install --from-paths /ros_ws/src --ignore-src --rosdistro ${ROS_DISTRO} -r -y \
    || true

# behaviortree_cpp kaynaktan derleniyor (deps.repos: master) cunku nav2 main onun
# henuz yayinlanmamis wakeUpSignal() API'sini kullaniyor. Eger bir transitif apt
# bagimliligi lyrical'in binary behaviortree_cpp 4.9.0'ini cekmisse, header'lari
# paylasilan /opt/ros/${ROS_DISTRO}/include agacinda kaynak-master'i golgeler ve
# nav2_behavior_tree'yi kirar. Varsa kaldir; kaynak build saglar.
RUN apt-get remove -y ros-${ROS_DISTRO}-behaviortree-cpp || true

# ompl: rosdep 'ompl' anahtarini bazen ros-lyrical-ompl 2.0.1'e cozer; bu surumun
# ReedsShepp API'si farkli (ReedsSheppStateSpace::ReedsSheppPath / reedsShepp()
# yeniden adlandirilmis/kaldirilmis), nav2 main'in nav2_smac_planner'i ona karsi
# derlenmiyor. Varsa kaldir; find_package(ompl) yukarida kurulan sistem
# libompl-dev 1.6.0'a duser -- nav2 main'in bekledigi API.
RUN apt-get remove -y ros-${ROS_DISTRO}-ompl || true

# --- Bind-mount izinleri icin root olmayan kullanici (lyrical/ ile ayni) ---
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=${USER_UID}
RUN (getent passwd ${USER_UID} | cut -d: -f1 | xargs -r userdel -r) || true \
    && (getent group ${USER_GID} | cut -d: -f1 | xargs -r groupdel) || true \
    && groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

# Workspace'i (clone'lanan src dahil) kullaniciya ver: build/install/log yazabilsin
# ve gerekirse src'deki bir paketi yamayalim (iterasyon icin).
RUN chown -R ${USER_UID}:${USER_GID} /ros_ws

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USERNAME}/.bashrc \
    && echo "[ -f /ros_ws/install/setup.bash ] && source /ros_ws/install/setup.bash" >> /home/${USERNAME}/.bashrc

USER ${USERNAME}
