#!/usr/bin/env bash
# Build all packages using ROS Noetic and optional workspace-local dependencies.
set -eo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$workspace_dir"
source /opt/ros/noetic/setup.bash

if [[ -f "$workspace_dir/.build_deps/mavros-root/setup-local.bash" ]]; then
    source "$workspace_dir/.build_deps/mavros-root/setup-local.bash"
fi

local_ros_prefix="$workspace_dir/.build_deps/prefix/opt/ros/noetic"
if [[ -d "$local_ros_prefix" ]]; then
    export CMAKE_PREFIX_PATH="$local_ros_prefix:${CMAKE_PREFIX_PATH:-}"
fi

if [[ ! -f src/livox_ros_driver2/package.xml ]]; then
    cp src/livox_ros_driver2/package_ROS1.xml src/livox_ros_driver2/package.xml
fi

dependency_options=()
if [[ -f "$local_ros_prefix/share/octomap/octomap-config.cmake" ]]; then
    dependency_options+=("-Doctomap_DIR=$local_ros_prefix/share/octomap")
fi
if [[ -f "$workspace_dir/.build_deps/livox-build/sdk_core/liblivox_lidar_sdk_static.a" ]]; then
    dependency_options+=(
        "-DLIVOX_LIDAR_SDK_LIBRARY=$workspace_dir/.build_deps/livox-build/sdk_core/liblivox_lidar_sdk_static.a"
        "-DLIVOX_LIDAR_SDK_INCLUDE_DIR=$workspace_dir/.build_deps/Livox-SDK2/include"
    )
fi

catkin_make -j2 -l4 -DROS_EDITION=ROS1 -DCMAKE_BUILD_TYPE=Release "${dependency_options[@]}" "$@"
