%bcond_without tests
%bcond_without weak_deps

%global __os_install_post %(echo '%{__os_install_post}' | sed -e 's!/usr/lib[^[:space:]]*/brp-python-bytecompile[[:space:]].*$!!g')
%global __provides_exclude_from ^/opt/ros/jazzy/.*$
%global __requires_exclude_from ^/opt/ros/jazzy/.*$

%global __cmake_in_source_build 1

Name:           ros-jazzy-autoware-behavior-velocity-planner
Version:        1.8.0
Release:        1%{?dist}%{?release_suffix}
Summary:        ROS autoware_behavior_velocity_planner package

License:        Apache License 2.0
URL:            https://index.ros.org/p/autoware_behavior_velocity_planner/#jazzy
Source0:        %{name}-%{version}.tar.gz

Requires:       boost-devel
Requires:       eigen3-devel
Requires:       ros-jazzy-autoware-behavior-velocity-planner-common
Requires:       ros-jazzy-autoware-internal-debug-msgs
Requires:       ros-jazzy-autoware-internal-planning-msgs
Requires:       ros-jazzy-autoware-map-msgs
Requires:       ros-jazzy-autoware-motion-utils
Requires:       ros-jazzy-autoware-perception-msgs
Requires:       ros-jazzy-autoware-planning-msgs
Requires:       ros-jazzy-autoware-route-handler
Requires:       ros-jazzy-autoware-trajectory
Requires:       ros-jazzy-autoware-utils-debug
Requires:       ros-jazzy-autoware-utils-logging
Requires:       ros-jazzy-autoware-utils-pcl
Requires:       ros-jazzy-autoware-utils-rclcpp
Requires:       ros-jazzy-autoware-velocity-smoother
Requires:       ros-jazzy-diagnostic-msgs
Requires:       ros-jazzy-geometry-msgs
Requires:       ros-jazzy-pcl-conversions
Requires:       ros-jazzy-pluginlib
Requires:       ros-jazzy-rclcpp
Requires:       ros-jazzy-rclcpp-components
Requires:       ros-jazzy-sensor-msgs
Requires:       ros-jazzy-tf2
Requires:       ros-jazzy-tf2-eigen
Requires:       ros-jazzy-tf2-geometry-msgs
Requires:       ros-jazzy-tf2-ros
Requires:       ros-jazzy-visualization-msgs
Requires:       ros-jazzy-ros-workspace
BuildRequires:  boost-devel
BuildRequires:  eigen3-devel
BuildRequires:  ros-jazzy-ament-cmake-auto
BuildRequires:  ros-jazzy-autoware-behavior-velocity-planner-common
BuildRequires:  ros-jazzy-autoware-cmake
BuildRequires:  ros-jazzy-autoware-internal-debug-msgs
BuildRequires:  ros-jazzy-autoware-internal-planning-msgs
BuildRequires:  ros-jazzy-autoware-map-msgs
BuildRequires:  ros-jazzy-autoware-motion-utils
BuildRequires:  ros-jazzy-autoware-perception-msgs
BuildRequires:  ros-jazzy-autoware-planning-msgs
BuildRequires:  ros-jazzy-autoware-route-handler
BuildRequires:  ros-jazzy-autoware-trajectory
BuildRequires:  ros-jazzy-autoware-utils-debug
BuildRequires:  ros-jazzy-autoware-utils-logging
BuildRequires:  ros-jazzy-autoware-utils-pcl
BuildRequires:  ros-jazzy-autoware-utils-rclcpp
BuildRequires:  ros-jazzy-autoware-velocity-smoother
BuildRequires:  ros-jazzy-diagnostic-msgs
BuildRequires:  ros-jazzy-eigen3-cmake-module
BuildRequires:  ros-jazzy-geometry-msgs
BuildRequires:  ros-jazzy-pcl-conversions
BuildRequires:  ros-jazzy-pluginlib
BuildRequires:  ros-jazzy-rclcpp
BuildRequires:  ros-jazzy-rclcpp-components
BuildRequires:  ros-jazzy-sensor-msgs
BuildRequires:  ros-jazzy-tf2
BuildRequires:  ros-jazzy-tf2-eigen
BuildRequires:  ros-jazzy-tf2-geometry-msgs
BuildRequires:  ros-jazzy-tf2-ros
BuildRequires:  ros-jazzy-visualization-msgs
BuildRequires:  ros-jazzy-ros-workspace
Provides:       %{name}-devel = %{version}-%{release}
Provides:       %{name}-doc = %{version}-%{release}
Provides:       %{name}-runtime = %{version}-%{release}

%if 0%{?with_tests}
BuildRequires:  ros-jazzy-ament-cmake-ros
BuildRequires:  ros-jazzy-ament-lint-auto
BuildRequires:  ros-jazzy-autoware-lint-common
%endif

%description
The autoware_behavior_velocity_planner package

%prep
%autosetup -p1

%build
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/jazzy/setup.sh" ]; then . "/opt/ros/jazzy/setup.sh"; fi
mkdir -p .obj-%{_target_platform} && cd .obj-%{_target_platform}
%cmake3 \
    -UINCLUDE_INSTALL_DIR \
    -ULIB_INSTALL_DIR \
    -USYSCONF_INSTALL_DIR \
    -USHARE_INSTALL_PREFIX \
    -ULIB_SUFFIX \
    -DCMAKE_INSTALL_PREFIX="/opt/ros/jazzy" \
    -DAMENT_PREFIX_PATH="/opt/ros/jazzy" \
    -DCMAKE_PREFIX_PATH="/opt/ros/jazzy" \
    -DSETUPTOOLS_DEB_LAYOUT=OFF \
%if !0%{?with_tests}
    -DBUILD_TESTING=OFF \
%endif
    ..

%make_build

%install
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/jazzy/setup.sh" ]; then . "/opt/ros/jazzy/setup.sh"; fi
%make_install -C .obj-%{_target_platform}

%if 0%{?with_tests}
%check
# Look for a Makefile target with a name indicating that it runs tests
TEST_TARGET=$(%__make -qp -C .obj-%{_target_platform} | sed "s/^\(test\|check\):.*/\\1/;t f;d;:f;q0")
if [ -n "$TEST_TARGET" ]; then
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/jazzy/setup.sh" ]; then . "/opt/ros/jazzy/setup.sh"; fi
CTEST_OUTPUT_ON_FAILURE=1 \
    %make_build -C .obj-%{_target_platform} $TEST_TARGET || echo "RPM TESTS FAILED"
else echo "RPM TESTS SKIPPED"; fi
%endif

%files
/opt/ros/jazzy

%changelog
* Fri May 08 2026 Zulfaqar Azmi <zulfaqar.azmi@tier4.jp> - 1.8.0-1
- Autogenerated by Bloom

