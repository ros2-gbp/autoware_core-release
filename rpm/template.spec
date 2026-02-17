%bcond_without tests
%bcond_without weak_deps

%global __os_install_post %(echo '%{__os_install_post}' | sed -e 's!/usr/lib[^[:space:]]*/brp-python-bytecompile[[:space:]].*$!!g')
%global __provides_exclude_from ^/opt/ros/humble/.*$
%global __requires_exclude_from ^/opt/ros/humble/.*$

%global __cmake_in_source_build 1

Name:           ros-humble-autoware-ndt-scan-matcher
Version:        1.7.0
Release:        1%{?dist}%{?release_suffix}
Summary:        ROS autoware_ndt_scan_matcher package

License:        Apache License 2.0 and BSD
Source0:        %{name}-%{version}.tar.gz

Requires:       fmt-devel
Requires:       pcl
Requires:       pcl-tools
Requires:       ros-humble-autoware-internal-debug-msgs
Requires:       ros-humble-autoware-internal-localization-msgs
Requires:       ros-humble-autoware-localization-util
Requires:       ros-humble-autoware-map-msgs
Requires:       ros-humble-autoware-qos-utils
Requires:       ros-humble-autoware-utils-diagnostics
Requires:       ros-humble-autoware-utils-logging
Requires:       ros-humble-autoware-utils-pcl
Requires:       ros-humble-autoware-utils-visualization
Requires:       ros-humble-diagnostic-msgs
Requires:       ros-humble-geometry-msgs
Requires:       ros-humble-nav-msgs
Requires:       ros-humble-pcl-conversions
Requires:       ros-humble-rclcpp
Requires:       ros-humble-rclcpp-components
Requires:       ros-humble-sensor-msgs
Requires:       ros-humble-std-srvs
Requires:       ros-humble-tf2
Requires:       ros-humble-tf2-eigen
Requires:       ros-humble-tf2-geometry-msgs
Requires:       ros-humble-tf2-ros
Requires:       ros-humble-tf2-sensor-msgs
Requires:       ros-humble-visualization-msgs
Requires:       ros-humble-ros-workspace
BuildRequires:  fmt-devel
BuildRequires:  pcl
BuildRequires:  pcl-devel
BuildRequires:  pcl-tools
BuildRequires:  ros-humble-ament-cmake-auto
BuildRequires:  ros-humble-autoware-cmake
BuildRequires:  ros-humble-autoware-internal-debug-msgs
BuildRequires:  ros-humble-autoware-internal-localization-msgs
BuildRequires:  ros-humble-autoware-localization-util
BuildRequires:  ros-humble-autoware-map-msgs
BuildRequires:  ros-humble-autoware-qos-utils
BuildRequires:  ros-humble-autoware-utils-diagnostics
BuildRequires:  ros-humble-autoware-utils-logging
BuildRequires:  ros-humble-autoware-utils-pcl
BuildRequires:  ros-humble-autoware-utils-visualization
BuildRequires:  ros-humble-diagnostic-msgs
BuildRequires:  ros-humble-geometry-msgs
BuildRequires:  ros-humble-nav-msgs
BuildRequires:  ros-humble-pcl-conversions
BuildRequires:  ros-humble-rclcpp
BuildRequires:  ros-humble-rclcpp-components
BuildRequires:  ros-humble-sensor-msgs
BuildRequires:  ros-humble-std-srvs
BuildRequires:  ros-humble-tf2
BuildRequires:  ros-humble-tf2-eigen
BuildRequires:  ros-humble-tf2-geometry-msgs
BuildRequires:  ros-humble-tf2-ros
BuildRequires:  ros-humble-tf2-sensor-msgs
BuildRequires:  ros-humble-visualization-msgs
BuildRequires:  ros-humble-ros-workspace
Provides:       %{name}-devel = %{version}-%{release}
Provides:       %{name}-doc = %{version}-%{release}
Provides:       %{name}-runtime = %{version}-%{release}

%if 0%{?with_tests}
BuildRequires:  ros-humble-ament-cmake-cppcheck
BuildRequires:  ros-humble-ament-index-cpp
BuildRequires:  ros-humble-ament-lint-auto
BuildRequires:  ros-humble-launch-testing-ament-cmake
BuildRequires:  ros-humble-ros-testing
%endif

%description
The autoware_ndt_scan_matcher package

%prep
%autosetup -p1

%build
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/humble/setup.sh" ]; then . "/opt/ros/humble/setup.sh"; fi
mkdir -p .obj-%{_target_platform} && cd .obj-%{_target_platform}
%cmake3 \
    -UINCLUDE_INSTALL_DIR \
    -ULIB_INSTALL_DIR \
    -USYSCONF_INSTALL_DIR \
    -USHARE_INSTALL_PREFIX \
    -ULIB_SUFFIX \
    -DCMAKE_INSTALL_PREFIX="/opt/ros/humble" \
    -DAMENT_PREFIX_PATH="/opt/ros/humble" \
    -DCMAKE_PREFIX_PATH="/opt/ros/humble" \
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
if [ -f "/opt/ros/humble/setup.sh" ]; then . "/opt/ros/humble/setup.sh"; fi
%make_install -C .obj-%{_target_platform}

%if 0%{?with_tests}
%check
# Look for a Makefile target with a name indicating that it runs tests
TEST_TARGET=$(%__make -qp -C .obj-%{_target_platform} | sed "s/^\(test\|check\):.*/\\1/;t f;d;:f;q0")
if [ -n "$TEST_TARGET" ]; then
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/humble/setup.sh" ]; then . "/opt/ros/humble/setup.sh"; fi
CTEST_OUTPUT_ON_FAILURE=1 \
    %make_build -C .obj-%{_target_platform} $TEST_TARGET || echo "RPM TESTS FAILED"
else echo "RPM TESTS SKIPPED"; fi
%endif

%files
/opt/ros/humble

%changelog
* Tue Feb 17 2026 Yamato Ando <yamato.ando@tier4.jp> - 1.7.0-1
- Autogenerated by Bloom

* Tue Aug 12 2025 Yamato Ando <yamato.ando@tier4.jp> - 1.4.0-1
- Autogenerated by Bloom

