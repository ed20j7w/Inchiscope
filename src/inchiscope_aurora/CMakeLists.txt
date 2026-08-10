cmake_minimum_required(VERSION 3.8)
project(inchiscope_aurora)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Threads REQUIRED)

# The NDI "Combined API Sample C++" SDK is proprietary vendor sample code
# with no redistribution grant in its license.txt, so it is NOT vendored
# into this repo -- see README.md in this package for how to obtain and
# place it. This defaults to third_party/ndi_combined_api/CombinedAPIsample
# at the repo root (gitignored); override with
# --cmake-args -DNDI_COMBINED_API_DIR=/path/to/CombinedAPIsample if you keep
# it elsewhere.
set(NDI_COMBINED_API_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/ndi_combined_api/CombinedAPIsample"
  CACHE PATH "Path to the extracted NDI Combined API Sample (the 'CombinedAPIsample' directory)"
)

if(NOT EXISTS "${NDI_COMBINED_API_DIR}/library/include/CombinedApi.h")
  message(WARNING
    "inchiscope_aurora: NDI Combined API not found at "
    "'${NDI_COMBINED_API_DIR}/library/include/CombinedApi.h'. "
    "aurora_tracker_node will NOT be built. See "
    "src/inchiscope_aurora/README.md, then re-run colcon build with "
    "--cmake-args -DNDI_COMBINED_API_DIR=/path/to/CombinedAPIsample if you "
    "keep the SDK somewhere other than third_party/ndi_combined_api/."
  )
else()
  # Compile the vendor's CAPI sources directly into our executable rather
  # than linking their prebuilt libndicapi.so, so there's no vendor .so to
  # place on the runtime library path. TlsConnection.cpp/DtlsConnection.cpp
  # are fully guarded by #ifdef OPENSSL and compile to empty translation
  # units without it -- we don't need TLS for a local serial connection.
  file(GLOB NDI_CAPI_SOURCES "${NDI_COMBINED_API_DIR}/library/src/*.cpp")

  add_executable(aurora_tracker_node
    src/aurora_tracker_node.cpp
    ${NDI_CAPI_SOURCES}
  )
  target_include_directories(aurora_tracker_node PRIVATE
    "${NDI_COMBINED_API_DIR}/library/include"
    "${NDI_COMBINED_API_DIR}/library/src/include"
  )

  # The vendor sources are pre-existing third-party code we don't maintain --
  # our -Wall -Wextra is directory-scoped so it lands on them too by default,
  # which is just noise for code we're not going to change. Silence warnings
  # there instead of on aurora_tracker_node.cpp; -w appended after -Wall
  # -Wextra on the same compile line wins for GCC/Clang.
  set_source_files_properties(${NDI_CAPI_SOURCES} PROPERTIES COMPILE_OPTIONS "-w")

  ament_target_dependencies(aurora_tracker_node rclcpp geometry_msgs tf2_ros)
  target_link_libraries(aurora_tracker_node Threads::Threads)

  install(TARGETS aurora_tracker_node
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
