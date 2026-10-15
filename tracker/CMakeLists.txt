cmake_minimum_required(VERSION 3.10)
project(target_tracker LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-O2")

# ---------------------------------------------------------------------------
# Dependencies
# ---------------------------------------------------------------------------
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(GST REQUIRED gstreamer-1.0 gstreamer-app-1.0)
pkg_check_modules(GLIB REQUIRED glib-2.0)
# Optional: RTSP server output (--rtsp-out). Install libgstrtspserver-1.0-dev to enable; the build works without it.
pkg_check_modules(GST_RTSP QUIET gstreamer-rtsp-server-1.0)

# MAVLink: raw generated C headers (common + ardupilotmega dialects).
# Fetch the pinned revision with scripts/fetch_mavlink.sh. Header-only.
set(MAVLINK_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/third_party/mavlink" CACHE PATH "Path to c_library_v2 checkout")
if(NOT EXISTS "${MAVLINK_INCLUDE_DIR}/ardupilotmega/mavlink.h")
    message(FATAL_ERROR
        "MAVLink C headers not found at ${MAVLINK_INCLUDE_DIR}/ardupilotmega/mavlink.h -- "
        "run: scripts/fetch_mavlink.sh")
endif()

# ---------------------------------------------------------------------------
# Target
# ---------------------------------------------------------------------------

# link_directories() (not target_link_directories -- that needs CMake >=3.13,
# the Jetson Nano's stock CMake is 3.10) must come BEFORE add_executable().
link_directories(
    ${GST_LIBRARY_DIRS}
    ${GLIB_LIBRARY_DIRS}
    ${GST_RTSP_LIBRARY_DIRS}
)

set(TARGET_SOURCES main.cpp)
add_executable(target_tracker ${TARGET_SOURCES})

target_include_directories(target_tracker PRIVATE
    ${CMAKE_SOURCE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${GST_INCLUDE_DIRS}
    ${GLIB_INCLUDE_DIRS}
)
# SYSTEM: the generated MAVLink headers are third-party code; keep -Wall -Wextra output limited to our own.
target_include_directories(target_tracker SYSTEM PRIVATE
    ${MAVLINK_INCLUDE_DIR}
    ${MAVLINK_INCLUDE_DIR}/common
    ${MAVLINK_INCLUDE_DIR}/ardupilotmega
)

target_link_libraries(target_tracker PRIVATE
    ${OpenCV_LIBS}
    ${GST_LIBRARIES}
    ${GLIB_LIBRARIES}
    Threads::Threads
)

target_compile_options(target_tracker PRIVATE -Wall -Wextra ${GST_CFLAGS_OTHER})

if(GST_RTSP_FOUND)
    message(STATUS "gst-rtsp-server ${GST_RTSP_VERSION} found: --rtsp-out enabled")
    target_compile_definitions(target_tracker PRIVATE HAVE_GST_RTSP_SERVER=1)
    target_include_directories(target_tracker PRIVATE ${GST_RTSP_INCLUDE_DIRS})
    target_link_libraries(target_tracker PRIVATE ${GST_RTSP_LIBRARIES})
else()
    message(STATUS "gst-rtsp-server not found: --rtsp-out disabled (apt install libgstrtspserver-1.0-dev to enable)")
endif()


# In-process AI models (DINOv2 ReID, EdgeSAM) through the OpenVINO C++ API -- no Python sidecar needed. The pip package
# (pip install openvino) ships the C++ headers and CMake config; a system install (OpenVINO_DIR) works too. Off: -DWITH_OPENVINO=OFF.
option(WITH_OPENVINO "Build the in-process AI models (OpenVINO C++ API)" ON)
if(WITH_OPENVINO)
    if(NOT OpenVINO_DIR)
        execute_process(COMMAND python3 -c "import openvino, os; print(os.path.join(os.path.dirname(openvino.__file__), 'cmake'))"
                        OUTPUT_VARIABLE OV_PY_CMAKE OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
        if(OV_PY_CMAKE AND EXISTS "${OV_PY_CMAKE}/OpenVINOConfig.cmake")
            set(OpenVINO_DIR "${OV_PY_CMAKE}")
        endif()
    endif()
    find_package(OpenVINO QUIET COMPONENTS Runtime)
endif()
if(OpenVINO_FOUND)
    message(STATUS "OpenVINO ${OpenVINO_VERSION} found (${OpenVINO_DIR}): in-process DINOv2 / EdgeSAM enabled")
    target_compile_definitions(target_tracker PRIVATE HAVE_OPENVINO=1)
    target_link_libraries(target_tracker PRIVATE openvino::runtime)
    get_filename_component(OV_LIBS_DIR "${OpenVINO_DIR}/../libs" ABSOLUTE)    # the pip package's shared libraries
    if(EXISTS "${OV_LIBS_DIR}")
        set_target_properties(target_tracker PROPERTIES BUILD_RPATH "${OV_LIBS_DIR}" INSTALL_RPATH "${OV_LIBS_DIR}")
    endif()
else()
    message(STATUS "OpenVINO C++ not found: AI models only through the Python sidecar (pip install openvino to enable)")
endif()
