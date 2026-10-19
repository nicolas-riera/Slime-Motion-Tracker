cmake_minimum_required(VERSION 3.24)
project(slimevr_lite LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")

include(FetchContent)

# SOURCE_SUBDIR pointing nowhere: download only, we use the headers/sources directly
FetchContent_Declare(openvr
  URL https://github.com/ValveSoftware/openvr/archive/refs/tags/v2.5.1.tar.gz
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  SOURCE_SUBDIR do-not-add)
FetchContent_Declare(imgui
  URL https://github.com/ocornut/imgui/archive/refs/tags/v1.91.9.tar.gz
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  SOURCE_SUBDIR do-not-add)
FetchContent_Declare(json
  URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE)

FetchContent_MakeAvailable(openvr imgui json)

if(MSVC)
  add_compile_options(/W4 /permissive- /utf-8 /wd4100)
  add_compile_definitions(_CRT_SECURE_NO_WARNINGS NOMINMAX WIN32_LEAN_AND_MEAN)
endif()

set(OPENVR_INCLUDE ${openvr_SOURCE_DIR}/headers)
set(OPENVR_LIB ${openvr_SOURCE_DIR}/lib/win64/openvr_api.lib)
set(OPENVR_DLL ${openvr_SOURCE_DIR}/bin/win64/openvr_api.dll)

# Build only the overlay (lets the UI be developed independently of the core)
option(SLIMELITE_OVERLAY_ONLY "Only build the dashboard overlay" OFF)
if(NOT SLIMELITE_OVERLAY_ONLY)

# ---- core: protocol, fusion, body model (no OpenVR dependency) ----
add_library(slimelite_core STATIC
  core/src/mag_calibrator.cpp
  core/src/heading_filter.cpp
  core/src/body_solver.cpp
  core/src/foot_model.cpp
  core/src/pose_model.cpp
  core/src/mounting_calibration.cpp
  core/src/protocol.cpp
  core/src/tracker_hub.cpp
  core/src/dongle_bridge.cpp
  core/src/config.cpp
  core/src/linalg.cpp)
target_include_directories(slimelite_core PUBLIC core/include)
# The pose model network (~70M multiply-adds per step) needs SIMD float math
if(MSVC)
  set_source_files_properties(core/src/pose_model.cpp PROPERTIES COMPILE_OPTIONS "/arch:AVX2;/fp:fast")
else()
  set_source_files_properties(core/src/pose_model.cpp PROPERTIES COMPILE_OPTIONS "-mavx2;-mfma;-ffast-math")
endif()
target_link_libraries(slimelite_core PUBLIC nlohmann_json::nlohmann_json)
if(WIN32)
  target_link_libraries(slimelite_core PUBLIC ws2_32)
endif()

# ---- offline tools ----
add_executable(slimelite_replay tools/replay/replay.cpp)
target_link_libraries(slimelite_replay PRIVATE slimelite_core)

add_executable(slimelite_body_replay tools/body_replay/body_replay.cpp)
target_link_libraries(slimelite_body_replay PRIVATE slimelite_core)

# Learned foot model: dataset dump and C++/Python consistency check (tools/ml)
add_executable(slimelite_ml_dump tools/ml/ml_dump.cpp)
target_link_libraries(slimelite_ml_dump PRIVATE slimelite_core)
add_executable(slimelite_foot_model_check tools/ml/foot_model_check.cpp)
target_link_libraries(slimelite_foot_model_check PRIVATE slimelite_core)
# Learned pose model: C++/Python consistency and timing (tools/pose_model)
add_executable(slimelite_pose_model_check tools/pose_model/pose_model_check.cpp)
target_link_libraries(slimelite_pose_model_check PRIVATE slimelite_core)
# Solver on per-frame inputs (recorded dumps or motion capture converted to our sensors)
add_executable(slimelite_dump_solve tools/pose_model/dump_solve.cpp)
target_link_libraries(slimelite_dump_solve PRIVATE slimelite_core)

# Kinect v2 depth recorder (ground truth), only with the Kinect for Windows SDK 2.0
if(WIN32 AND DEFINED ENV{KINECTSDK20_DIR})
  file(TO_CMAKE_PATH "$ENV{KINECTSDK20_DIR}" KINECT_SDK)
  add_executable(slimelite_kinect_recorder tools/kinect_recorder/kinect_recorder.cpp)
  target_include_directories(slimelite_kinect_recorder PRIVATE ${KINECT_SDK}/inc)
  target_link_libraries(slimelite_kinect_recorder PRIVATE ${KINECT_SDK}/Lib/x64/Kinect20.lib Cabinet Ole32)
endif()

add_executable(slimelite_heading_replay tools/heading_replay/heading_replay.cpp)
target_link_libraries(slimelite_heading_replay PRIVATE slimelite_core)

add_executable(slimelite_monitor tools/monitor/monitor.cpp)
target_link_libraries(slimelite_monitor PRIVATE slimelite_core)

add_executable(slimelite_tests tests/tests.cpp)
target_link_libraries(slimelite_tests PRIVATE slimelite_core)
enable_testing()
add_test(NAME core COMMAND slimelite_tests)

# ---- engine: everything but the SteamVR glue, hot-reloaded by the driver ----
add_library(slimelite_engine SHARED
  engine/src/engine.cpp
  engine/src/overlay_link.cpp
  engine/src/pose_worker.cpp
  engine/src/recorder.cpp)
target_include_directories(slimelite_engine PUBLIC engine/include)
target_link_libraries(slimelite_engine PRIVATE slimelite_core)

# ---- SteamVR driver host (thin, needs a SteamVR restart when it changes) ----
add_library(driver_slimevr_lite SHARED
  driver/src/driver_main.cpp
  driver/src/device_provider.cpp
  driver/src/virtual_tracker.cpp
  driver/src/engine_loader.cpp)
target_include_directories(driver_slimevr_lite PRIVATE ${OPENVR_INCLUDE} engine/include)

add_executable(slimelite_engine_host tools/engine_host/engine_host.cpp driver/src/engine_loader.cpp)
target_include_directories(slimelite_engine_host PRIVATE engine/include)

# Hot deploy: after a build, copy the engine (and the overlay) into the installed
# driver folder when it exists. The running driver reloads the engine by itself.
file(TO_CMAKE_PATH "$ENV{LOCALAPPDATA}" SLIMELITE_LOCALAPPDATA)
set(SLIMELITE_INSTALL_BIN "${SLIMELITE_LOCALAPPDATA}/SlimeVR-lite/driver/slimevr_lite/bin/win64")
add_custom_command(TARGET slimelite_engine POST_BUILD
  COMMAND ${CMAKE_COMMAND} -DSRC=$<TARGET_FILE:slimelite_engine> -DDST=${SLIMELITE_INSTALL_BIN}
          -P ${CMAKE_SOURCE_DIR}/tools/cmake/hot_deploy.cmake
  COMMAND ${CMAKE_COMMAND} -DSRC=${CMAKE_SOURCE_DIR}/engine/models -DDST=${SLIMELITE_INSTALL_BIN}
          -P ${CMAKE_SOURCE_DIR}/tools/cmake/deploy_models.cmake)
# The overlay starts the host when SteamVR is not running (standalone mode)
add_custom_command(TARGET slimelite_engine_host POST_BUILD
  COMMAND ${CMAKE_COMMAND} -DSRC=$<TARGET_FILE:slimelite_engine_host> -DDST=${SLIMELITE_INSTALL_BIN}
          -P ${CMAKE_SOURCE_DIR}/tools/cmake/hot_deploy.cmake)
endif()

# ---- dashboard overlay ----
add_executable(slimevr_lite_overlay WIN32
  overlay/src/main.cpp
  overlay/src/ui.cpp
  overlay/src/driver_link.cpp
  overlay/resources/overlay.rc
  ${imgui_SOURCE_DIR}/imgui.cpp
  ${imgui_SOURCE_DIR}/imgui_draw.cpp
  ${imgui_SOURCE_DIR}/imgui_tables.cpp
  ${imgui_SOURCE_DIR}/imgui_widgets.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_dx11.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_win32.cpp)
target_include_directories(slimevr_lite_overlay PRIVATE
  ${OPENVR_INCLUDE} ${imgui_SOURCE_DIR} ${imgui_SOURCE_DIR}/backends)
target_link_libraries(slimevr_lite_overlay PRIVATE
  ${OPENVR_LIB} nlohmann_json::nlohmann_json d3d11 dxgi ws2_32)

add_custom_command(TARGET slimevr_lite_overlay POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy ${OPENVR_DLL} $<TARGET_FILE_DIR:slimevr_lite_overlay>)
if(NOT SLIMELITE_OVERLAY_ONLY)
  add_custom_command(TARGET slimevr_lite_overlay POST_BUILD
    COMMAND ${CMAKE_COMMAND} -DSRC=$<TARGET_FILE:slimevr_lite_overlay> -DDST=${SLIMELITE_INSTALL_BIN}
            -DRESTART_OVERLAY=ON -P ${CMAKE_SOURCE_DIR}/tools/cmake/hot_deploy.cmake)
endif()

# ---- assemble the SteamVR driver folder in build/slimevr_lite ----
if(NOT SLIMELITE_OVERLAY_ONLY)
set(DRIVER_OUT ${CMAKE_BINARY_DIR}/slimevr_lite)
add_custom_target(driver_package ALL
  COMMAND ${CMAKE_COMMAND} -E make_directory ${DRIVER_OUT}/bin/win64
  COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/driver/package ${DRIVER_OUT}
  COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:driver_slimevr_lite> ${DRIVER_OUT}/bin/win64/
  COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:slimelite_engine> ${DRIVER_OUT}/bin/win64/
  COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/engine/models ${DRIVER_OUT}/bin/win64/models
  COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:slimevr_lite_overlay> ${DRIVER_OUT}/bin/win64/
  COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:slimelite_engine_host> ${DRIVER_OUT}/bin/win64/
  COMMAND ${CMAKE_COMMAND} -E copy ${OPENVR_DLL} ${DRIVER_OUT}/bin/win64/
  DEPENDS driver_slimevr_lite slimelite_engine slimevr_lite_overlay slimelite_engine_host)
endif()
