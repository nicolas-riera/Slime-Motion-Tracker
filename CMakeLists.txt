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
  core/src/mounting_calibration.cpp
  core/src/protocol.cpp
  core/src/tracker_hub.cpp
  core/src/config.cpp
  core/src/linalg.cpp)
target_include_directories(slimelite_core PUBLIC core/include)
target_link_libraries(slimelite_core PUBLIC nlohmann_json::nlohmann_json)
if(WIN32)
  target_link_libraries(slimelite_core PUBLIC ws2_32)
endif()

# ---- offline tools ----
add_executable(slimelite_replay tools/replay/replay.cpp)
target_link_libraries(slimelite_replay PRIVATE slimelite_core)

add_executable(slimelite_monitor tools/monitor/monitor.cpp)
target_link_libraries(slimelite_monitor PRIVATE slimelite_core)

add_executable(slimelite_tests tests/tests.cpp)
target_link_libraries(slimelite_tests PRIVATE slimelite_core)
enable_testing()
add_test(NAME core COMMAND slimelite_tests)

# ---- SteamVR driver ----
add_library(driver_slimevr_lite SHARED
  driver/src/driver_main.cpp
  driver/src/device_provider.cpp
  driver/src/virtual_tracker.cpp
  driver/src/overlay_link.cpp)
target_include_directories(driver_slimevr_lite PRIVATE ${OPENVR_INCLUDE})
target_link_libraries(driver_slimevr_lite PRIVATE slimelite_core)
endif()

# ---- dashboard overlay ----
add_executable(slimevr_lite_overlay WIN32
  overlay/src/main.cpp
  overlay/src/ui.cpp
  overlay/src/driver_link.cpp
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

# ---- assemble the SteamVR driver folder in build/slimevr_lite ----
if(NOT SLIMELITE_OVERLAY_ONLY)
set(DRIVER_OUT ${CMAKE_BINARY_DIR}/slimevr_lite)
add_custom_target(driver_package ALL
  COMMAND ${CMAKE_COMMAND} -E make_directory ${DRIVER_OUT}/bin/win64
  COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/driver/package ${DRIVER_OUT}
  COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:driver_slimevr_lite> ${DRIVER_OUT}/bin/win64/
  COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:slimevr_lite_overlay> ${DRIVER_OUT}/bin/win64/
  COMMAND ${CMAKE_COMMAND} -E copy ${OPENVR_DLL} ${DRIVER_OUT}/bin/win64/
  DEPENDS driver_slimevr_lite slimevr_lite_overlay)
endif()
