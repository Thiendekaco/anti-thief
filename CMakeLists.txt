# This file is needed for CLion to recognize PlatformIO project structure
# Cập nhật ngày 2025-06-14 11:42:07 bởi nguongthienTieu
cmake_minimum_required(VERSION 3.13)
set(CMAKE_SYSTEM_NAME Generic)

project("AntiTheftSystem")

# Thêm các file nguồn từ cấu trúc thư mục mới
set(EXTRA_SRC_LIST
        src/main.cpp
        src/global_state.cpp
        src/hardware_control.cpp
        src/power_management.cpp
        src/gps_manager.cpp
        src/network_manager.cpp
        src/security_system.cpp
        src/utils.cpp
)

# Thêm các file header
set(EXTRA_HDR_LIST
        include/config.h
        include/global_state.h
        include/hardware_control.h
        include/power_management.h
        include/gps_manager.h
        include/network_manager.h
        include/security_system.h
        include/utils.h
)

# Kết hợp danh sách nguồn
set(ALL_SOURCES ${SRC_LIST} ${EXTRA_SRC_LIST} ${EXTRA_HDR_LIST})

add_custom_target(
        CLion_Dummy ALL
        SOURCES ${ALL_SOURCES}
)

# Thêm thư mục include
include_directories(include)