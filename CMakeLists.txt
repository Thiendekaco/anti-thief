# This file is needed for CLion to recognize PlatformIO project structure
cmake_minimum_required(VERSION 3.13)
set(CMAKE_SYSTEM_NAME Generic)

include(CMakeListsPrivate.txt)

project("AntiTheftSystem")

add_custom_target(
        CLion_Dummy ALL
        SOURCES ${SRC_LIST}
)