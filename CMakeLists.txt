cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif ()

# Set the project name
set(CMAKE_PROJECT_NAME RoboMaster_C)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# 自动收集源文件（使用 CONFIGURE_DEPENDS 自动检测文件变化）

# BSP 层
file(GLOB_RECURSE BSP_SOURCES CONFIGURE_DEPENDS
    "UserFiles/Bsp/*.c"
    "UserFiles/Bsp/*.cpp"
)

# Component 层
file(GLOB_RECURSE COMPONENT_SOURCES CONFIGURE_DEPENDS
    "UserFiles/Component/*.c"
    "UserFiles/Component/*.cpp"
)

# Algorithm 层
file(GLOB_RECURSE ALGORITHM_SOURCES CONFIGURE_DEPENDS
    "UserFiles/Algorithm/*.c"
    "UserFiles/Algorithm/*.cpp"
)

# Subsystem 层
file(GLOB_RECURSE SUBSYSTEM_SOURCES CONFIGURE_DEPENDS
    "UserFiles/Subsystem/*.c"
    "UserFiles/Subsystem/*.cpp"
)

# Application 层
file(GLOB_RECURSE APPLICATION_SOURCES CONFIGURE_DEPENDS
    "UserFiles/Application/*.c"
    "UserFiles/Application/*.cpp"
)

# 打印收集到的源文件数量（调试用）
list(LENGTH BSP_SOURCES BSP_COUNT)
list(LENGTH COMPONENT_SOURCES COMPONENT_COUNT)
list(LENGTH ALGORITHM_SOURCES ALGORITHM_COUNT)
list(LENGTH SUBSYSTEM_SOURCES SUBSYSTEM_COUNT)
list(LENGTH APPLICATION_SOURCES APPLICATION_COUNT)
message(STATUS "Source files collected:")
message(STATUS "  BSP:         ${BSP_COUNT} files")
message(STATUS "  Component:   ${COMPONENT_COUNT} files")
message(STATUS "  Algorithm:   ${ALGORITHM_COUNT} files")
message(STATUS "  Subsystem:   ${SUBSYSTEM_COUNT} files")
message(STATUS "  Application: ${APPLICATION_COUNT} files")

# 创建可执行文件

add_executable(${CMAKE_PROJECT_NAME}
    ${BSP_SOURCES}
    ${COMPONENT_SOURCES}
    ${ALGORITHM_SOURCES}
    ${SUBSYSTEM_SOURCES}
    ${APPLICATION_SOURCES}
)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
        # Add user defined library search paths
)

# 自动收集 Include 路径

# 自动扫描所有源文件所在的目录作为 include 路径
set(INCLUDE_DIRS_SET "")

# 收集所有源文件列表
set(ALL_USER_SOURCES
    ${BSP_SOURCES}
    ${COMPONENT_SOURCES}
    ${ALGORITHM_SOURCES}
    ${SUBSYSTEM_SOURCES}
    ${APPLICATION_SOURCES}
)

# 遍历所有源文件，提取所在目录
foreach (SOURCE_FILE ${ALL_USER_SOURCES})
    get_filename_component(SOURCE_DIR "${SOURCE_FILE}" DIRECTORY)

    # 如果目录不为空且不在已处理集合中，添加到包含路径
    if (NOT SOURCE_DIR STREQUAL "" AND NOT SOURCE_DIR IN_LIST INCLUDE_DIRS_SET)
        list(APPEND INCLUDE_DIRS_SET "${SOURCE_DIR}")
    endif ()
endforeach ()

# 添加收集到的 include 路径
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${INCLUDE_DIRS_SET}
)

# 添加 Core 层的目录
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Core/Inc
    Core/Src
)

# 打印 include 路径数量
list(LENGTH INCLUDE_DIRS_SET INCLUDE_COUNT)
message(STATUS "Include directories: ${INCLUDE_COUNT} paths")

# 其他配置

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
        # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
        stm32cubemx
        # Add user defined libraries
)

# 为 Clangd 和 VSCode IntelliSense 设置完整的编译命令
set(CMAKE_CXX_STANDARD_INCLUDE_DIRECTORIES
        ${CMAKE_CXX_IMPLICIT_INCLUDE_DIRECTORIES}
)
