cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME Car_Control)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
        user/common/debug.h
        user/app/arm_debug.c)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)

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

# 递归收集 c/h文件

function(recursively_add_c_h root_dir OUT_SRCS OUT_INC_DIRS)
    if(NOT EXISTS "${root_dir}")
        message(WARNING "[recursively_add_c_h]目录不存在: ${root_dir}")
        set(${OUT_SRCS} "" PARENT_SCOPE)
        set(${OUT_INC_DIRS} "" PARENT_SCOPE)
        return()
    endif()
    file(GLOB_RECURSE c_h_files CONFIGURE_DEPENDS
            "${root_dir}/*.c"
            "${root_dir}/*.h"
    )
    set(src_list "")
    set(inc_dirs "")
    foreach(f ${c_h_files})
        get_filename_component(dir "${f}" DIRECTORY)
        list(APPEND inc_dirs "${dir}")
        if(f MATCHES "\\.c$")
            list(APPEND src_list "${f}")
        endif()
    endforeach()
    list(REMOVE_DUPLICATES inc_dirs)
    set(${OUT_SRCS} "${src_list}" PARENT_SCOPE)
    set(${OUT_INC_DIRS} "${inc_dirs}" PARENT_SCOPE)
endfunction()
# 扫描user目录

recursively_add_c_h(${CMAKE_CURRENT_SOURCE_DIR}/user USER_SOURCES USER_INCS)
message(STATUS "USER_SOURCES = ${USER_SOURCES}")
message(STATUS "USER_INCS = ${USER_INCS}")
if(USER_SOURCES)
    # 给主目标添加源码文件

    target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${USER_SOURCES})
    # 把user所有子目录加入主目标头文件搜索路径

    target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${USER_INCS})
else()
    message(STATUS "user目录没有找到.c文件，跳过user源码")
endif()