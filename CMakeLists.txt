cmake_minimum_required(VERSION 3.15)
project(SimuSherpaConsole C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Find packages
find_package(OpenGL REQUIRED)
find_package(GSL REQUIRED)
find_package(SDL REQUIRED)

# Add GSL, SDL, OpenGL include directories
include_directories(
    src
    src/ezxml
    ${GSL_INCLUDE_DIRS}
    ${SDL_INCLUDE_DIR}
)

# Collect root C source files and ezxml library sources
file(GLOB ROOT_SOURCES "src/*.c")
set(SOURCES
    ${ROOT_SOURCES}
    src/ezxml/ezxml.c
)

# Optimization compile flags and definitions
add_compile_definitions(HAVE_INLINE GSL_RANGE_CHECK_OFF)

# Set compile options for MinGW
if(CMAKE_COMPILER_IS_GNUCC)
    add_compile_options(-O3 -msse2)
endif()

# Executable target
add_executable(SimuSherpaConsole ${SOURCES})

# Target include directories
target_include_directories(SimuSherpaConsole PRIVATE
    ${GSL_INCLUDE_DIRS}
    ${SDL_INCLUDE_DIR}
)

# Target link libraries
target_link_libraries(SimuSherpaConsole PRIVATE
    ${GSL_LIBRARIES}
    ${SDL_LIBRARY}
    OpenGL::GL
    OpenGL::GLU
)

# Link Winsock on Windows
if(WIN32)
    target_link_libraries(SimuSherpaConsole PRIVATE wsock32 ws2_32)
endif()
