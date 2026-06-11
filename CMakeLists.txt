cmake_minimum_required(VERSION 3.15)
project(SimuSherpaConsole C CXX)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find packages
find_package(OpenGL REQUIRED)
find_package(GSL REQUIRED)
find_package(SDL2 REQUIRED)
find_package(pugixml REQUIRED)
find_package(GLEW REQUIRED)
find_package(glm REQUIRED)

# Add include directories
include_directories(
    src
    src/math
    src/renderer
    src/simulation
    src/control
    src/utils
    ${GSL_INCLUDE_DIRS}
    ${SDL2_INCLUDE_DIRS}
    ${GLEW_INCLUDE_DIRS}
)

# Collect C++ source files recursively
file(GLOB_RECURSE SOURCES "src/*.cpp")

# Optimization compile flags and definitions
add_compile_definitions(HAVE_INLINE GSL_RANGE_CHECK_OFF)

# Set compile options for MinGW
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_COMPILER_IS_GNUCC)
    add_compile_options(-O3 -msse2 -fcommon)
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
    SDL2::SDL2main
    SDL2::SDL2
    OpenGL::GL
    GLEW::GLEW
    glm::glm
    pugixml::pugixml
)

# Link Winsock on Windows
if(WIN32)
    target_link_libraries(SimuSherpaConsole PRIVATE wsock32 ws2_32)
endif()
