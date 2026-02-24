cmake_minimum_required(VERSION 3.16)
project(MocapPlayer)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# FLTK root (relative to this project)
set(FLTK_ROOT ${CMAKE_SOURCE_DIR}/../fltk-1.4.4)

# Headers
include_directories(${FLTK_ROOT})
link_directories(${FLTK_ROOT}/lib)

set(ENGINE_CORE_SOURCES
        Engine/core/motion.cpp
        Engine/core/posture.cpp
        Engine/core/skeleton.cpp
        Engine/core/transform.cpp
        Engine/core/vector.cpp
        Engine/core/quaternion.cpp
)

add_library(engine_core STATIC ${ENGINE_CORE_SOURCES})

set(PLAYER_SOURCES
        Engine/player/displaySkeleton.cpp
        Engine/player/interface.cpp
        Engine/player/mocapPlayer.cpp
        Engine/player/ppm.cpp
        Engine/player/pic.cpp
        Engine/player/performanceCounter.cpp
)

set(INTERPOLATE_SOURCES
        Engine/interpolate/interpolator.cpp
        Engine/interpolate/interpolate.cpp
)

target_include_directories(engine_core PUBLIC
        ${CMAKE_SOURCE_DIR}/Engine/core
)

add_executable(mocapPlayer ${PLAYER_SOURCES})
target_link_libraries(mocapPlayer engine_core)

add_executable(interpolate ${INTERPOLATE_SOURCES})
target_link_libraries(interpolate engine_core)

# macOS linking (NO X11)
target_link_libraries(mocapPlayer
        fltk
        fltk_images
        fltk_gl
        "-framework Cocoa"
        "-framework OpenGL"
        "-framework ScreenCaptureKit"
)

target_link_libraries(interpolate
        fltk
        fltk_images
        fltk_gl
        "-framework Cocoa"
        "-framework OpenGL"
        "-framework ScreenCaptureKit"
)