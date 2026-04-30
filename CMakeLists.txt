cmake_minimum_required(VERSION 3.16)
project(franka_haply_teleop)

set(CMAKE_CXX_STANDARD 17)

if(APPLE)
    set(MUJOCO_PATH "$ENV{HOME}/mujoco/Contents/Frameworks/mujoco.framework")
    set(MUJOCO_INCLUDE "$ENV{HOME}/mujoco_include")

    include_directories(${MUJOCO_INCLUDE})
    include_directories(/usr/local/Cellar/eigen/5.0.1/include/eigen3)
    include_directories(/usr/local/Cellar/libwebsockets/4.5.8/include)

    link_directories(${MUJOCO_PATH}/Versions/A)
    link_directories(/usr/local/Cellar/libwebsockets/4.5.8/lib)

    add_executable(teleop main.cpp)
    target_link_libraries(teleop
        ${MUJOCO_PATH}/Versions/A/libmujoco.3.7.0.dylib
        glfw
        websockets
    )

elseif(UNIX)
    set(MUJOCO_PATH "$ENV{HOME}/mujoco")

    include_directories(${MUJOCO_PATH}/include)
    include_directories(/usr/include/eigen3)

    find_package(nlohmann_json REQUIRED)

    link_directories(${MUJOCO_PATH}/lib)

    add_executable(teleop main.cpp)
    target_link_libraries(teleop
        mujoco
        glfw
        websockets
        nlohmann_json::nlohmann_json
    )
endif()