cmake_minimum_required(VERSION 3.10)
project(SimUnderwater)

set(CMAKE_CXX_STANDARD 14)

find_package(Qt5 COMPONENTS Core Widgets OpenGL REQUIRED)

find_package(GDAL REQUIRED)

find_package(OpenSceneGraph REQUIRED osgDB osgUtil osgGA osgViewer)

add_executable(SimUnderwater main.cpp depth.cpp rgb.cpp box_visitor.cpp node_user_data.h camera_model.cpp camera_model.h water_model.cpp water_model.h light_model.cpp light_model.h equations.cpp equations.h)

target_link_libraries(SimUnderwater
        Qt5::Core
        Qt5::Widgets
        Qt5::OpenGL
        ${GDAL_LIBRARY}
        ${OPENSCENEGRAPH_LIBRARIES})
