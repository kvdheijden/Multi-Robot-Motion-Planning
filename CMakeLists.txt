cmake_minimum_required(VERSION 3.10)
project("Multi-Robot Motion Planning" LANGUAGES CXX VERSION 1.0.0.0)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror -Wall -Wextra")

# CGAL and its components
find_package(CGAL COMPONENTS Core Qt5 REQUIRED)
include(${CGAL_USE_FILE})

# Boost and its components
find_package(Boost COMPONENTS filesystem unit_test_framework REQUIRED)
include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIRS})

# Qt5 and its components
find_package(Qt5 COMPONENTS Core Gui REQUIRED)
include_directories(${Qt5Core_INCLUDE_DIRS} ${Qt5Gui_INCLUDE_DIRS})
add_definitions(${Qt5Core_DEFINITIONS} ${Qt5Gui_DEFINITIONS})

## Add ui files to end of list
#qt5_wrap_ui(CGAL_Qt5_UI_FILES src/MainWindow.ui)
#
## Add qrc files to end of list
#qt5_add_resources(CGAL_Qt5_RESOURCE_FILES)

add_executable(generate final/generate.cpp final/free_space.cpp final/motion_graph.cpp final/Configuration.cpp)

add_executable(benchmark final/benchmark.cpp final/free_space.cpp final/motion_graph.cpp final/Configuration.cpp final/edge_weight.cpp final/interference_forest.cpp)
target_link_libraries(benchmark ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES} ${Boost_LIBRARIES})

add_executable(draw final/draw.cpp final/free_space.cpp final/motion_graph.cpp final/Configuration.cpp)
target_link_libraries(draw ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES} ${Boost_LIBRARIES} ${Qt5Core_LIBRARIES} ${Qt5Gui_LIBRARIES})
target_compile_definitions(draw PUBLIC ${Qt5Core_COMPILE_DEFINITIONS} ${Qt5Gui_COMPILE_DEFINITIONS})

## Create the library and link it
#add_library(mrmp
#        src/Configuration.cpp
#        src/ConfigurationSet.cpp
##        src/FreeSpace.cpp
#        src/edge_weight.cpp
#        src/interference_forest.cpp
#        src/motion_graph.cpp
#        src/free_space.cpp
#        src/mrmp.cpp)
#
#qt5_use_modules(mrmp Xml Script OpenGL Svg)
#add_to_cached_list(CGAL_EXECUTABLE_TARGETS mrmp)
#target_link_libraries(mrmp ${QT_LIBRARIES} ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES} ${Boost_LIBRARIES})
#
#add_executable(mrmp_gui
#        src/main.cpp
#        src/WorkspaceGraphicsItem.cpp
#        src/ConfigurationSetGraphicsItem.cpp
#        src/FreeSpaceGraphicsItem.cpp
#        src/PathGraphicsItem.cpp
#        src/MainWindow.cpp
#        ${CGAL_Qt5_UI_FILES}
#        ${CGAL_Qt5_RESOURCE_FILES}
#        ${CGAL_Qt5_MOC_FILES})
#target_link_libraries(mrmp_gui mrmp)
#
#add_library(mrmp_benchmark test/benchmark.cpp)
#target_include_directories(mrmp_benchmark PRIVATE src)
#target_link_libraries(mrmp_benchmark mrmp)
#
#file(GLOB BENCHMARKS /MainWindow.ui)
#
## Add qrc files to end of list
#qt5_add_resources(CGAL_Qt5_RESOURCE_FILES)
