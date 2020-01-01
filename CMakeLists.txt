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
find_package(CGAL REQUIRED COMPONENTS Core Qt5 PATHS "/home/koen/CGAL-4.14/build" NO_DEFAULT_PATH REQUIRED)
include(${CGAL_USE_FILE})

add_definitions(-DCGAL_USE_BASIC_VIEWER -DQT_NO_KEYWORDS)
message("${CGAL_LIBRARIES};${CGAL_3RD_PARTY_LIBRARIES}")

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

add_library(generate final/free_space.cpp final/motion_graph.cpp final/Configuration.cpp)

add_executable(generate_1 final/generate_1.cpp)
target_link_libraries(generate_1 generate ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES} ${Boost_LIBRARIES})

add_executable(generate_2 final/generate_2.cpp)
target_link_libraries(generate_2 generate ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES} ${Boost_LIBRARIES})

add_executable(generate_3 final/generate_3.cpp)
target_link_libraries(generate_3 generate ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES} ${Boost_LIBRARIES})

add_executable(generate_4 final/generate_4.cpp)
target_link_libraries(generate_4 generate ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES} ${Boost_LIBRARIES})

add_executable(generate_5 final/generate_5.cpp)
target_link_libraries(generate_5 generate ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES} ${Boost_LIBRARIES})

add_executable(benchmark final/benchmark.cpp final/edge_weight.cpp final/interference_forest.cpp)
target_link_libraries(benchmark generate ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES} ${Boost_LIBRARIES})

add_executable(draw final/draw.cpp)
target_link_libraries(draw generate ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES} ${Boost_LIBRARIES} ${Qt5Core_LIBRARIES} ${Qt5Gui_LIBRARIES})
target_compile_definitions(draw PUBLIC ${Qt5Core_COMPILE_DEFINITIONS} ${Qt5Gui_COMPILE_DEFINITIONS})
