cmake_minimum_required(VERSION 3.10)
project("Multi-Robot Motion Planning" LANGUAGES CXX VERSION 1.0.0.0)
set(CMAKE_AUTOMOC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# CGAL and its components
find_package(CGAL COMPONENTS Core Qt5 REQUIRED)
include(${CGAL_USE_FILE})

# Boost and its components
find_package(Boost COMPONENTS filesystem unit_test_framework REQUIRED)
include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIRS})

# Qt5 and its components
find_package(Qt5 COMPONENTS Xml Script OpenGL Svg REQUIRED)

# Add ui files to end of list
qt5_wrap_ui(CGAL_Qt5_UI_FILES src/MainWindow.ui)

# Add qrc files to end of list
qt5_add_resources(CGAL_Qt5_RESOURCE_FILES)

# Create the library and link it
add_library(mrmp
        src/Configuration.cpp
        src/ConfigurationSet.cpp
        src/FreeSpace.cpp
        src/mrmp.cpp)

qt5_use_modules(mrmp Xml Script OpenGL Svg)
add_to_cached_list(CGAL_EXECUTABLE_TARGETS mrmp)
target_link_libraries(mrmp ${QT_LIBRARIES} ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES} ${Boost_LIBRARIES})

add_executable(mrmp_gui
        src/main.cpp
        src/WorkspaceGraphicsItem.cpp
        src/ConfigurationSetGraphicsItem.cpp
        src/FreeSpaceGraphicsItem.cpp
        src/PathGraphicsItem.cpp
        src/MainWindow.cpp
        ${CGAL_Qt5_UI_FILES}
        ${CGAL_Qt5_RESOURCE_FILES}
        ${CGAL_Qt5_MOC_FILES})
target_link_libraries(mrmp_gui mrmp)

add_library(mrmp_benchmark test/benchmark.cpp)
target_include_directories(mrmp_benchmark PRIVATE src)
target_link_libraries(mrmp_benchmark mrmp)

file(GLOB BENCHMARKS "benchmark/*.cpp")
foreach(BENCHMARK ${BENCHMARKS})
    get_filename_component(BENCHMARK_NAME ${BENCHMARK} NAME_WE)
    set(exec mrmp_${BENCHMARK_NAME})
    add_executable(${exec} benchmark/${BENCHMARK_NAME}.cpp)
    target_include_directories(${exec} PRIVATE test src)
    target_link_libraries(${exec} mrmp_benchmark mrmp)
endforeach()

#add_executable(mrmp_benchmark_grid_20
#        benchmark/benchmark_grid_20.cpp
#        test/benchmark.cpp)
#target_include_directories(mrmp_benchmark_grid_20 PRIVATE src)
#target_link_libraries(mrmp_benchmark_grid_20 mrmp)
#
#add_executable(mrmp_benchmark_grid_40
#        benchmark/benchmark_grid_40.cpp
#        test/benchmark.cpp)
#target_include_directories(mrmp_benchmark_grid_40 PRIVATE src)
#target_link_libraries(mrmp_benchmark_grid_40 mrmp)
#
#add_executable(mrmp_benchmark_grid_diagonal
#        benchmark/benchmark_grid_diagonal.cpp
#        test/benchmark.cpp)
#target_include_directories(mrmp_benchmark_grid_diagonal PRIVATE src)
#target_link_libraries(mrmp_benchmark_grid_diagonal mrmp)
#
#add_executable(mrmp_benchmark_small_corridor
#        benchmark/benchmark_small_corridor.cpp
#        test/benchmark.cpp)
#target_include_directories(mrmp_benchmark_small_corridor PRIVATE src)
#target_link_libraries(mrmp_benchmark_small_corridor mrmp)
#
#add_executable(mrmp_benchmark_wide_DIF
#        benchmark/benchmark_wide_DIF.cpp
#        test/benchmark.cpp)
#target_include_directories(mrmp_benchmark_wide_DIF PRIVATE src)
#target_link_libraries(mrmp_benchmark_wide_DIF mrmp)
#
#add_executable(mrmp_benchmark_deep_DIF
#        benchmark/benchmark_deep_DIF.cpp
#        test/benchmark.cpp)
#target_include_directories(mrmp_benchmark_deep_DIF PRIVATE src)
#target_link_libraries(mrmp_benchmark_deep_DIF mrmp)
