cmake_minimum_required(VERSION 3.10)
project("Multi-Robot Motion Planning" LANGUAGES CXX VERSION 1.0.0.0)
set(CMAKE_AUTOMOC ON)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0")
add_definitions(-DCGAL_DISABLE_ROUNDING_MATH_CHECK=ON)

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
add_executable(mrmp
        src/main.cpp
        src/WorkspaceGraphicsItem.cpp
        src/Configuration.cpp
        src/ConfigurationSet.cpp
        src/ConfigurationSetGraphicsItem.cpp
        src/FreeSpace.cpp
        src/FreeSpaceGraphicsItem.cpp
        src/MainWindow.cpp
        src/mrmp.cpp
        ${CGAL_Qt5_UI_FILES}
        ${CGAL_Qt5_RESOURCE_FILES}
        ${CGAL_Qt5_MOC_FILES})

add_executable(mrmp_test
        test/main.cpp
        src/Configuration.cpp
        src/ConfigurationSet.cpp
        src/FreeSpace.cpp
        src/mrmp.cpp)

qt5_use_modules(mrmp Xml Script OpenGL Svg)
add_to_cached_list(CGAL_EXECUTABLE_TARGETS mrmp)
target_link_libraries(mrmp ${QT_LIBRARIES} ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES} ${Boost_LIBRARIES})

add_to_cached_list(CGAL_EXECUTABLE_TARGETS mrmp_test)
target_compile_definitions(mrmp_test PUBLIC BOOST_TEST_DYN_LINK)
target_link_libraries(mrmp_test ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES} ${Boost_LIBRARIES})
