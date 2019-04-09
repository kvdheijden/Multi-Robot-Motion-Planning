cmake_minimum_required(VERSION 3.10)
project("Multi-Robot Motion Planning" LANGUAGES CXX VERSION 1.0.0.0)
set(CMAKE_AUTOMOC ON)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Og")
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -Og")

# CGAL and its components
find_package(CGAL COMPONENTS Core Qt5 REQUIRED)
include(${CGAL_USE_FILE})

# Boost and its components
find_package(Boost COMPONENTS filesystem REQUIRED)
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
        src/mrmp.cpp
        src/MainWindow.cpp
        ${CGAL_Qt5_UI_FILES}
        ${CGAL_Qt5_RESOURCE_FILES}
        ${CGAL_Qt5_MOC_FILES})

qt5_use_modules(mrmp Xml Script OpenGL Svg)
add_to_cached_list(CGAL_EXECUTABLE_TARGETS mrmp)
target_link_libraries(mrmp ${QT_LIBRARIES} ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES} ${Boost_LIBRARIES})
