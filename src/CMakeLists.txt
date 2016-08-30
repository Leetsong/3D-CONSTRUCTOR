cmake_minimum_required (VERSION 2.8.11 FATAL_ERROR)

project(3D-CONSTRUCTOR)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(KINECT_INCLUDE_DIRS $ENV{KINECTSDK20_DIR}/inc)
set(KINECT_LIBRARY_DIRS $ENV{KINECTSDK20_DIR}/Lib/x64)
set(KINECT_LIBRARIES ${KINECT_LIBRARY_DIRS}/kinect20.lib)

find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(VTK REQUIRED)
find_package(PCL 1.8 REQUIRED io filters registration visualization)

include_directories (${PCL_INCLUDE_DIRS} ${KINECT_INCLUDE_DIRS})
link_directories    (${PCL_LIBRARY_DIRS} ${KINECT_LIBRARY_DIRS})
add_definitions     (${PCL_DEFINITIONS})
add_definitions     (${QT_DEFINITIONS})

set(PROJECT_NEED_MOCING_HEADERS wrapper.h controller.h openglviewer.h kinectreceiver.h pclregister.h)
set(PROJECT_FORMS wrapper.ui)

qt5_wrap_cpp(PROJECT_MOC_HEADERS ${PROJECT_NEED_MOCING_HEADERS})
qt5_wrap_ui(PROJECT_FORMS_HEADERS ${PROJECT_FORMS})

set(PROJECT_SOURCES main.cpp wrapper.cpp controller.cpp openglviewer.cpp kinectreceiver.cpp pclregister.cpp)
set(PROJECT_HEADERS common.h ${PROJECT_MOC_HEADERS} ${PROJECT_FORMS_HEADERS})

add_executable(3D-CONSTRUCTOR ${PROJECT_SOURCES} ${PROJECT_HEADERS})
target_link_libraries (3D-CONSTRUCTOR Qt5::Core Qt5::Gui Qt5::Widgets ${VTK_LIBRARIES} ${KINECT_LIBRARIES} ${PCL_LIBRARIES})
