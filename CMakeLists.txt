cmake_minimum_required(VERSION 2.6)

PROJECT(CompetitionED3D)

set(OSGDB_LIBRARY /usr/local/lib64/libosgDB.so)
set(OSGUTIL_LIBRARY /usr/local/lib64/libosgUtil.so)
set(OSGVIEWER_LIBRARY /usr/local/lib64/libosgViewer.so)
set(OSG_LIBRARY /usr/local/lib64/libosg.so)
set(OSGGA_LIBRARY /usr/local/lib64/libosgGA.so)
set(OPENTHREADS_LIBRARY /usr/local/lib64/libOpenThreads.so)

#FIND_PACKAGE(OpenThreads)
FIND_PACKAGE(OpenGL)
FIND_PACKAGE(osg)
FIND_PACKAGE(osgDB)
FIND_PACKAGE(osgViewer)
FIND_PACKAGE(osgGA)
FIND_PACKAGE(osgUtil)
FIND_PACKAGE(OpenCV)
find_package( PCL 1.2 REQUIRED )


SET(HEADERS 
   src/Scanner.h
   src/ScreenCapture.h
   src/ImageProcessing.h
)

SET(SOURCES
src/main.cpp
src/Scanner.cpp
src/ScreenCapture.cpp
src/ImageProcessing.cpp
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(${OSG_INCLUDE_DIR} ${OSGDB_INCLUDE_DIR} ${OSGUTIL_INCLUDE_DIR} ${OSGVIEWER_INCLUDE_DIR} ${OSGGA_INCLUDE_DIR} ${PCL_INCLUDE_DIRS})


LINK_DIRECTORIES(${OSG_LIB_DIRS} ${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


ADD_EXECUTABLE(main ${SOURCES})

TARGET_LINK_LIBRARIES(main ${OSG_LIBRARY} ${OSGDB_LIBRARY} ${OSGVIEWER_LIBRARY} ${OSGGA_LIBRARY} ${OSGUTIL_LIBRARY} ${OPENTHREADS_LIBRARY} ${OPENGL_LIBRARY} ${OpenCV_LIBS} ${PCL_LIBRARIES}) 