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

SET(HEADERS 
   src/Scanner.h
   src/ScreenCapture.h
)

SET(SOURCES
src/main.cpp
src/Scanner.cpp
src/ScreenCapture.cpp
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(${OSG_INCLUDE_DIR} ${OSGDB_INCLUDE_DIR} ${OSGUTIL_INCLUDE_DIR} ${OSGVIEWER_INCLUDE_DIR} ${OSGGA_INCLUDE_DIR})

LINK_DIRECTORIES(${OSG_LIB_DIRS})

ADD_EXECUTABLE(main ${SOURCES})

TARGET_LINK_LIBRARIES(main ${OSG_LIBRARY} ${OSGDB_LIBRARY} ${OSGVIEWER_LIBRARY} ${OSGGA_LIBRARY} ${OSGUTIL_LIBRARY} ${OPENTHREADS_LIBRARY} ${OPENGL_LIBRARY}) 