cmake_minimum_required(VERSION 2.6)

project(CompetitionED3D)

find_package(OpenThreads)
find_package(OpenGL)
find_package(osg)
find_package(osgDB)
find_package(osgViewer)
find_package(osgGA)
find_package(osgUtil)
find_package(OpenCV)
find_package(PCL)

# Please insert the correct path to these libraries if they are not found automatically
if(NOT ${OPENTHREADS_FOUND})
	set(OPENTHREADS_LIBRARY /usr/local/lib64/libOpenThreads.so)
	message(STATUS Found\ OPENTHREADS_LIBRARY)
endif()
if(NOT ${OSG_FOUND})
	set(OSG_LIBRARY /usr/local/lib64/libosg.so)
	message(STATUS Found\ OSG_LIBRARY)
endif()
if(NOT ${OSGDB_FOUND})
	set(OSGDB_LIBRARY /usr/local/lib64/libosgDB.so)
	message(STATUS Found\ OSGDB_LIBRARY)
endif()
if(NOT ${OSGVIEWER_FOUND})
	set(OSGVIEWER_LIBRARY /usr/local/lib64/libosgViewer.so)
	message(STATUS Found\ OSGVIEWER_LIBRARY)
endif()
if(NOT ${OSGGA_FOUND})
	set(OSGGA_LIBRARY /usr/local/lib64/libosgGA.so)
	message(STATUS Found\ OSGGA_LIBRARY)
endif()
if(NOT ${OSGUTIL_FOUND})
	set(OSGUTIL_LIBRARY /usr/local/lib64/libosgUtil.so)
	message(STATUS Found\ OGSUTIL_LIBRARY)
endif()

set(HEADERS 
   src/Scanner.h
   src/ImageProcessing.h
)

set(SOURCES
	src/main.cpp
	src/Scanner.cpp
	src/ImageProcessing.cpp
)

#set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall")

include_directories(${OSG_INCLUDE_DIR} ${OSGDB_INCLUDE_DIR} ${OSGUTIL_INCLUDE_DIR} ${OSGVIEWER_INCLUDE_DIR} ${OSGGA_INCLUDE_DIR} ${PCL_INCLUDE_DIRS})


link_directories(${OSG_LIB_DIRS} ${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(LaserScanner ${SOURCES})

target_link_libraries(LaserScanner ${OSG_LIBRARY} ${OSGDB_LIBRARY} ${OSGVIEWER_LIBRARY} ${OSGGA_LIBRARY} ${OSGUTIL_LIBRARY} ${OPENTHREADS_LIBRARY} ${OPENGL_LIBRARY} ${OpenCV_LIBS} ${PCL_LIBRARIES}) 