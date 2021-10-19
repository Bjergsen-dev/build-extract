cmake_minimum_required( VERSION 2.8 )
project( 3D_rebuild )

set(CMAKE_CXX_FLAGS "-std=c++11")

#build type
set(CMAKE_BUILD_TYPE "Release")


# 添加GDAL依赖
find_package( GDAL 2.4.2 REQUIRED )
include_directories( ${GDAL_INCLUDE_DIRS} )
link_directories( ${GDAL_LIBRARY_DIRS})
add_definitions(${GDAL_DEFINITIONS})

# 添加OpenCV依赖
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories( ${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})


## 添加PCL依赖
find_package(PCL 1.11 REQUIRED)
include_directories( ${PCL_INCLUDE_DIRS} )
link_directories( ${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

## 添加jsoncpp依赖
set(lib_DIR /usr/lib/x84_64-linux-gnu)
link_directories(${lib_DIR})


## 添加eigen3依赖
include_directories( "/usr/include/eigen3" )

include_directories(includes) #添加头文件
 
FILE(GLOB_RECURSE CURRENT_INCLUDE includes/*.hpp)
FILE(GLOB_RECURSE CURRENT_SOURCES src/*.cpp)     #aux_source_directory(src CURRENT_SOURCES)



add_executable(checkCornerPoi ${CURRENT_SOURCES} ${CURRENT_INCLUDE})  #制定生成目标
target_link_libraries( checkCornerPoi ${OpenCV_LIBRARIES} ${GDAL_LIBRARIES} ${PCL_LIBRARIES} libjsoncpp.a)
