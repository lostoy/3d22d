cmake_minimum_required(VERSION 2.8)
project(3d22d)
set(BOOST_LIBRARYDIR C:/local/boost_1_55_0/lib64-msvc-11.0)
set(BOOST_INCLUDEDIR C:/local/boost_1_55_0)

set(Boost_USE_STATIC_LIBS ON)
set(Boost_USE_STATIC ON)
set(Boost_USE_MULTITHREAD )
set(Boost_NO_BOOST_CMAKE ON)

find_package(Boost COMPONENTS filesystem system REQUIRED)


find_package(PCL 1.2 REQUIRED COMPONENTS common io io_ply filters)

#set(OpenCV_STATIC ON)
#set(Qt_DIR c:/Qt/4.8.0)
#find_package( OpenCV REQUIRED COMPONENTS core features2d highgui #nonfree imgproc gpu)


include_directories(${Boost_INCLUDE_DIRS} "C:/Program Files (x86)/Eigen/include" ${FLANN_INCLUDE_DIRS})


set(
srcs src/3d22d.cpp
)  


add_definitions(${PCL_DEFINITIONS})

include_directories(${PCL_INCLUDE_DIRS} ${VISION_INCLUDE_DIRS})
add_executable(3d22d ${srcs} )

target_link_libraries(3d22d ${Boost_LIBRARIES} ${PCL_LIBRARIES} )


