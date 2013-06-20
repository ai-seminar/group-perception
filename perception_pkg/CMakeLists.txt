cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()
find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})
include_directories(/usr/include/vtk-5.8)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_executable(perception_server src/perception_server.cpp)
target_link_libraries(perception_server libvtkCommon.so libvtkFiltering.so libvtkRendering.so)
target_link_libraries(perception_server ${PCL_LIBRARIES})
rosbuild_link_boost(perception_server thread)

rosbuild_add_executable(pcd_processor src/pcd_processor.cpp)
target_link_libraries(pcd_processor libvtkCommon.so libvtkFiltering.so libvtkRendering.so)

rosbuild_add_executable(pcd_view src/pcd_view.cpp)
target_link_libraries(pcd_view libvtkCommon.so libvtkFiltering.so libvtkRendering.so)

rosbuild_add_executable(concave_hull src/concave_hull.cpp)
target_link_libraries(concave_hull libvtkCommon.so libvtkFiltering.so libvtkRendering.so)

rosbuild_add_executable(cluster_extraction src/cluster_extraction.cpp)
target_link_libraries(cluster_extraction libvtkCommon.so libvtkFiltering.so libvtkRendering.so)

rosbuild_add_executable(objects_on_table src/objects_on_table.cpp)
target_link_libraries(objects_on_table libvtkCommon.so libvtkFiltering.so libvtkRendering.so)
target_link_libraries(objects_on_table ${PCL_LIBRARIES})

rosbuild_add_executable(subscribe src/subscribe.cpp)
rosbuild_link_boost(subscribe system)
target_link_libraries(subscribe ${PCL_LIBRARIES})
