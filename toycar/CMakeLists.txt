cmake_minimum_required(VERSION 2.8.3)

add_compile_options(-std=c++11)

project(toycar)

catkin_package()

find_package(catkin REQUIRED COMPONENTS roslaunch tf nav_msgs roscpp rospy std_msgs
geometry_msgs )

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)


add_executable(toycar_publisher src/ToyCar.cpp src/main.cpp)

include_directories(include ${catkin_INCLUDE_DIRS})

## Specify libraries to link a library or executable target against
target_link_libraries(toycar_publisher ${catkin_LIBRARIES})
