cmake_minimum_required(VERSION 2.8.3)
project(alei)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(Armadillo REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED)

if(OPENMP_FOUND)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

add_definitions(-std=c++14)

find_package(catkin REQUIRED COMPONENTS std_msgs sensor_msgs roscpp
	iiwa_tools
	)
	
catkin_package(CATKIN_DEPENDS std_msgs sensor_msgs roscpp
	iiwa_tools
	)
 
set(EXECUTABLE_OUTPUT_PATH build/)
include_directories(${ARMADILLO_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${OpenMP_INCLUDE_DIRS} "/home/zxl5344/IROS2022/src/alei")
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS})

set(EXECUTABLE_OUTPUT_PATH build/)

add_executable(errsac_cp errsac_cartpend.cpp)
target_link_libraries(errsac_cp ${ARMADILLO_LIBRARIES})

add_executable(figure8 figure8.cpp)
target_link_libraries(figure8 
	${ARMADILLO_LIBRARIES}
	${catkin_LIBRARIES}
	${std_msgs_LIBRARIES})
	
add_executable(f8lqr figure8_lqr.cpp)
target_link_libraries(f8lqr
	${ARMADILLO_LIBRARIES}
	${catkin_LIBRARIES}
	${std_msgs_LIBRARIES})

add_executable(sac_koopquad errsac_koopquad.cpp)
target_link_libraries(sac_koopquad ${ARMADILLO_LIBRARIES})

add_executable(al_quad al-quad.cpp)
target_link_libraries(al_quad ${ARMADILLO_LIBRARIES})

	




