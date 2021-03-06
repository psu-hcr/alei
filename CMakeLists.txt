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
	
add_executable(f8IK figure8_IK.cpp)
target_link_libraries(f8IK
	${ARMADILLO_LIBRARIES}
	${catkin_LIBRARIES}
	${std_msgs_LIBRARIES})
	
add_executable(f8IK_pos figure8_IK_pos.cpp)
target_link_libraries(f8IK_pos
	${ARMADILLO_LIBRARIES}
	${catkin_LIBRARIES}
	${std_msgs_LIBRARIES})

add_executable(dot_dklsac dklsac_dot.cpp)
target_link_libraries(dot_dklsac
	${ARMADILLO_LIBRARIES}
	${catkin_LIBRARIES}
	${std_msgs_LIBRARIES})
	
add_executable(IK_pos IK_pos.cpp)
target_link_libraries(IK_pos
	${ARMADILLO_LIBRARIES}
	${catkin_LIBRARIES}
	${std_msgs_LIBRARIES})
	
add_executable(IK_pos_cl IK_pos_cl.cpp)
target_link_libraries(IK_pos_cl
	${ARMADILLO_LIBRARIES}
	${catkin_LIBRARIES}
	${std_msgs_LIBRARIES})

add_executable(sac_koopquad errsac_koopquad.cpp)
target_link_libraries(sac_koopquad ${ARMADILLO_LIBRARIES})

add_executable(al_quad al-quad.cpp)
target_link_libraries(al_quad ${ARMADILLO_LIBRARIES})

add_executable(twolink errsac_KO_twolink.cpp)
target_link_libraries(twolink ${ARMADILLO_LIBRARIES})

add_executable(twolink_lqr_KO LQR_KO_twolink.cpp)
target_link_libraries(twolink_lqr_KO ${ARMADILLO_LIBRARIES})

add_executable(twolink_lqr LQR_twolink.cpp)
target_link_libraries(twolink_lqr ${ARMADILLO_LIBRARIES})

add_executable(KO_test KO_test.cpp)
target_link_libraries(KO_test ${ARMADILLO_LIBRARIES})

add_executable(KO_calc KO_calc.cpp)
target_link_libraries(KO_calc ${ARMADILLO_LIBRARIES})

add_executable(linear linear.cpp)
target_link_libraries(linear ${ARMADILLO_LIBRARIES})

add_executable(data2pdf data2pdftest.cpp)
target_link_libraries(data2pdf ${ARMADILLO_LIBRARIES})

add_executable(data2pdf_KL data2pdftest_KL.cpp)
target_link_libraries(data2pdf_KL ${ARMADILLO_LIBRARIES})

add_executable(data2pdf_auto data2pdftest_auto.cpp)
target_link_libraries(data2pdf_auto ${ARMADILLO_LIBRARIES})

add_executable(3point 3point_dot.cpp)
target_link_libraries(3point  ${ARMADILLO_LIBRARIES})

add_executable(k_medoid k_medoidstest.cpp)
target_link_libraries(k_medoid  ${ARMADILLO_LIBRARIES})

add_executable(dklsac_cube dklsac_cube.cpp)
target_link_libraries(dklsac_cube ${ARMADILLO_LIBRARIES})




