cmake_minimum_required(VERSION 2.8.3)
project(rgbd_detect)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    message_filters
    cv_bridge
    image_transport
    tf
        )

find_package(OpenCV REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})

# find_package(Boost REQUIRED COMPONENTS system)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rgbd_detect
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
        /home/jieming/libs/OOQP-0.99.26/include
)

add_executable(${PROJECT_NAME}_node src/main.cpp src/LaserRecognizer.cpp src/LaserRecognizer.h src/YoloRecognizer.cpp src/YoloRecognizer.h src/KalmanFilter.cpp src/KalmanFilter.h src/Hungarian.cpp src/Hungarian.h src/SORT.cpp src/SORT.h )
#add_executable(${PROJECT_NAME}_node src/main2.cpp )


target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        Eigen3::Eigen
        /home/jieming/car_ws/src/rgbd_detect/darknet/libdarknet.so
        ooqpgensparse
        ooqpsparse
        ooqpgondzio
        ooqpbase  blas ma27 gfortran
        )

