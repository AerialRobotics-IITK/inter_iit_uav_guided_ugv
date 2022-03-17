cmake_minimum_required(VERSION 2.8.2)
project(CV)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(open_cv open_cv.cpp)
target_link_libraries(open_cv ${OpenCV_LIBS})

#add_executable(filters filters.cpp)
# target_link_libraries(filters ${OpenCV_LIBS})