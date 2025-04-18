cmake_minimum_required(VERSION 3.0.0)
project(QUADRUPED_MPC)  # 可以根据实际项目名修改

find_package(pinocchio REQUIRED)
find_package(aligator REQUIRED)
find_package(proxsuite-nlp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Python3 REQUIRED)

# 获取 C++ 源文件
file(GLOB_RECURSE SRC_LIST 
    ./src/*.cpp
)

# 获取 C++ 头文件
include_directories(include/common include)

# 设置目标可执行文件
add_executable(${PROJECT_NAME} ${SRC_LIST})

# 添加 pinocchio 相关的头文件
include_directories(${PINOCCHIO_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

# 链接动态库
target_link_libraries(${PROJECT_NAME} pinocchio::pinocchio)
target_link_libraries(${PROJECT_NAME} aligator::aligator)
target_link_libraries(${PROJECT_NAME} proxsuite-nlp)

add_custom_command(
        TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/result_visual.py
 )