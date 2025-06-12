# cmake_minimum_required(VERSION 3.0.0)
# project(QUADRUPED_MPC) # 可以根据实际项目名修改

# # set(CMAKE_SKIP_RPATH TRUE)

# find_package(pinocchio REQUIRED)
# find_package(aligator REQUIRED)
# find_package(proxsuite-nlp REQUIRED)
# find_package(Eigen3 REQUIRED)
# find_package(Python3 REQUIRED)
# find_package(yaml-cpp REQUIRED)
# find_package(proxsuite REQUIRED)  
# find_package(qpOASES REQUIRED)


# set(WEBOTS_HOME "/usr/local/webots")
# set(WEBOTSLIBRARIES m ${CMAKE_SHARED_LIBRARY_PREFIX}CppController${CMAKE_SHARED_LIBRARY_SUFFIX})
# link_directories(${WEBOTS_HOME}/lib/controller)
# include_directories(${WEBOTS_HOME}/include/controller/cpp)

# # 获取 C++ 源文件
# file(GLOB_RECURSE SRC_LIST
#     ./src/*.cpp
#     ./include/*.txx
#     ./include/*.hxx
# )

# # 获取 C++ 头文件
# # include_directories(include/common include/mpc include/utils include/wbc include/object_qp)
# include_directories(include)

# # 设置目标可执行文件
# add_executable(${PROJECT_NAME} ${SRC_LIST})

# # 添加 pinocchio 相关的头文件
# include_directories(${PINOCCHIO_INCLUDE_DIRS})
# include_directories(${EIGEN3_INCLUDE_DIR})
# include_directories(${PROXSUITE_INCLUDE_DIRS})

# # 链接动态库
# target_link_libraries(${PROJECT_NAME} pinocchio::pinocchio)
# target_link_libraries(${PROJECT_NAME} aligator::aligator)
# target_link_libraries(${PROJECT_NAME} proxsuite-nlp)
# target_link_libraries(${PROJECT_NAME} yaml-cpp::yaml-cpp)
# target_link_libraries(${PROJECT_NAME} proxsuite::proxsuite)
# target_link_libraries(${PROJECT_NAME} qpOASES)

# # add_custom_command(
# #         TARGET ${PROJECT_NAME} POST_BUILD
# #     COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/result_visual.py
# # )


cmake_minimum_required(VERSION 3.10.0)
project(QUADRUPED_MPC VERSION 0.1.0 LANGUAGES C CXX)

find_package(pinocchio REQUIRED)
find_package(aligator REQUIRED)
find_package(example-robot-data REQUIRED)
find_package(proxsuite REQUIRED)
find_package(qpOASES REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(include)


# webots
set(WEBOTS_HOME "/usr/local/webots")
link_directories(${WEBOTS_HOME}/lib/controller)
set(WEBOTSLIBRARIES m ${CMAKE_SHARED_LIBRARY_PREFIX}Controller${CMAKE_SHARED_LIBRARY_SUFFIX} ${CMAKE_SHARED_LIBRARY_PREFIX}CppController${CMAKE_SHARED_LIBRARY_SUFFIX})
include_directories(${WEBOTS_HOME}/include/controller/c ${WEBOTS_HOME}/include/controller/cpp)

file(GLOB_RECURSE SRC_LIST
  src/**.cpp
  src/**/**.cpp
)
add_executable(QUADRUPED_MPC ${SRC_LIST} src/main.cpp)
target_link_libraries(QUADRUPED_MPC pinocchio::pinocchio)
target_link_libraries(QUADRUPED_MPC aligator::aligator)
target_link_libraries(QUADRUPED_MPC example-robot-data::example-robot-data)
target_link_libraries(QUADRUPED_MPC ${WEBOTSLIBRARIES})
target_link_libraries(QUADRUPED_MPC proxsuite::proxsuite)
target_link_libraries(QUADRUPED_MPC qpOASES)
target_link_libraries(QUADRUPED_MPC yaml-cpp::yaml-cpp)

