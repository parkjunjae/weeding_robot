cmake_minimum_required(VERSION 3.16)
project(weedbot_lidar_odom_ndt LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(PCL REQUIRED COMPONENTS common io filters registration)
find_package(pybind11 REQUIRED)                    # apt(pybind11-dev) or pip
# Python은 굳이 찾지 않아도 됨(Interpreter Development 없어도 OK)

# 파이썬 확장 모듈 생성 (pybind11이 알아서 모듈용 플래그를 세팅)
pybind11_add_module(lidar_odom_ndt
    localization/lidar_odom_ndt.cpp
)

target_include_directories(lidar_odom_ndt PRIVATE
    ${PCL_INCLUDE_DIRS}
)
target_link_libraries(lidar_odom_ndt PRIVATE
    ${PCL_LIBRARIES}
)

# 모듈 이름의 lib 접두사 제거
set_target_properties(lidar_odom_ndt PROPERTIES
    PREFIX ""
    OUTPUT_NAME "lidar_odom_ndt"
)

# ⚠️ 넣지 마세요:
# target_link_options(lidar_odom_ndt PRIVATE -Wl,--no-undefined)
# target_link_libraries(lidar_odom_ndt PRIVATE Python3::Module)
