cmake_minimum_required(VERSION 3.20.0)

# 导出编译命令（用于 clangd）
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

project(CloudPoints VERSION 0.0.1 LANGUAGES C CXX)

# 设置标准
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)

if(MSVC)
  add_compile_options(/utf-8)
endif()

# 开启 gtest_discover_tests 注册测试用例依赖
# enable_testing()

# 脚本
# include(cmake/check_arch.cmake)

if(WIN32)
  list(APPEND
    CMAKE_PREFIX_PATH
      "C:\\Packages\\PCL_1.15.1"
  )
endif()

find_package(PCL REQUIRED COMPONENTS common io kdtree visualization)
find_package(Boost REQUIRED CONFIG COMPONENTS thread system)

# 子模块
add_subdirectory(src)
add_subdirectory(tests)
