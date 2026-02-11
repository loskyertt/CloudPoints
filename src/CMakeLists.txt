# for all "src/x.cpp", generate target "x"
message("CMakeLists.txt for src directory")

set(ALL_SRCS
  main.cpp
)


# 1. 查找 Boost 和 PCL
find_package(PCL REQUIRED COMPONENTS common io kdtree visualization)
find_package(Boost REQUIRED COMPONENTS thread system)

# 2. 打印查找结果
message(STATUS "PCL library status:")
message(STATUS "version: ${PCL_VERSION}")
message(STATUS "libraries: ${PCL_LIBRARIES}")
message(STATUS "include path: ${PCL_INCLUDE_DIRS}")


add_executable(my_program ${ALL_SRCS})

target_include_directories(my_program PUBLIC ${PROJECT_SOURCE_DIR}/include)

# 链接编译好的动态库
# target_link_libraries(my_program PUBLIC math_utils)

# 3. 链接库文件
target_include_directories(my_program
  PRIVATE
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

target_link_libraries(my_program
  PRIVATE
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
)

message("\n--------------------------------------------")
