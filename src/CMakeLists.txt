# for all "src/x.cpp", generate target "x"
message("CMakeLists.txt for src directory")

set(ALL_SRCS
  main.cpp
)

if(WIN32)
  list(APPEND
    CMAKE_PREFIX_PATH
      "C:\\Packages\\PCL 1.15.1\\cmake"
      "C:\\Packages\\PCL 1.15.1\\3rdParty\\Boost"
  )
endif()

# 1. 查找 Boost 和 PCL
find_package(PCL REQUIRED COMPONENTS common io kdtree visualization)
find_package(Boost REQUIRED CONFIG COMPONENTS thread system)

# 2. 打印查找结果
message(STATUS "Boost library status:")
message(STATUS "version: ${Boost_VERSION}")
message(STATUS "libraries: ${Boost_LIBRARIES}")
message(STATUS "include path: ${Boost_INCLUDE_DIRS}\n")

message(STATUS "PCL library status:")
message(STATUS "version: ${PCL_VERSION}")
message(STATUS "libraries: ${PCL_LIBRARIES}")
message(STATUS "include path: ${PCL_INCLUDE_DIRS}\n")

add_executable(my_program ${ALL_SRCS})

target_include_directories(my_program
  PUBLIC
    ${PROJECT_SOURCE_DIR}/include
)

# 链接编译好的动态库
# target_link_libraries(my_program PUBLIC math_utils)

target_link_libraries(my_program
  PRIVATE
    ${PCL_LIBRARIES}
    Boost::thread
    Boost::system
)

# MSVC UTF-8 支持
if(MSVC)
  target_compile_options(my_program PRIVATE /utf-8)
endif()

message("\n--------------------------------------------")
