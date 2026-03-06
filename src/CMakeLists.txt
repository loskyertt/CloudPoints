# for all "src/x.cpp", generate target "x"
message("CMakeLists.txt for src directory")

set(ALL_SRCS
  main.cpp
)

add_executable(my_program ${ALL_SRCS})

target_link_libraries(my_program
  PRIVATE
    ${PCL_LIBRARIES}
    Boost::thread
    Boost::system
)

if(MSVC)
  target_compile_options(my_program PRIVATE /utf-8)  # UTF-8 支持
endif()

message("\n--------------------------------------------")
