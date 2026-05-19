set(TARGET ${PROJECT_NAME}-${CMAKE_SYSTEM_NAME})

add_executable(${TARGET} main.cpp)

target_link_libraries(${TARGET}
  PRIVATE
    ${PCL_LIBRARIES}
    Boost::thread
    Boost::system
)

message("\n--------------------------------------------")
