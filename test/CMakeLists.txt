# for each "test/x.cpp", generate target "x"
message("CMakeLists.txt for test directory")

file(GLOB_RECURSE ALL_TESTS *.cpp)
foreach(v ${ALL_TESTS})
  message("Processing: ${v}")
  get_filename_component(target_name ${v} NAME_WE) # 生成目标名（去掉路径和扩展名）

  add_executable(${target_name} ${v})

  # 为当前目标添加头文件路径
  target_include_directories(${target_name} PUBLIC ${PROJECT_SOURCE_DIR}/include)

  # target_compile_options(${target_name} PRIVATE -Wall -Wextra -Werror)
endforeach()

message("\n--------------------------------------------")
