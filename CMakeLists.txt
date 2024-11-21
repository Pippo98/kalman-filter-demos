cmake_minimum_required(VERSION 3.16)
project(TelemetryApp)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(DIR ${CMAKE_CURRENT_LIST_DIR})
set(USER_HOME $ENV{HOME})
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -g -Wall -Wextra -Werror -fPIC")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -DNDEBUG -w -fPIC")


find_package(GLEW REQUIRED)
find_package(OpenGL REQUIRED)
find_package(glfw3 3.3 REQUIRED)

add_compile_definitions(IMGUI_USE_WCHAR32)
add_compile_definitions("ImDrawIdx=unsigned int")
add_compile_definitions(IMGUI_DEFINE_MATH_OPERATORS)

include_directories(
  ${DIR}
)

add_subdirectory(${DIR}/external)
include_directories(PUBLIC ${EXTERNAL_DIRS})
add_subdirectory(${DIR}/core)
