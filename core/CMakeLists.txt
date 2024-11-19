message("-- [CORE]")

set(CDIR ${CMAKE_CURRENT_LIST_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${DIR}/bin/core)

include_directories(
  ${CDIR}/src
)

add_library(demos SHARED)
target_sources(demos
  PRIVATE
    ${CDIR}/src/demos/1/demo_1.cpp
    ${CDIR}/src/demos/2/demo_2.cpp
)

add_executable(kalman-filter_demos
    ${CDIR}/executables/kf/main.cpp
)
target_link_libraries(kalman-filter_demos demos imgui implot)

