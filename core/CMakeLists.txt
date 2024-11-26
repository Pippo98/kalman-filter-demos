message("-- [CORE]")

set(CDIR ${CMAKE_CURRENT_LIST_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${DIR}/bin/core)

configure_file(${CDIR}/src/defines.hpp.in ${CDIR}/src/defines.hpp)

include_directories(
  ${CDIR}/src
)

add_library(app SHARED)
target_sources(app
  PRIVATE
    ${CDIR}/src/app/app.cpp
)
target_link_libraries(app PUBLIC imgui implot)

add_library(simulator SHARED)
target_sources(simulator
  PRIVATE
    ${CDIR}/src/simulator/simulator.cpp
    ${CDIR}/src/simulator/simulation_manager.cpp
)

add_library(demos SHARED)
target_sources(demos
  PRIVATE
    ${CDIR}/src/demos/1/demo_1.cpp
    ${CDIR}/src/demos/2/demo_2.cpp
    ${CDIR}/src/demos/3/demo_3.cpp
    ${CDIR}/src/demos/4/demo_4.cpp

    ${CDIR}/src/demos/s1/demo_sigma_points.cpp
)
target_link_libraries(demos PUBLIC kflib)

add_executable(kalman-filter_demos
    ${CDIR}/executables/kf/main.cpp
    ${CDIR}/executables/kf/main_menu.cpp
)
target_link_libraries(kalman-filter_demos demos app simulator)

