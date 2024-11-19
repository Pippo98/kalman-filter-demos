message("-- [EXTERNAL]")

set(CDIR ${CMAKE_CURRENT_LIST_DIR})

add_subdirectory(${CDIR}/kflib)

set(EXTERNAL_DIRS
  ${CDIR}

  ${CDIR}/imgui
  ${CDIR}/imgui/misc/cpp
  ${CDIR}/imgui/backends
  ${CDIR}/implot
)
include_directories(PUBLIC ${EXTERNAL_DIRS})

FILE(GLOB imgui_srcs ${CDIR}/imgui/*)
add_library(
    imgui
    SHARED 
    ${imgui_srcs}
    ${CDIR}/imgui/backends/imgui_impl_glfw.cpp
    ${CDIR}/imgui/backends/imgui_impl_opengl2.cpp
    ${CDIR}/imgui/misc/cpp/imgui_stdlib.cpp
)
target_link_libraries(imgui
    PUBLIC
    glfw
    OpenGL::GL
    ${GLEW_LIBRARIES}
    GLEW::GLEW
)

FILE(GLOB implot_srcs ${CDIR}/implot/*)
add_library(
    implot
    SHARED 
    ${implot_srcs}
    ${CDIR}/implot/implot.cpp
    ${CDIR}/implot/implot_items.cpp
)
target_link_libraries(implot PUBLIC imgui)





set(EXTERNAL_DIRS ${EXTERNAL_DIRS} PARENT_SCOPE)
