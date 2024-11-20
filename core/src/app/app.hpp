#pragma once

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <string>
#include "external/imgui/imgui.h"

class App {
 public:
  App(const std::string &_name) : name(_name) {}

  bool Open();
  bool IsOpen() const;

  void BeginFrame();
  void EndFrame();

 private:
  std::string name;
  GLFWwindow *window = nullptr;
};
