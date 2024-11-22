#pragma once

class MainMenu {
 public:
  void draw();

  bool isPaused() const { return paused; }

 private:
  bool paused = false;

  void onPausedChanged();
};
