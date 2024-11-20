#pragma once

#include <sys/time.h>

#include <cstdint>

class TimeBase {
 private:
  bool _amITesting;
  uint64_t time;

  constexpr TimeBase() {
    _amITesting = false;
    time = 0;
  };
  TimeBase(const TimeBase &other) = delete;
  TimeBase &operator=(const TimeBase &other) = delete;
  TimeBase(TimeBase &&other) = delete;
  TimeBase &operator=(TimeBase &&other) = delete;
  static TimeBase &get() {
    static TimeBase inst;
    return inst;
  }

 public:
  static void enableTesting() { get()._amITesting = true; }
  static void disableTesting() { get()._amITesting = false; }
  static void setTimestamp(uint64_t _time) { get().time = _time; }
  static bool amITesting() { return get()._amITesting; }

  static uint64_t getTimestampMicroseconds() {
    if (amITesting()) {
      return get().time;
    } else {
      struct timeval tv;
      gettimeofday(&tv, nullptr);
      return tv.tv_sec * 1000000 + tv.tv_usec;
    }
  }
  static double getTimestampSeconds() {
    return getTimestampMicroseconds() / 1e6;
  }
};
