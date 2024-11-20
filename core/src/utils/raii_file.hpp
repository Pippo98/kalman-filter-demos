#pragma once

#include <cstdio>
#include <memory>

struct FileCloser {
  void operator()(FILE *file) const {
    if (file) {
      fclose(file);
    }
  }
};

typedef std::unique_ptr<FILE, FileCloser> RAIIFile;
