#pragma once
#include <string>

extern "C" {
#include "ethercat.h"
}

class SoemMaster {
public:
  bool init(const std::string& ifname);
  bool config_and_map();
  bool to_safeop(int timeout_mult = 4);
  bool to_op_all(int timeout_mult = 4);
  void close();

  void print_slaves() const;
};
