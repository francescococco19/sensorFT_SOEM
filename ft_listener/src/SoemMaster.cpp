#include "SoemMaster.hpp"
#include <iostream>
#include <cstring>

static char IOmap[4096];

bool SoemMaster::init(const std::string& ifname) {
  std::memset(IOmap, 0, sizeof(IOmap));
  if (!ec_init(ifname.c_str())) return false;
  return true;
}

bool SoemMaster::config_and_map() {
  if (!(ec_config_init(FALSE) > 0)) return false;
  ec_config_map(&IOmap);
  ec_configdc();
  return true;
}

bool SoemMaster::to_safeop(int timeout_mult) {
  ec_readstate();
  ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * timeout_mult);
  return true;
}

bool SoemMaster::to_op_all(int timeout_mult) {
  ec_slave[0].state = EC_STATE_OPERATIONAL;
  ec_writestate(0);
  ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE * timeout_mult);
  ec_readstate();
  return (ec_slave[0].state == EC_STATE_OPERATIONAL);
}

void SoemMaster::print_slaves() const {
  std::cout << ec_slavecount << " slaves found and configured.\n";
  for (int i = 1; i <= ec_slavecount; i++) {
    std::cout << "Slave " << i
              << " name='" << ec_slave[i].name << "'"
              << " man=" << std::hex << ec_slave[i].eep_man
              << " id="  << std::hex << ec_slave[i].eep_id
              << " rev=" << std::hex << ec_slave[i].eep_rev
              << std::dec << "\n";
  }
}

void SoemMaster::close() {
  ec_close();
}
