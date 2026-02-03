#include "FtSensor.hpp"
#include <cstring>
#include <type_traits>

static constexpr int kSzI32 = 4;
static constexpr int kSzU32 = 4;

#pragma pack(push, 1)
// Assumes PDO input starts with 6 x int32: Fx, Fy, Fz, Tx, Ty, Tz.
struct FtPdoIn6I32 {
  int32_t fx;
  int32_t fy;
  int32_t fz;
  int32_t tx;
  int32_t ty;
  int32_t tz;
};
#pragma pack(pop)

static_assert(sizeof(FtPdoIn6I32) == 24, "FtPdoIn6I32 must be exactly 24 bytes");

// Helper to read a uint32 via SDO (little-endian handled by SOEM).
static bool sdo_read_u32(uint16_t slave, uint16_t index, uint8_t sub, uint32_t& out) {
  int size = kSzU32;
  out = 0;
  const int wkc = ec_SDOread(slave, index, sub, FALSE, &size, &out, EC_TIMEOUTSAFE);
  return (wkc > 0 && size == kSzU32);
}

// Helper to read an int32 via SDO.
static bool sdo_read_i32(uint16_t slave, uint16_t index, uint8_t sub, int32_t& out) {
  int size = kSzI32;
  out = 0;
  const int wkc = ec_SDOread(slave, index, sub, FALSE, &size, &out, EC_TIMEOUTSAFE);
  return (wkc > 0 && size == kSzI32);
}

FtSensor::FtSensor(uint16_t slave_index) : slave_(slave_index) {}

bool FtSensor::read_calibration(std::string& out) {
  // ATI Axia: 0x2021:0x02 is typically a 30-byte ASCII string.
  char calib[30];
  std::memset(calib, 0, sizeof(calib));
  int size = static_cast<int>(sizeof(calib));

  const int wkc = ec_SDOread(slave_, 0x2021, 0x02, FALSE, &size, calib, EC_TIMEOUTSAFE);
  if (wkc <= 0) return false;

  // If the device doesn't null-terminate, size helps; but if it does, std::string(calib) is fine.
  // We'll use size defensively and trim trailing zeros.
  int n = size;
  if (n < 0) n = 0;
  if (n > static_cast<int>(sizeof(calib))) n = static_cast<int>(sizeof(calib));
  while (n > 0 && calib[n - 1] == '\0') --n;
  out.assign(calib, calib + n);

  return true;
}

bool FtSensor::read_scale_factors(double& countsPerForce, double& countsPerTorque) {
  // ATI Axia scale factors:
  // 0x2021:0x37 -> Counts Per Force
  // 0x2021:0x38 -> Counts Per Torque
  uint32_t cpf = 0;
  uint32_t cpt = 0;

  if (!sdo_read_u32(slave_, 0x2021, 0x37, cpf)) return false;
  if (!sdo_read_u32(slave_, 0x2021, 0x38, cpt)) return false;

  if (cpf == 0 || cpt == 0) return false;

  countsPerForce  = static_cast<double>(cpf);
  countsPerTorque = static_cast<double>(cpt);
  return true;
}

bool FtSensor::read_ft_counts(FtRaw& out) {
  // ATI Axia F/T sample:
  // 0x6000:01..06 -> Fx, Fy, Fz, Tx, Ty, Tz (int32 counts)
  if (!sdo_read_i32(slave_, 0x6000, 0x01, out.fx)) return false;
  if (!sdo_read_i32(slave_, 0x6000, 0x02, out.fy)) return false;
  if (!sdo_read_i32(slave_, 0x6000, 0x03, out.fz)) return false;
  if (!sdo_read_i32(slave_, 0x6000, 0x04, out.tx)) return false;
  if (!sdo_read_i32(slave_, 0x6000, 0x05, out.ty)) return false;
  if (!sdo_read_i32(slave_, 0x6000, 0x06, out.tz)) return false;
  return true;
}

bool FtSensor::read_ft_counts_pdo(FtRaw& out) const {
  // PDO inputs are available after ec_config_map() and when exchanging processdata.
  if (slave_ == 0 || slave_ > ec_slavecount) return false;

  // Need at least 24 bytes for 6x int32.
  if (ec_slave[slave_].Ibytes < static_cast<int>(sizeof(FtPdoIn6I32))) return false;

  const void* inputs = ec_slave[slave_].inputs;
  if (!inputs) return false;

  const auto* in = reinterpret_cast<const FtPdoIn6I32*>(inputs);

  out.fx = in->fx; out.fy = in->fy; out.fz = in->fz;
  out.tx = in->tx; out.ty = in->ty; out.tz = in->tz;

  return true;
}

FtScaled FtSensor::to_physical(const FtRaw& c, double countsPerForce, double countsPerTorque) {
  FtScaled s;
  s.fx_N  = static_cast<double>(c.fx) / countsPerForce;
  s.fy_N  = static_cast<double>(c.fy) / countsPerForce;
  s.fz_N  = static_cast<double>(c.fz) / countsPerForce;

  s.tx_Nm = static_cast<double>(c.tx) / countsPerTorque;
  s.ty_Nm = static_cast<double>(c.ty) / countsPerTorque;
  s.tz_Nm = static_cast<double>(c.tz) / countsPerTorque;
  return s;
}
