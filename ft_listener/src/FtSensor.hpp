#pragma once
#include <cstdint>
#include <string>

extern "C" {
#include "ethercat.h"
}

// Raw F/T sample in device counts.
struct FtRaw {
  int32_t fx{0}, fy{0}, fz{0}, tx{0}, ty{0}, tz{0};
};

// F/T sample in physical units (N and Nm).
struct FtScaled {
  double fx_N{0.0}, fy_N{0.0}, fz_N{0.0};
  double tx_Nm{0.0}, ty_Nm{0.0}, tz_Nm{0.0};
};

class FtSensor {
public:
  explicit FtSensor(uint16_t slave_index);

  // Reads the calibration string (e.g., "SI-150-8") used to verify the loaded calibration.
  bool read_calibration(std::string& out);

  // Reads the device-specific scale factors:
  // - countsPerForce: counts per Newton
  // - countsPerTorque: counts per Newton-meter
  bool read_scale_factors(double& countsPerForce, double& countsPerTorque);

  // Reads the 6-axis F/T sample in raw counts via SDO (mailbox/CoE).
  bool read_ft_counts(FtRaw& out);

  // Reads the 6-axis F/T sample in raw counts via PDO (process data).
  // This is the preferred method for real-time loops.
  bool read_ft_counts_pdo(FtRaw& out) const;

  // Converts a raw sample (counts) into physical units using provided scale factors.
  static FtScaled to_physical(const FtRaw& counts,
                              double countsPerForce,
                              double countsPerTorque);

  uint16_t slave() const { return slave_; }

private:
  uint16_t slave_;
};
