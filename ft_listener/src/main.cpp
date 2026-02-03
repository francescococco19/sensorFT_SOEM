#include <iostream>
#include <thread>
#include <chrono>
#include <string>
#include <atomic>
#include <csignal>
#include <iomanip>

#include "SoemMaster.hpp"
#include "FtSensor.hpp"

// Global run flag used to stop the acquisition loop gracefully on SIGINT (Ctrl+C).
static std::atomic<bool> g_run(true);

static void on_sigint(int) {
  g_run.store(false);
}

// Compute average (tare) over N samples in raw counts.
static FtRaw compute_tare_pdo(FtSensor& ft,
                              int expectedWKC,
                              int samples,
                              std::chrono::milliseconds samplePeriod)
{
  int64_t sfx = 0, sfy = 0, sfz = 0, stx = 0, sty = 0, stz = 0;
  int valid = 0;

  for (int i = 0; i < samples && g_run.load(); ++i) {
    // Mantieni vivo il ciclo PDO
    ec_send_processdata();
    const int wkc = ec_receive_processdata(EC_TIMEOUTRET);
    if (wkc < expectedWKC) {
      // durante tare puoi anche solo loggare raramente
      std::cout << "Low WKC (tare): " << wkc << "\n";
    }

    FtRaw r;
    if (ft.read_ft_counts_pdo(r)) {
      sfx += r.fx; sfy += r.fy; sfz += r.fz;
      stx += r.tx; sty += r.ty; stz += r.tz;
      ++valid;
    }
    if (!ft.read_ft_counts_pdo(r)) {
      std::cout << "PDO read failed (Ibytes=" << ec_slave[1].Ibytes << ")\n";
    }

    std::this_thread::sleep_for(samplePeriod);
  }

  FtRaw off;
  if (valid > 0) {
    off.fx = static_cast<int32_t>(sfx / valid);
    off.fy = static_cast<int32_t>(sfy / valid);
    off.fz = static_cast<int32_t>(sfz / valid);
    off.tx = static_cast<int32_t>(stx / valid);
    off.ty = static_cast<int32_t>(sty / valid);
    off.tz = static_cast<int32_t>(stz / valid);
  }
  return off;
}

static FtRaw subtract_offset(const FtRaw& r, const FtRaw& off) {
  FtRaw z;
  z.fx = r.fx - off.fx;
  z.fy = r.fy - off.fy;
  z.fz = r.fz - off.fz;
  z.tx = r.tx - off.tx;
  z.ty = r.ty - off.ty;
  z.tz = r.tz - off.tz;
  return z;
}

int main(int argc, char** argv) {
  // --- Command line parsing ---------------------------------------------------
  if (argc < 2) {
    std::cerr << "Usage: sudo " << argv[0] << " <iface>\n";
    std::cerr << "Example: sudo " << argv[0] << " enx207bd51ab7ad\n";
    return 1;
  }
  const std::string ifname = argv[1];

  // Install SIGINT handler to allow a graceful exit with Ctrl+C.
  std::signal(SIGINT, on_sigint);

  // --- SOEM master initialization --------------------------------------------
  SoemMaster master;
  if (!master.init(ifname)) {
    std::cerr << "Error: ec_init failed on interface '" << ifname << "'. Run as root.\n";
    return 1;
  }

  if (!master.config_and_map()) {
    std::cerr << "Error: slave discovery/configuration failed (no slaves found?).\n";
    master.close();
    return 1;
  }

  const int expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
  std::cout << "Expected WKC = " << expectedWKC << "\n";
  std::cout << "Slave[1] Ibytes=" << ec_slave[1].Ibytes
          << " Obytes=" << ec_slave[1].Obytes << "\n";


  master.print_slaves();

  // For this test setup we expect ONLY the F/T sensor to be connected.
  if (ec_slavecount != 1) {
    std::cerr << "Error: expected exactly 1 slave (the F/T sensor), but found "
              << ec_slavecount << ".\n";
    master.close();
    return 1;
  }

  // Bring the network to SAFE_OP and then OP.
  master.to_safeop();

  // Request OP. Some slaves require processdata exchange to complete SAFE_OP -> OP.
  if (!master.to_op_all()) {
    for (int i = 0; i < 50; ++i) {
      ec_send_processdata();
      ec_receive_processdata(EC_TIMEOUTRET);
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    ec_readstate();
    if (ec_slave[1].state == EC_STATE_OPERATIONAL) {
      std::cout << "Network reached OP state after processdata pumping.\n";
    } else {
      std::cerr << "Warning: slave did not reach OP state (state="
                << ec_slave[1].state << "). Continuing anyway.\n";
    }
  } else {
    std::cout << "Network is in OP state.\n";
  }

  // With a single connected slave, the sensor is always slave index 1.
  constexpr uint16_t kFtSlaveIndex = 1;
  FtSensor ft(kFtSlaveIndex);

  // --- One-time reads: calibration and scaling --------------------------------
  std::string calib;
  if (ft.read_calibration(calib)) {
    std::cout << "Calibration: " << calib << "\n";
  } else {
    std::cout << "Calibration: (read failed)\n";
  }

  double countsPerForce = 0.0;
  double countsPerTorque = 0.0;
  if (!ft.read_scale_factors(countsPerForce, countsPerTorque)) {
    std::cerr << "Error: failed to read scale factors (counts-per-force/torque).\n";
    master.close();
    return 1;
  }

  std::cout << "Scale factors:\n";
  std::cout << "  CountsPerForce  = " << countsPerForce  << " [counts/N]\n";
  std::cout << "  CountsPerTorque = " << countsPerTorque << " [counts/Nm]\n";

  // --- Tare / zeroing ---------------------------------------------------------
  // Average a short window at startup and subtract that offset from subsequent readings.
  constexpr int kTareSamples = 100;                         // ~2 seconds at 20 ms
  const auto kTarePeriod = std::chrono::milliseconds(20);

  std::cout << "Tare: acquiring " << kTareSamples << " samples...\n";
  FtRaw tareOffset = compute_tare_pdo(ft, expectedWKC, kTareSamples, kTarePeriod);
  std::cout << "Tare offset (counts): "
            << "Fx=" << tareOffset.fx << " Fy=" << tareOffset.fy << " Fz=" << tareOffset.fz
            << " Tx=" << tareOffset.tx << " Ty=" << tareOffset.ty << " Tz=" << tareOffset.tz
            << "\n";

  // --- Acquisition loop -------------------------------------------------------
  // Read raw counts via SDO, apply tare, then convert to physical units.
  constexpr auto kLoopPeriod = std::chrono::milliseconds(20);

  std::cout << std::fixed << std::setprecision(4);
  while (g_run.load()) {
    // 1) Exchange process data (PDO)
    ec_send_processdata();
    const int wkc = ec_receive_processdata(EC_TIMEOUTRET);

    if (wkc < expectedWKC) {
      std::cout << "Low WKC: " << wkc << " (expected " << expectedWKC << ")\n";
      // opzionale: ec_readstate(); log state; recovery...
    }

    // 2) Read PDO-mapped counts
    FtRaw raw;
    if (!ft.read_ft_counts_pdo(raw)) {
      std::cout << "PDO read failed (Ibytes=" << ec_slave[1].Ibytes << ")\n";
      std::this_thread::sleep_for(kLoopPeriod);
      continue;
    }

    const FtRaw zeroed = subtract_offset(raw, tareOffset);
    const FtScaled phys = FtSensor::to_physical(zeroed, countsPerForce, countsPerTorque);

    std::cout
      << "F[N]  = (" << phys.fx_N  << ", " << phys.fy_N  << ", " << phys.fz_N  << ")  "
      << "T[Nm] = (" << phys.tx_Nm << ", " << phys.ty_Nm << ", " << phys.tz_Nm << ")\n";

    std::this_thread::sleep_for(kLoopPeriod);
  }

  // --- Shutdown ---------------------------------------------------------------
  std::cout << "Shutdown requested (Ctrl+C). Closing EtherCAT master...\n";
  master.close();
  return 0;
}
