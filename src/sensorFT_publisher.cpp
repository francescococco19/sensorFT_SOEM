#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>

#include <deque>
#include <vector>
#include <cmath>
#include <algorithm>
#include <numeric>

#include <string>
#include <cstdint>
#include <chrono>
#include <thread>

#include "SoemMaster.hpp"
#include "FtSensor.hpp"

// Compute a tare offset (average) over N samples in raw counts.
// This is used to zero the sensor output in software.

static FtRaw subtract_offset(const FtRaw& r, const FtRaw& off)
{
  FtRaw z;
  z.fx = r.fx - off.fx;
  z.fy = r.fy - off.fy;
  z.fz = r.fz - off.fz;
  z.tx = r.tx - off.tx;
  z.ty = r.ty - off.ty;
  z.tz = r.tz - off.tz;
  return z;
}

// Compute mean and standard deviation (population) for a vector of doubles.
// Returns {mean, stddev}. If input is empty, returns {0, 0}.
static std::pair<double, double> mean_stddev(const std::vector<double>& v)
{
  if (v.empty()) return {0.0, 0.0};
  const double mean = std::accumulate(v.begin(), v.end(), 0.0) / static_cast<double>(v.size());
  double acc = 0.0;
  for (double x : v) {
    const double d = x - mean;
    acc += d * d;
  }
  const double var = acc / static_cast<double>(v.size());
  return {mean, std::sqrt(std::max(0.0, var))};
}

struct FtStats {
  // Standard deviation in physical units
  double std_fx = 0, std_fy = 0, std_fz = 0;
  double std_tx = 0, std_ty = 0, std_tz = 0;

  // Means in physical units (sometimes useful for debugging)
  double mean_fx = 0, mean_fy = 0, mean_fz = 0;
  double mean_tx = 0, mean_ty = 0, mean_tz = 0;
};

// Compute per-axis stats over a window of samples (already converted to physical units).
static FtStats compute_window_stats(const std::deque<FtScaled>& w)
{
  FtStats s;
  if (w.empty()) return s;

  std::vector<double> fx, fy, fz, tx, ty, tz;
  fx.reserve(w.size()); fy.reserve(w.size()); fz.reserve(w.size());
  tx.reserve(w.size()); ty.reserve(w.size()); tz.reserve(w.size());

  for (const auto& a : w) {
    fx.push_back(a.fx_N);  fy.push_back(a.fy_N);  fz.push_back(a.fz_N);
    tx.push_back(a.tx_Nm); ty.push_back(a.ty_Nm); tz.push_back(a.tz_Nm);
  }

  auto [mfx, sfx] = mean_stddev(fx); auto [mfy, sfy] = mean_stddev(fy); auto [mfz, sfz] = mean_stddev(fz);
  auto [mtx, stx] = mean_stddev(tx); auto [mty, sty] = mean_stddev(ty); auto [mtz, stz] = mean_stddev(tz);

  s.mean_fx = mfx; s.std_fx = sfx;
  s.mean_fy = mfy; s.std_fy = sfy;
  s.mean_fz = mfz; s.std_fz = sfz;

  s.mean_tx = mtx; s.std_tx = stx;
  s.mean_ty = mty; s.std_ty = sty;
  s.mean_tz = mtz; s.std_tz = stz;

  return s;
}

// Wait until the signal is stable enough (stability gate).
// - Reads samples periodically.
// - Maintains a sliding window of size stability_samples.
// - Declares "stable" when stddev for all axes is below thresholds.
// Returns true if stable is reached before max_wait_seconds; false otherwise.
static bool wait_for_stability(
    FtSensor& ft,
    double countsPerForce,
    double countsPerTorque,
    int stability_samples,
    int period_ms,
    double force_std_thresh_N,
    double torque_std_thresh_Nm,
    double max_wait_seconds,
    FtStats* last_stats_out = nullptr)
{
  std::deque<FtScaled> window;
  window.clear();

  const auto t_start = ros::Time::now();
  const ros::Duration max_wait(max_wait_seconds);

  while (ros::ok()) {
    FtRaw raw;
    if (ft.read_ft_counts(raw)) {
      FtScaled phys = FtSensor::to_physical(raw, countsPerForce, countsPerTorque);

      window.push_back(phys);
      if (static_cast<int>(window.size()) > stability_samples) {
        window.pop_front();
      }

      if (static_cast<int>(window.size()) == stability_samples) {
        FtStats stats = compute_window_stats(window);

        if (last_stats_out) *last_stats_out = stats;

        const bool stable_force =
            (stats.std_fx <= force_std_thresh_N) &&
            (stats.std_fy <= force_std_thresh_N) &&
            (stats.std_fz <= force_std_thresh_N);

        const bool stable_torque =
            (stats.std_tx <= torque_std_thresh_Nm) &&
            (stats.std_ty <= torque_std_thresh_Nm) &&
            (stats.std_tz <= torque_std_thresh_Nm);

        if (stable_force && stable_torque) {
          return true;
        }
      }
    }

    if ((ros::Time::now() - t_start) > max_wait) {
      return false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(period_ms));
  }

  return false;
}

// Compute tare offset with outlier rejection.
// 1) Collect tare_samples raw-count samples.
// 2) Convert to physical units, compute per-axis mean/std.
// 3) Reject samples beyond k*sigma on ANY axis (force or torque).
// 4) Recompute mean in raw counts of the retained samples -> tare offset (counts).
static FtRaw compute_tare_outlier_rejection(
    FtSensor& ft,
    double countsPerForce,
    double countsPerTorque,
    int tare_samples,
    int period_ms,
    double k_sigma,
    int* kept_out = nullptr)
{
  std::vector<FtRaw> raw_samples;
  raw_samples.reserve(tare_samples);

  // Collect samples
  for (int i = 0; i < tare_samples && ros::ok(); ++i) {
    FtRaw r;
    if (ft.read_ft_counts(r)) {
      raw_samples.push_back(r);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(period_ms));
  }

  // If nothing collected, return zero offset.
  if (raw_samples.empty()) {
    if (kept_out) *kept_out = 0;
    return FtRaw{};
  }

  // Compute mean/std in physical units for gating outliers robustly.
  std::vector<double> fx, fy, fz, tx, ty, tz;
  fx.reserve(raw_samples.size()); fy.reserve(raw_samples.size()); fz.reserve(raw_samples.size());
  tx.reserve(raw_samples.size()); ty.reserve(raw_samples.size()); tz.reserve(raw_samples.size());

  for (const auto& r : raw_samples) {
    FtScaled p = FtSensor::to_physical(r, countsPerForce, countsPerTorque);
    fx.push_back(p.fx_N);  fy.push_back(p.fy_N);  fz.push_back(p.fz_N);
    tx.push_back(p.tx_Nm); ty.push_back(p.ty_Nm); tz.push_back(p.tz_Nm);
  }

  auto [mfx, sfx] = mean_stddev(fx); auto [mfy, sfy] = mean_stddev(fy); auto [mfz, sfz] = mean_stddev(fz);
  auto [mtx, stx] = mean_stddev(tx); auto [mty, sty] = mean_stddev(ty); auto [mtz, stz] = mean_stddev(tz);

  // Avoid division by zero; if std is ~0, treat as "no outliers" on that axis.
  auto is_outlier = [&](double x, double mean, double sd) -> bool {
    if (sd < 1e-12) return false;
    return std::fabs(x - mean) > (k_sigma * sd);
  };

  // Keep only inliers.
  std::vector<FtRaw> kept;
  kept.reserve(raw_samples.size());

  for (size_t i = 0; i < raw_samples.size(); ++i) {
    const bool out =
        is_outlier(fx[i], mfx, sfx) || is_outlier(fy[i], mfy, sfy) || is_outlier(fz[i], mfz, sfz) ||
        is_outlier(tx[i], mtx, stx) || is_outlier(ty[i], mty, sty) || is_outlier(tz[i], mtz, stz);

    if (!out) kept.push_back(raw_samples[i]);
  }

  if (kept_out) *kept_out = static_cast<int>(kept.size());

  // If rejection was too aggressive, fall back to all samples.
  const auto& final_set = (!kept.empty() ? kept : raw_samples);

  // Compute mean in raw counts for the offset.
  int64_t sfxc = 0, sfyc = 0, sfzc = 0, stxc = 0, styc = 0, stzc = 0;
  for (const auto& r : final_set) {
    sfxc += r.fx; sfyc += r.fy; sfzc += r.fz;
    stxc += r.tx; styc += r.ty; stzc += r.tz;
  }

  FtRaw off;
  off.fx = static_cast<int32_t>(sfxc / static_cast<int64_t>(final_set.size()));
  off.fy = static_cast<int32_t>(sfyc / static_cast<int64_t>(final_set.size()));
  off.fz = static_cast<int32_t>(sfzc / static_cast<int64_t>(final_set.size()));
  off.tx = static_cast<int32_t>(stxc / static_cast<int64_t>(final_set.size()));
  off.ty = static_cast<int32_t>(styc / static_cast<int64_t>(final_set.size()));
  off.tz = static_cast<int32_t>(stzc / static_cast<int64_t>(final_set.size()));

  return off;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ft_sensor_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // --- Parameters ------------------------------------------------------------
  std::string iface;
  pnh.param<std::string>("iface", iface, std::string("enx207bd51ab7ad"));

  std::string frame_id;
  pnh.param<std::string>("frame_id", frame_id, std::string("ft_sensor"));

  double rate_hz;
  pnh.param<double>("rate_hz", rate_hz, 50.0);

  int tare_samples;
  pnh.param<int>("tare_samples", tare_samples, 100);

  int tare_period_ms;
  pnh.param<int>("tare_period_ms", tare_period_ms, 20);

  bool require_single_slave;
  pnh.param<bool>("require_single_slave", require_single_slave, true);

  std::string topic;
  pnh.param<std::string>("topic", topic, std::string("/ft_sensor/wrench"));

  // Improved automatic tare parameters
  int warmup_seconds;
  pnh.param<int>("warmup_seconds", warmup_seconds, 15);

  int stability_samples;
  pnh.param<int>("stability_samples", stability_samples, 50);

  int stability_period_ms;
  pnh.param<int>("stability_period_ms", stability_period_ms, 20);

  double stability_force_std_thresh_N;
  pnh.param<double>("stability_force_std_thresh_N", stability_force_std_thresh_N, 0.05);

  double stability_torque_std_thresh_Nm;
  pnh.param<double>("stability_torque_std_thresh_Nm", stability_torque_std_thresh_Nm, 0.005);

  double stability_max_wait_s;
  pnh.param<double>("stability_max_wait_s", stability_max_wait_s, 60.0);

  double tare_outlier_k_sigma;
  pnh.param<double>("tare_outlier_k_sigma", tare_outlier_k_sigma, 3.0);

  ROS_INFO("ft_sensor_node starting...");
  ROS_INFO("Parameters: iface=%s frame_id=%s rate_hz=%.2f topic=%s",
           iface.c_str(), frame_id.c_str(), rate_hz, topic.c_str());
  ROS_INFO("Tare params: warmup_seconds=%d stability_samples=%d stability_period_ms=%d "
           "force_std_thresh_N=%.4f torque_std_thresh_Nm=%.5f stability_max_wait_s=%.1f "
           "tare_samples=%d tare_period_ms=%d outlier_k_sigma=%.2f",
           warmup_seconds, stability_samples, stability_period_ms,
           stability_force_std_thresh_N, stability_torque_std_thresh_Nm, stability_max_wait_s,
           tare_samples, tare_period_ms, tare_outlier_k_sigma);

  // Publisher
  ros::Publisher pub = nh.advertise<geometry_msgs::WrenchStamped>(topic, 10);

  // --- SOEM master bring-up ---------------------------------------------------
  SoemMaster master;

  if (!master.init(iface)) {
    ROS_ERROR("ec_init failed on interface '%s'. Run with sudo or set capabilities.", iface.c_str());
    return 1;
  }

  if (!master.config_and_map()) {
    ROS_ERROR("Slave discovery/configuration failed (no slaves found?).");
    master.close();
    return 1;
  }

  master.print_slaves();

  if (require_single_slave && ec_slavecount != 1) {
    ROS_ERROR("Expected exactly 1 slave (the F/T sensor), but found %d. Aborting.", ec_slavecount);
    master.close();
    return 1;
  }

  // Move to SAFE_OP then request OP.
  master.to_safeop();

  // Some slaves require processdata exchange to complete SAFE_OP -> OP.
  if (!master.to_op_all()) {
    for (int i = 0; i < 50 && ros::ok(); ++i) {
      ec_send_processdata();
      ec_receive_processdata(EC_TIMEOUTRET);
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    ec_readstate();

    if (ec_slave[1].state == EC_STATE_OPERATIONAL) {
      ROS_INFO("Network reached OP state after processdata pumping.");
    } else {
      ROS_WARN("Slave did not reach OP state (state=%d). Continuing anyway (SDO may still work).",
               ec_slave[1].state);
    }
  } else {
    ROS_INFO("Network is in OP state.");
  }

  // With a single connected slave, the sensor is always slave index 1.
  constexpr uint16_t kFtSlaveIndex = 1;
  FtSensor ft(kFtSlaveIndex);

  // --- One-time reads: calibration and scaling --------------------------------
  std::string calib;
  if (ft.read_calibration(calib)) {
    ROS_INFO("Calibration: %s", calib.c_str());
  } else {
    ROS_WARN("Calibration read failed.");
  }

  double countsPerForce = 0.0;
  double countsPerTorque = 0.0;
  if (!ft.read_scale_factors(countsPerForce, countsPerTorque)) {
    ROS_ERROR("Failed to read scale factors (counts-per-force/torque).");
    master.close();
    return 1;
  }

  ROS_INFO("Scale factors: CountsPerForce=%.0f [counts/N], CountsPerTorque=%.0f [counts/Nm]",
           countsPerForce, countsPerTorque);

  // --- Improved automatic tare: warm-up + stability gate + outlier rejection ---
  ROS_INFO("Warm-up: waiting %d seconds before stability check...", warmup_seconds);
  for (int i = 0; i < warmup_seconds && ros::ok(); ++i) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  ROS_INFO("Stability gate: waiting for stable signal (window=%d samples @ %d ms)...",
           stability_samples, stability_period_ms);

  FtStats last_stats;
  const bool stable = wait_for_stability(
      ft,
      countsPerForce,
      countsPerTorque,
      stability_samples,
      stability_period_ms,
      stability_force_std_thresh_N,
      stability_torque_std_thresh_Nm,
      stability_max_wait_s,
      &last_stats);

  if (!stable) {
    ROS_WARN("Stability gate timed out after %.1f s. Proceeding with tare anyway.", stability_max_wait_s);
    ROS_WARN("Last window stddev: F[N] std=(%.4f, %.4f, %.4f)  T[Nm] std=(%.5f, %.5f, %.5f)",
             last_stats.std_fx, last_stats.std_fy, last_stats.std_fz,
             last_stats.std_tx, last_stats.std_ty, last_stats.std_tz);
  } else {
    ROS_INFO("Signal is stable. Window stddev: F[N] std=(%.4f, %.4f, %.4f)  T[Nm] std=(%.5f, %.5f, %.5f)",
             last_stats.std_fx, last_stats.std_fy, last_stats.std_fz,
             last_stats.std_tx, last_stats.std_ty, last_stats.std_tz);
  }

  ROS_INFO("Tare: acquiring %d samples (period=%d ms) with outlier rejection (k=%.2f)...",
           tare_samples, tare_period_ms, tare_outlier_k_sigma);

  int kept = 0;
  FtRaw tareOffset = compute_tare_outlier_rejection(
      ft,
      countsPerForce,
      countsPerTorque,
      tare_samples,
      tare_period_ms,
      tare_outlier_k_sigma,
      &kept);

  ROS_INFO("Tare kept %d/%d samples after outlier rejection.", kept, tare_samples);
  ROS_INFO("Tare offset (counts): Fx=%d Fy=%d Fz=%d Tx=%d Ty=%d Tz=%d",
           tareOffset.fx, tareOffset.fy, tareOffset.fz,
           tareOffset.tx, tareOffset.ty, tareOffset.tz);

  // --- Publish loop -----------------------------------------------------------
  ros::Rate rate(rate_hz);

  while (ros::ok()) {
    FtRaw raw;
    if (!ft.read_ft_counts(raw)) {
      ROS_WARN_THROTTLE(1.0, "SDO read failed.");
      ros::spinOnce();
      rate.sleep();
      continue;
    }

    // Apply tare offset in counts.
    const FtRaw zeroed = subtract_offset(raw, tareOffset);

    // Convert to physical units (N, Nm).
    const FtScaled phys = FtSensor::to_physical(zeroed, countsPerForce, countsPerTorque);

    geometry_msgs::WrenchStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = frame_id;

    msg.wrench.force.x  = phys.fx_N;
    msg.wrench.force.y  = phys.fy_N;
    msg.wrench.force.z  = phys.fz_N;

    msg.wrench.torque.x = phys.tx_Nm;
    msg.wrench.torque.y = phys.ty_Nm;
    msg.wrench.torque.z = phys.tz_Nm;

    pub.publish(msg);

    ros::spinOnce();
    rate.sleep();
  }

  ROS_INFO("Shutting down: closing EtherCAT master...");
  master.close();
  return 0;
}
