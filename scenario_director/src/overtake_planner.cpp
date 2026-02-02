#include "scenario_director/overtake_planner.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <numeric>
#include <sstream>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace scenario_director {

namespace {

std::vector<double> linspace(double start, double end, int num) {
  std::vector<double> out;
  if (num <= 0) {
    return out;
  }
  if (num == 1) {
    out.push_back(start);
    return out;
  }
  out.resize(static_cast<size_t>(num));
  double step = (end - start) / static_cast<double>(num - 1);
  for (int i = 0; i < num; ++i) {
    out[static_cast<size_t>(i)] = start + step * static_cast<double>(i);
  }
  return out;
}

std::vector<double> movingAverage(const std::vector<double> &values, int window) {
  if (window <= 1 || values.empty()) {
    return values;
  }
  if (window % 2 == 0) {
    window += 1;
  }
  int half = window / 2;
  std::vector<double> out(values.size());
  for (size_t i = 0; i < values.size(); ++i) {
    int start = static_cast<int>(i) - half;
    int end = static_cast<int>(i) + half;
    start = std::max(start, 0);
    end = std::min(end, static_cast<int>(values.size()) - 1);
    double sum = 0.0;
    int count = 0;
    for (int j = start; j <= end; ++j) {
      sum += values[static_cast<size_t>(j)];
      ++count;
    }
    out[i] = sum / std::max(count, 1);
  }
  return out;
}

std::vector<double> gradient(const std::vector<double> &v) {
  std::vector<double> out(v.size(), 0.0);
  if (v.size() < 2) {
    return out;
  }
  out[0] = v[1] - v[0];
  for (size_t i = 1; i + 1 < v.size(); ++i) {
    out[i] = 0.5 * (v[i + 1] - v[i - 1]);
  }
  out[v.size() - 1] = v[v.size() - 1] - v[v.size() - 2];
  return out;
}

std::vector<double> unwrapYaw(const std::vector<double> &yaw) {
  if (yaw.empty()) {
    return yaw;
  }
  std::vector<double> out = yaw;
  for (size_t i = 1; i < out.size(); ++i) {
    double delta = out[i] - out[i - 1];
    if (delta > M_PI) {
      out[i] -= 2.0 * M_PI;
    } else if (delta < -M_PI) {
      out[i] += 2.0 * M_PI;
    }
  }
  return out;
}

std::vector<double> computeYawFromXY(const std::vector<double> &x,
                                     const std::vector<double> &y) {
  std::vector<double> yaw(x.size(), 0.0);
  if (x.size() < 2) {
    return yaw;
  }
  for (size_t i = 0; i + 1 < x.size(); ++i) {
    double dx = x[i + 1] - x[i];
    double dy = y[i + 1] - y[i];
    yaw[i] = std::atan2(dy, dx);
  }
  yaw[x.size() - 1] = yaw[x.size() - 2];
  return unwrapYaw(yaw);
}

std::vector<double> computePathYaw(const std::vector<double> &x,
                                   const std::vector<double> &y) {
  return computeYawFromXY(x, y);
}

std::vector<double> computeArcLength(const std::vector<double> &x,
                                     const std::vector<double> &y) {
  std::vector<double> s(x.size(), 0.0);
  for (size_t i = 1; i < x.size(); ++i) {
    double dx = x[i] - x[i - 1];
    double dy = y[i] - y[i - 1];
    s[i] = s[i - 1] + std::hypot(dx, dy);
  }
  return s;
}

std::vector<double> computeCurvature(const std::vector<double> &yaw,
                                     const std::vector<double> &s) {
  std::vector<double> dyaw = gradient(yaw);
  std::vector<double> ds = gradient(s);
  std::vector<double> kappa(yaw.size(), 0.0);
  for (size_t i = 0; i < kappa.size(); ++i) {
    double denom = std::abs(ds[i]) < 1e-6 ? 1e-6 : ds[i];
    kappa[i] = dyaw[i] / denom;
  }
  return kappa;
}

std::vector<double> applyLateralLimit(const std::vector<double> &v,
                                      const std::vector<double> &kappa,
                                      double alat_max) {
  std::vector<double> out = v;
  for (size_t i = 0; i < out.size(); ++i) {
    double k = std::abs(kappa[i]);
    if (k > 1e-6) {
      double limit = std::sqrt(std::max(alat_max / k, 0.0));
      out[i] = std::min(out[i], limit);
    }
  }
  return out;
}

std::vector<double> applyLongitudinalLimits(const std::vector<double> &v,
                                            const std::vector<double> &s,
                                            double amax,
                                            double bmax) {
  std::vector<double> out = v;
  if (out.empty()) {
    return out;
  }
  for (size_t i = 1; i < out.size(); ++i) {
    double ds = s[i] - s[i - 1];
    double v_allow = std::sqrt(std::max(out[i - 1] * out[i - 1] + 2.0 * amax * ds, 0.0));
    out[i] = std::min(out[i], v_allow);
  }
  for (size_t i = out.size(); i-- > 1;) {
    double ds = s[i] - s[i - 1];
    double v_allow = std::sqrt(std::max(out[i] * out[i] + 2.0 * bmax * ds, 0.0));
    out[i - 1] = std::min(out[i - 1], v_allow);
  }
  return out;
}

std::vector<double> smoothstepProfile(const std::vector<double> &s,
                                      double s0, double s1) {
  std::vector<double> t(s.size(), 0.0);
  double denom = std::max(s1 - s0, 1e-6);
  for (size_t i = 0; i < s.size(); ++i) {
    double ti = (s[i] - s0) / denom;
    ti = std::clamp(ti, 0.0, 1.0);
    t[i] = ti * ti * (3.0 - 2.0 * ti);
  }
  return t;
}

void offsetPath(const std::vector<double> &x,
                const std::vector<double> &y,
                const std::vector<double> &yaw,
                const std::vector<double> &l,
                std::vector<double> *x_out,
                std::vector<double> *y_out) {
  x_out->resize(x.size());
  y_out->resize(y.size());
  for (size_t i = 0; i < x.size(); ++i) {
    double nx = -std::sin(yaw[i]);
    double ny = std::cos(yaw[i]);
    (*x_out)[i] = x[i] + l[i] * nx;
    (*y_out)[i] = y[i] + l[i] * ny;
  }
}

}  // namespace

OvertakePlanner::OvertakePlanner(const std::string &map_dir,
                                 const std::string &map_yaml,
                                 const std::string &waypoints_path,
                                 const OvertakeConfig &config)
: waypoints_path_(waypoints_path),
  config_(config),
  checker_(map_dir, map_yaml) {}

void OvertakePlanner::loadWaypoints() {
  std::ifstream file(waypoints_path_);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to open waypoints: " + waypoints_path_);
  }

  std::string line;
  bool header_checked = false;
  while (std::getline(file, line)) {
    if (line.empty()) {
      continue;
    }
    std::stringstream ss(line);
    std::vector<std::string> tokens;
    std::string token;
    while (std::getline(ss, token, ',')) {
      tokens.push_back(token);
    }
    if (!header_checked) {
      header_checked = true;
      if (!tokens.empty()) {
        try {
          std::stod(tokens[0]);
        } catch (...) {
          continue;
        }
      }
    }
    if (tokens.size() < 2) {
      continue;
    }
    x_.push_back(std::stod(tokens[0]));
    y_.push_back(std::stod(tokens[1]));
    if (tokens.size() >= 3) {
      yaw_.push_back(std::stod(tokens[2]));
    } else {
      yaw_.push_back(std::numeric_limits<double>::quiet_NaN());
    }
    if (tokens.size() >= 4) {
      v_raw_.push_back(std::stod(tokens[3]));
    } else {
      v_raw_.push_back(std::numeric_limits<double>::quiet_NaN());
    }
  }
  if (x_.empty()) {
    throw std::runtime_error("Waypoints contain no data: " + waypoints_path_);
  }
}

void OvertakePlanner::computeArcLength() {
  s_ = ::scenario_director::computeArcLength(x_, y_);
}

void OvertakePlanner::computeYawIfMissing() {
  bool all_nan = true;
  for (double v : yaw_) {
    if (!std::isnan(v)) {
      all_nan = false;
      break;
    }
  }
  if (all_nan) {
    yaw_ = computeYawFromXY(x_, y_);
    return;
  }
  yaw_ = unwrapYaw(yaw_);
}

void OvertakePlanner::computeCurvature() {
  kappa_ = ::scenario_director::computeCurvature(yaw_, s_);
}

void OvertakePlanner::smoothCurvature() {
  kappa_ = movingAverage(kappa_, config_.smooth_curvature_window);
}

void OvertakePlanner::computeBaseSpeed() {
  v_ref_.resize(v_raw_.size());
  double v_median = 6.0;
  std::vector<double> temp;
  for (double v : v_raw_) {
    if (!std::isnan(v)) {
      temp.push_back(v);
    }
  }
  if (!temp.empty()) {
    std::sort(temp.begin(), temp.end());
    v_median = temp[temp.size() / 2];
  }
  for (size_t i = 0; i < v_raw_.size(); ++i) {
    v_ref_[i] = std::isnan(v_raw_[i]) ? v_median : v_raw_[i];
  }
  v_ref_ = applyLateralLimit(v_ref_, kappa_, config_.alat_max);
}

TrackLimits OvertakePlanner::estimateTrackLimits() const {
  TrackLimits limits;
  limits.left.resize(x_.size(), 0.0);
  limits.right.resize(x_.size(), 0.0);

  for (size_t i = 0; i < x_.size(); ++i) {
    double nx = -std::sin(yaw_[i]);
    double ny = std::cos(yaw_[i]);
    double left = 0.0;
    double right = 0.0;
    for (double dist = config_.raycast_step_m; dist <= config_.raycast_max_m + 1e-6; dist += config_.raycast_step_m) {
      double lx = x_[i] + nx * dist;
      double ly = y_[i] + ny * dist;
      if (checker_.isDrivableWithMargin(lx, ly, config_.safety_margin_m)) {
        left = dist;
      } else {
        break;
      }
    }
    for (double dist = config_.raycast_step_m; dist <= config_.raycast_max_m + 1e-6; dist += config_.raycast_step_m) {
      double rx = x_[i] - nx * dist;
      double ry = y_[i] - ny * dist;
      if (checker_.isDrivableWithMargin(rx, ry, config_.safety_margin_m)) {
        right = dist;
      } else {
        break;
      }
    }
    limits.left[i] = std::max(0.0, left - config_.safety_margin_m);
    limits.right[i] = std::max(0.0, right - config_.safety_margin_m);
  }
  return limits;
}

std::vector<CornerSegment> OvertakePlanner::detectCorners() const {
  std::vector<double> abs_k(kappa_.size());
  for (size_t i = 0; i < kappa_.size(); ++i) {
    abs_k[i] = std::abs(kappa_[i]);
  }
  std::vector<double> sorted = abs_k;
  std::sort(sorted.begin(), sorted.end());
  double percentile = config_.curvature_threshold_percentile / 100.0;
  size_t idx = static_cast<size_t>(percentile * (sorted.size() - 1));
  double thresh = sorted[idx];

  std::vector<bool> mask(abs_k.size(), false);
  for (size_t i = 0; i < abs_k.size(); ++i) {
    mask[i] = abs_k[i] >= thresh;
  }

  std::vector<std::pair<int, int>> segments;
  bool in_seg = false;
  int start = 0;
  for (size_t i = 0; i < mask.size(); ++i) {
    if (mask[i] && !in_seg) {
      in_seg = true;
      start = static_cast<int>(i);
    } else if (!mask[i] && in_seg) {
      in_seg = false;
      int end = static_cast<int>(i) - 1;
      if (s_[end] - s_[start] >= config_.min_corner_length_m) {
        segments.emplace_back(start, end);
      }
    }
  }
  if (in_seg) {
    int end = static_cast<int>(mask.size()) - 1;
    if (s_[end] - s_[start] >= config_.min_corner_length_m) {
      segments.emplace_back(start, end);
    }
  }

  std::vector<CornerSegment> corners;
  for (size_t i = 0; i < segments.size(); ++i) {
    auto [seg_start, seg_end] = segments[i];
    double max_val = 0.0;
    int apex = seg_start;
    for (int j = seg_start; j <= seg_end; ++j) {
      double val = std::abs(kappa_[j]);
      if (val > max_val) {
        max_val = val;
        apex = j;
      }
    }
    int turn_dir = (kappa_[apex] >= 0.0) ? 1 : -1;
    corners.push_back({static_cast<int>(i), seg_start, seg_end, apex, turn_dir});
  }
  return corners;
}

std::vector<Candidate> OvertakePlanner::generateCandidates(const TrackLimits &limits,
                                                           const std::vector<CornerSegment> &corners) const {
  std::vector<Candidate> candidates;
  int candidate_id = 0;
  auto scales = linspace(0.6, 1.0, std::max(1, config_.num_candidates_per_corner));

  for (const auto &corner : corners) {
    double s_start = s_[corner.start_idx];
    double s_end = s_[corner.end_idx];
    double s_apex = s_[corner.apex_idx];
    double s_entry = std::max(s_start - config_.corner_buffer_m, s_.front());
    double s_exit = std::min(s_end + config_.corner_buffer_m, s_.back());
    double s_build_start = std::max(s_entry - config_.corner_buffer_m, s_.front());

    const std::vector<double> &left = limits.left;
    const std::vector<double> &right = limits.right;

    int turn_dir = corner.turn_dir;
    double outside_sign = (turn_dir > 0) ? -1.0 : 1.0;
    double inside_sign = (turn_dir > 0) ? 1.0 : -1.0;

    double cross_start = s_apex + 0.15 * (s_end - s_start);
    double cross_end = s_apex + 0.35 * (s_end - s_start);

    double inside_margin = 0.0;
    int count = 0;
    for (int i = corner.start_idx; i <= corner.end_idx; ++i) {
      inside_margin += (turn_dir > 0) ? left[i] : right[i];
      ++count;
    }
    inside_margin = (count > 0) ? inside_margin / count : 0.0;

    for (double scale : scales) {
      Candidate cand;
      cand.candidate_id = candidate_id++;
      cand.corner_id = corner.corner_id;
      cand.family = 'A';
      cand.l.resize(s_.size(), 0.0);
      cand.phase.resize(s_.size(), "");

      for (size_t i = 0; i < s_.size(); ++i) {
        double outside_cap = (turn_dir > 0) ? right[i] : left[i];
        double inside_cap = (turn_dir > 0) ? left[i] : right[i];
        double outside = config_.family_a_outside_frac * outside_cap * scale;
        double inside = config_.family_a_inside_frac * inside_cap * scale;
        if (s_[i] <= s_entry) {
          auto t = smoothstepProfile(s_, s_build_start, s_entry);
          cand.l[i] = outside_sign * outside * t[i];
          cand.phase[i] = "build";
        } else if (s_[i] <= cross_start) {
          cand.l[i] = outside_sign * outside;
          cand.phase[i] = "hold";
        } else if (s_[i] <= cross_end) {
          auto t = smoothstepProfile(s_, cross_start, cross_end);
          cand.l[i] = outside_sign * outside + (inside_sign * inside - outside_sign * outside) * t[i];
          cand.phase[i] = "cross";
        } else if (s_[i] <= s_exit) {
          auto t = smoothstepProfile(s_, cross_end, s_exit);
          cand.l[i] = inside_sign * inside * (1.0 - t[i]);
          cand.phase[i] = "unwind";
        }
      }

      offsetPath(x_, y_, yaw_, cand.l, &cand.x, &cand.y);
      cand.yaw = computePathYaw(cand.x, cand.y);
      cand.v = v_ref_;
      candidates.push_back(cand);
    }

    if (inside_margin >= config_.min_inside_offset_m) {
      for (double scale : scales) {
        Candidate cand;
        cand.candidate_id = candidate_id++;
        cand.corner_id = corner.corner_id;
        cand.family = 'B';
        cand.l.resize(s_.size(), 0.0);
        cand.phase.resize(s_.size(), "");

        for (size_t i = 0; i < s_.size(); ++i) {
          double inside_cap = (turn_dir > 0) ? left[i] : right[i];
          double inside = config_.family_b_inside_frac * inside_cap * scale;
          if (s_[i] <= s_entry) {
            auto t = smoothstepProfile(s_, s_build_start, s_entry);
            cand.l[i] = inside_sign * inside * t[i];
            cand.phase[i] = "build";
          } else if (s_[i] <= s_apex) {
            cand.l[i] = inside_sign * inside;
            cand.phase[i] = "hold";
          } else if (s_[i] <= s_exit) {
            auto t = smoothstepProfile(s_, s_apex, s_exit);
            cand.l[i] = inside_sign * inside * (1.0 - t[i]);
            cand.phase[i] = "unwind";
          }
        }

        offsetPath(x_, y_, yaw_, cand.l, &cand.x, &cand.y);
        cand.yaw = computePathYaw(cand.x, cand.y);
        cand.v = v_ref_;
        candidates.push_back(cand);
      }
    }

    for (double scale : scales) {
      Candidate cand;
      cand.candidate_id = candidate_id++;
      cand.corner_id = corner.corner_id;
      cand.family = 'C';
      cand.l.resize(s_.size(), 0.0);
      cand.phase.resize(s_.size(), "");

      for (size_t i = 0; i < s_.size(); ++i) {
        double outside_cap = (turn_dir > 0) ? right[i] : left[i];
        double inside_cap = (turn_dir > 0) ? left[i] : right[i];
        double feint = config_.family_c_feint_frac * outside_cap * scale;
        double sw = config_.family_c_switch_frac * inside_cap * scale;
        double feint_end = s_start + config_.dummy_feint_distance_m;

        if (s_[i] <= feint_end) {
          auto t = smoothstepProfile(s_, s_build_start, feint_end);
          cand.l[i] = outside_sign * feint * t[i];
          cand.phase[i] = "feint";
        } else if (s_[i] <= s_apex) {
          cand.l[i] = outside_sign * feint;
          cand.phase[i] = "hold";
        } else if (s_[i] <= s_apex + 0.25 * (s_end - s_start)) {
          auto t = smoothstepProfile(s_, s_apex, s_apex + 0.25 * (s_end - s_start));
          cand.l[i] = outside_sign * feint + (inside_sign * sw - outside_sign * feint) * t[i];
          cand.phase[i] = "cross";
        } else if (s_[i] <= s_exit) {
          auto t = smoothstepProfile(s_, s_apex + 0.25 * (s_end - s_start), s_exit);
          cand.l[i] = inside_sign * sw * (1.0 - t[i]);
          cand.phase[i] = "unwind";
        }
      }

      offsetPath(x_, y_, yaw_, cand.l, &cand.x, &cand.y);
      cand.yaw = computePathYaw(cand.x, cand.y);
      cand.v = v_ref_;
      candidates.push_back(cand);
    }
  }

  return candidates;
}

void OvertakePlanner::applySpeedProfile(Candidate &cand, const CornerSegment &corner) const {
  double s_start = s_[corner.start_idx];
  double s_apex = s_[corner.apex_idx];

  for (size_t i = 0; i < cand.v.size(); ++i) {
    if (cand.family == 'A') {
      if (s_[i] < s_apex) {
        cand.v[i] *= config_.speed_entry_scale;
      } else {
        cand.v[i] *= config_.speed_exit_scale;
      }
    } else if (cand.family == 'B') {
      double brake_start = std::max(s_apex - config_.late_brake_distance_m, s_start);
      if (s_[i] >= brake_start) {
        cand.v[i] *= config_.speed_entry_scale;
      }
    } else if (cand.family == 'C') {
      if (s_[i] < s_apex) {
        cand.v[i] *= (0.95 * config_.speed_entry_scale);
      } else {
        cand.v[i] *= (0.98 * config_.speed_exit_scale);
      }
    }
  }

  cand.v = applyLateralLimit(cand.v, kappa_, config_.alat_max);
  cand.v = applyLongitudinalLimits(cand.v, s_, config_.amax, config_.bmax);
}

std::pair<double, double> OvertakePlanner::validateCandidate(const Candidate &cand) const {
  int off_count = 0;
  for (size_t i = 0; i < cand.x.size(); ++i) {
    if (!checker_.isDrivableWithMargin(cand.x[i], cand.y[i], config_.safety_margin_m)) {
      off_count++;
    }
  }
  double ratio = cand.x.empty() ? 1.0 : static_cast<double>(off_count) / cand.x.size();
  return {ratio, static_cast<double>(off_count)};
}

void OvertakePlanner::writeTrackLimits(const TrackLimits &limits, const std::string &out_dir) const {
  std::ofstream out(out_dir + "/track_limits.csv");
  out << "wp_index,s,x,y,yaw,l_left_max,l_right_max\n";
  for (size_t i = 0; i < x_.size(); ++i) {
    out << i << "," << s_[i] << "," << x_[i] << "," << y_[i]
        << "," << yaw_[i] << "," << limits.left[i] << "," << limits.right[i] << "\n";
  }
}

void OvertakePlanner::writeCorners(const std::vector<CornerSegment> &corners,
                                  const TrackLimits &limits,
                                  const std::string &out_dir) const {
  std::ofstream out(out_dir + "/corners.csv");
  out << "corner_id,start_wp,end_wp,apex_wp,turn_dir,available_left_median,available_right_median\n";
  for (const auto &corner : corners) {
    double left_sum = 0.0;
    double right_sum = 0.0;
    int count = 0;
    for (int i = corner.start_idx; i <= corner.end_idx; ++i) {
      left_sum += limits.left[i];
      right_sum += limits.right[i];
      ++count;
    }
    double left_med = (count > 0) ? left_sum / count : 0.0;
    double right_med = (count > 0) ? right_sum / count : 0.0;
    out << corner.corner_id << "," << corner.start_idx << "," << corner.end_idx
        << "," << corner.apex_idx << "," << corner.turn_dir << "," << left_med
        << "," << right_med << "\n";
  }
}

void OvertakePlanner::writeCandidates(const std::vector<Candidate> &candidates,
                                     const std::string &out_dir) const {
  std::ofstream out(out_dir + "/candidates.csv");
  out << "candidate_id,corner_id,family,wp_index,s,l,x,y,yaw,v,phase,quality_score\n";
  for (const auto &cand : candidates) {
    for (size_t i = 0; i < cand.x.size(); ++i) {
      out << cand.candidate_id << "," << cand.corner_id << "," << cand.family
          << "," << i << "," << s_[i] << "," << cand.l[i] << ","
          << cand.x[i] << "," << cand.y[i] << "," << cand.yaw[i] << ","
          << cand.v[i] << "," << cand.phase[i] << "," << cand.quality_score << "\n";
    }
  }
}

void OvertakePlanner::writePlots(const std::vector<Candidate> &candidates,
                                const std::vector<CornerSegment> &corners,
                                const std::string &out_dir) const {
  cv::Mat base = checker_.mapImage().clone();
  cv::Mat overlay;
  cv::cvtColor(base, overlay, cv::COLOR_GRAY2BGR);

  for (size_t i = 1; i < x_.size(); ++i) {
    auto [x0, y0] = checker_.worldToPixel(x_[i - 1], y_[i - 1]);
    auto [x1, y1] = checker_.worldToPixel(x_[i], y_[i]);
    cv::line(overlay, cv::Point(x0, y0), cv::Point(x1, y1), cv::Scalar(0, 0, 255), 1);
  }
  cv::imwrite(out_dir + "/map_overlay.png", overlay);

  for (const auto &corner : corners) {
    cv::Mat corner_img = overlay.clone();
    int min_x = std::numeric_limits<int>::max();
    int min_y = std::numeric_limits<int>::max();
    int max_x = std::numeric_limits<int>::min();
    int max_y = std::numeric_limits<int>::min();

    for (int i = corner.start_idx; i <= corner.end_idx; ++i) {
      auto [px, py] = checker_.worldToPixel(x_[i], y_[i]);
      min_x = std::min(min_x, px);
      min_y = std::min(min_y, py);
      max_x = std::max(max_x, px);
      max_y = std::max(max_y, py);
    }

    for (const auto &cand : candidates) {
      if (cand.corner_id != corner.corner_id) {
        continue;
      }
      cv::Scalar color = (cand.family == 'A') ? cv::Scalar(0, 255, 0)
                                              : (cand.family == 'B' ? cv::Scalar(255, 0, 0)
                                                                    : cv::Scalar(0, 255, 255));
      for (size_t i = 1; i < cand.x.size(); ++i) {
        auto [x0, y0] = checker_.worldToPixel(cand.x[i - 1], cand.y[i - 1]);
        auto [x1, y1] = checker_.worldToPixel(cand.x[i], cand.y[i]);
        min_x = std::min(min_x, std::min(x0, x1));
        min_y = std::min(min_y, std::min(y0, y1));
        max_x = std::max(max_x, std::max(x0, x1));
        max_y = std::max(max_y, std::max(y0, y1));
        cv::line(corner_img, cv::Point(x0, y0), cv::Point(x1, y1), color, 1);
      }
    }

    int pad = 20;
    min_x = std::max(min_x - pad, 0);
    min_y = std::max(min_y - pad, 0);
    max_x = std::min(max_x + pad, corner_img.cols - 1);
    max_y = std::min(max_y + pad, corner_img.rows - 1);
    if (min_x < max_x && min_y < max_y) {
      cv::Rect roi(min_x, min_y, max_x - min_x, max_y - min_y);
      cv::Mat cropped = corner_img(roi).clone();
      cv::imwrite(out_dir + "/corner_" + std::to_string(corner.corner_id) + ".png", cropped);
    } else {
      cv::imwrite(out_dir + "/corner_" + std::to_string(corner.corner_id) + ".png", corner_img);
    }
  }
}

void OvertakePlanner::generate(const std::string &out_dir) {
  loadWaypoints();
  computeArcLength();
  computeYawIfMissing();
  computeCurvature();
  smoothCurvature();
  computeBaseSpeed();

  TrackLimits limits = estimateTrackLimits();
  auto corners = detectCorners();
  auto candidates = generateCandidates(limits, corners);

  std::vector<Candidate> filtered;
  for (auto &cand : candidates) {
    double clamp_count = 0.0;
    for (size_t i = 0; i < cand.l.size(); ++i) {
      double l = cand.l[i];
      if (l > limits.left[i]) {
        cand.l[i] = limits.left[i];
        clamp_count += 1.0;
      }
      if (l < -limits.right[i]) {
        cand.l[i] = -limits.right[i];
        clamp_count += 1.0;
      }
    }
    offsetPath(x_, y_, yaw_, cand.l, &cand.x, &cand.y);
    cand.yaw = computePathYaw(cand.x, cand.y);

    const auto corner = corners[static_cast<size_t>(cand.corner_id)];
    applySpeedProfile(cand, corner);

    auto [off_ratio, off_count] = validateCandidate(cand);
    double clamp_ratio = cand.l.empty() ? 1.0 : clamp_count / cand.l.size();
    if (off_ratio > config_.offtrack_max_ratio || clamp_ratio > config_.clamp_max_ratio) {
      continue;
    }

    double mean_v = std::accumulate(cand.v.begin(), cand.v.end(), 0.0) / cand.v.size();
    double mean_l = 0.0;
    for (double val : cand.l) {
      mean_l += std::abs(val);
    }
    mean_l /= cand.l.size();
    cand.quality_score = -mean_v + mean_l + off_ratio * 5.0;

    filtered.push_back(cand);
  }

  std::filesystem::create_directories(out_dir);
  writeTrackLimits(limits, out_dir);
  writeCorners(corners, limits, out_dir);
  writeCandidates(filtered, out_dir);
  writePlots(filtered, corners, out_dir);
}

}  // namespace scenario_director
