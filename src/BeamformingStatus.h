// SPDX-License-Identifier: GPL-2.0-only
#pragma once
#include <chrono>
#include <cstdint>

namespace devourer {
struct BeamformingStatus {
  bool active = false;
  uint64_t feedback_reports = 0;
  uint64_t rejected_reports = 0;
  uint64_t feedback_age_ms = 0;
};
inline uint64_t beamforming_now_ms() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}
}
