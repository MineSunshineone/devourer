// SPDX-License-Identifier: GPL-2.0-only
#pragma once

#include "BfReportDecode.h"
#include "TxMode.h"
#include <array>
#include <cstring>
#include <span>

namespace devourer::bf {

inline TxMode vht_su_mode(TxMode mode) {
  if (mode.mode == TxMode::Mode::HT && mode.ht_mcs <= 7) {
    mode.mode = TxMode::Mode::VHT;
    mode.vht_mcs = mode.ht_mcs;
    mode.vht_nss = 1;
  }
  mode.stbc = false;
  return mode;
}

// Single unassociated SU peer, AID 0, one feedback column (IEEE VHT NDPA).
inline std::array<uint8_t, 19> vht_ndpa(const uint8_t self[6],
                                         const uint8_t peer[6], uint8_t token,
                                         uint8_t bandwidth = 0) {
  // Vendor NDPA duration: two 16-us SIFS + 44-us NDP + feedback airtime.
  const uint16_t duration = 76 + (bandwidth == 2 ? 40 : bandwidth == 1 ? 87 : 180);
  std::array<uint8_t, 19> frame{0x54, 0};
  frame[2] = static_cast<uint8_t>(duration);
  frame[3] = static_cast<uint8_t>(duration >> 8);
  std::memcpy(frame.data() + 4, peer, 6);
  std::memcpy(frame.data() + 10, self, 6);
  frame[16] = static_cast<uint8_t>((token & 63) << 2);
  return frame;
}

inline bool is_vht_ndpa(std::span<const uint8_t> frame) {
  return frame.size() == 19 && frame[0] == 0x54 && frame[1] == 0;
}

// Realtek RX includes FCS. Only complete, unsegmented SU matrices from this
// peer to this sounder may enable steering on the 2T2R backend.
inline bool valid_vht_feedback(std::span<const uint8_t> frame,
                                 const uint8_t self[6], const uint8_t peer[6],
                                 uint8_t bandwidth, uint8_t token,
                                 bool fcs_present = true) {
  if (frame.size() < 30 || (frame[0] != 0xd0 && frame[0] != 0xe0) ||
      (frame[1] & 0xc7) != 0 || frame[24] != 0x15 || frame[25] != 0 ||
      (frame[22] & 0x0f) != 0 ||
      std::memcmp(frame.data() + 4, self, 6) != 0 ||
      std::memcmp(frame.data() + 10, peer, 6) != 0)
    return false;
  const unsigned nc = (frame[26] & 7) + 1;
  const unsigned nr = ((frame[26] >> 3) & 7) + 1;
  const unsigned bw = frame[26] >> 6;
  const unsigned grouping = frame[27] & 3;
  if (nr != 2 || nc > nr || bw != bandwidth || grouping == 3 ||
      (frame[27] & 0x78) != 0 || (frame[28] & 3) != 0 ||
      (frame[28] >> 2) != (token & 63))
    return false;
  const auto tones = report_ns(bw, 1 << grouping);
  const unsigned angle_bits = (frame[27] & 4) ? 10 : 6;
  const auto matrix_bytes = (tones * angle_bits + 7) / 8;
  return tones != 0 && frame.size() == 29 + nc + matrix_bytes +
                                                (fcs_present ? 4 : 0);
}

} // namespace devourer::bf
