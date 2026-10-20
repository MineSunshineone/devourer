// SPDX-License-Identifier: GPL-2.0-only
#pragma once

#include "BfReportDecode.h"
#include <array>
#include <cstring>
#include <optional>
#include <span>

namespace devourer::bf {

inline bool valid_peer(const uint8_t self[6], const uint8_t peer[6]) {
  const auto unicast = [](const uint8_t mac[6]) {
    uint8_t any = 0;
    for (unsigned i = 0; i < 6; ++i) any |= mac[i];
    return any != 0 && (mac[0] & 1) == 0;
  };
  return unicast(self) && unicast(peer) && std::memcmp(self, peer, 6) != 0;
}

inline bool feedback_fresh(uint64_t now, uint64_t received, uint64_t max_age) {
  return received != 0 && now >= received && now - received <= max_age;
}

inline bool poll_required(bool fresh, bool active, bool requested, bool allow_apply) {
  return fresh ? allow_apply && requested : active;
}

// Compare validated 2-TX SU steering directions, excluding SNR and dialog token.
class FeedbackChange {
public:
  void reset() { _phi.clear(); _psi.clear(); _format = 0; }

  std::optional<uint16_t> observe(std::span<const uint8_t> frame, bool fcs_present) {
    if (frame.size() < 30) return std::nullopt;
    const unsigned nc = (frame[26] & 7) + 1, nr = ((frame[26] >> 3) & 7) + 1;
    const unsigned bw = frame[26] >> 6, ng = frame[27] & 3;
    if (nr != 2 || nc > 2 || bw > 2 || ng == 3 || (frame[27] & 0x78))
      return std::nullopt;
    const int bpsi = frame[27] & 4 ? 4 : 2, bphi = bpsi + 2;
    const int ns = report_ns(bw, 1 << ng);
    const size_t bytes = (ns * (bphi + bpsi) + 7) / 8;
    if (frame.size() != 29 + nc + bytes + (fcs_present ? 4 : 0))
      return std::nullopt;
    ReportHdr hdr;
    hdr.nr = nr; hdr.nc = nc; hdr.ns = ns;
    hdr.angles = frame.data() + 29 + nc; hdr.angle_len = bytes;
    if (!decode_angles(hdr, bphi, bpsi, &_next_phi, &_next_psi)) return std::nullopt;
    const unsigned format = frame[26] | ((frame[27] & 7u) << 8);
    std::optional<uint16_t> result;
    if (_format == format && _phi.size() == _next_phi.size()) {
      double change = 0;
      for (size_t i = 0; i < _phi.size(); ++i) {
        const double a = std::cos(_psi[i]) * std::cos(_next_psi[i]);
        const double b = std::sin(_psi[i]) * std::sin(_next_psi[i]);
        const double correlation = a*a + b*b + 2*a*b*std::cos(_phi[i] - _next_phi[i]);
        change += std::clamp(1.0 - correlation, 0.0, 1.0);
      }
      result = static_cast<uint16_t>(std::lround(change * 1000 / _phi.size()));
    }
    _phi.swap(_next_phi); _psi.swap(_next_psi); _format = format;
    return result;
  }
private:
  unsigned _format = 0;
  std::vector<double> _phi, _psi, _next_phi, _next_psi;
};

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
