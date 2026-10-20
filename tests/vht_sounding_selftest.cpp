// SPDX-License-Identifier: GPL-2.0-only
#include "VhtSounding.h"
#include "jaguar3/FrameParserJaguar3.h"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <vector>

int main() {
  const uint8_t self[6]{2, 1, 2, 3, 4, 5}, peer[6]{2, 6, 7, 8, 9, 10};
  assert(!devourer::bf::poll_required(true, true, false, true));
  assert(!devourer::bf::poll_required(true, true, true, false));
  assert(!devourer::bf::poll_required(false, false, true, true));
  assert(devourer::bf::poll_required(true, false, true, true));
  assert(devourer::bf::poll_required(false, true, false, false));
  assert(devourer::bf::poll_required(false, true, true, true));
  const uint8_t zero[6]{}, multicast[6]{3, 1, 2, 3, 4, 5};
  assert(devourer::bf::valid_peer(self, peer));
  assert(!devourer::bf::valid_peer(self, self));
  assert(!devourer::bf::valid_peer(zero, peer));
  assert(!devourer::bf::valid_peer(self, zero));
  assert(!devourer::bf::valid_peer(multicast, peer));
  assert(!devourer::bf::valid_peer(self, multicast));
  const auto ndpa = devourer::bf::vht_ndpa(self, peer, 65);
  assert(devourer::bf::is_vht_ndpa(ndpa));
  assert(ndpa[16] == 4 && ndpa[17] == 0 && ndpa[18] == 0);
  assert((ndpa[2] | (ndpa[3] << 8)) == 256);
  const auto wide = devourer::bf::vht_ndpa(self, peer, 1, 2);
  assert((wide[2] | (wide[3] << 8)) == 116);
  assert(std::memcmp(ndpa.data() + 4, peer, 6) == 0);
  assert(std::memcmp(ndpa.data() + 10, self, 6) == 0);
  std::vector<uint8_t> feedback(99);
  feedback[0] = 0xe0;
  std::memcpy(feedback.data() + 4, self, 6);
  std::memcpy(feedback.data() + 10, peer, 6);
  feedback[24] = 0x15;
  feedback[26] = 8; // Nc=1, Nr=2, BW20.
  feedback[27] = 4; // Ng=1, SU codebook 6/4 bits.
  feedback[28] = 4; // Dialog token 1.
  const auto valid = [&](const auto& frame) {
    return devourer::bf::valid_vht_feedback(frame, self, peer, 0, 1);
  };
  assert(valid(feedback));
  for (unsigned bw = 0; bw < 3; ++bw) for (unsigned ng = 0; ng < 3; ++ng)
    for (bool codebook : {false, true}) {
      const unsigned bpsi = codebook ? 4 : 2, bphi = bpsi + 2;
      const auto report = [&](unsigned phi, unsigned psi) {
        auto frame = feedback;
        const unsigned tones = devourer::bf::report_ns(bw, 1 << ng);
        frame.resize(30 + (tones * (bphi + bpsi) + 7) / 8 + 4);
        frame[26] = static_cast<uint8_t>((bw << 6) | 8);
        frame[27] = static_cast<uint8_t>(ng | (codebook ? 4 : 0));
        std::fill(frame.begin() + 30, frame.end(), 0);
        unsigned bit = 0;
        const auto put = [&](unsigned value, unsigned count) {
          for (unsigned i = 0; i < count; ++i, ++bit)
            frame[30 + bit / 8] |= ((value >> i) & 1u) << (bit % 8);
        };
        for (unsigned i = 0; i < tones; ++i) { put(phi, bphi); put(psi, bpsi); }
        return frame;
      };
      const auto first = report(0, (1u << (bpsi - 1)) - 1);
      devourer::bf::FeedbackChange change;
      assert(!change.observe(first, true));
      assert(change.observe(first, true) == 0);
      auto token_only = first; token_only[28] = 8; token_only[29] = 255;
      assert(change.observe(token_only, true) == 0);
      const auto opposite = report(1u << (bphi - 1), (1u << (bpsi - 1)) - 1);
      assert(change.observe(opposite, true).value() >= 800);
      change.reset();
      assert(!change.observe(first, true));
      const auto wrap = report((1u << bphi) - 1, (1u << (bpsi - 1)) - 1);
      assert(change.observe(wrap, true).value() <= (codebook ? 10 : 40));
      assert(!change.observe(std::span(first).first(29), true));
      auto stripped = wrap; stripped.resize(stripped.size() - 4);
      assert(change.observe(stripped, false) == 0);
    }
  assert(!devourer::bf::is_vht_ndpa(feedback));
  for (const auto& [offset, value] : {
           std::pair{0, 0x88}, {1, 4}, {4, 3}, {10, 3}, {22, 1}, {24, 7}, {25, 1},
           {26, 0}, {26, 9 + 64}, {26, 10}, {27, 7}, {27, 12}, {27, 20},
           {28, 8}, {28, 5}}) {
    auto bad = feedback;
    bad[offset] = static_cast<uint8_t>(value);
    assert(!valid(bad));
  }
  for (size_t size = 0; size < feedback.size(); ++size)
    assert(!valid(std::span(feedback).first(size)));
  // Retry/power-management flags do not shift the Action body.
  feedback[1] = 0x18;
  assert(valid(feedback));
  feedback[1] = 0;
  const unsigned tones[3][3]{{52, 30, 16}, {108, 58, 30}, {234, 122, 62}};
  for (unsigned bw = 0; bw < 3; ++bw) {
    for (unsigned ng = 0; ng < 3; ++ng) {
      for (unsigned nc : {1u, 2u}) {
        for (unsigned cb : {0u, 1u}) {
          auto report = feedback;
          report[26] = static_cast<uint8_t>((bw << 6) | 8 | (nc - 1));
          report[27] = static_cast<uint8_t>(ng | (cb << 2) | 0x80);
          report.resize(29 + nc + (tones[bw][ng] * (cb ? 10 : 6) + 7) / 8 + 4);
          assert(devourer::bf::valid_vht_feedback(report, self, peer, bw, 1));
        }
      }
    }
  }
  feedback.push_back(0);
  assert(!valid(feedback));
  feedback.resize(95);
  assert(devourer::bf::valid_vht_feedback(feedback, self, peer, 0, 1, false));
  for (const bool steered : {false, true}) {
    std::array<uint8_t, jaguar3::TXDESC_SIZE_8822C> descriptor{};
    jaguar3::fill_data_tx_desc_8822c(descriptor.data(), 100, 0x13, 8, 0,
        false, true, 1, false, false, 0, 0, 0, false, steered);
    assert(LE_BITS_TO_4BYTE(descriptor.data() + 0x08, 24, 6) == 63);
    assert(LE_BITS_TO_4BYTE(descriptor.data() + 0x14, 8, 2) == (steered ? 0 : 1));
    assert(LE_BITS_TO_4BYTE(descriptor.data() + 0x28, 8, 8) == (steered ? 0x13 : 0));
    assert(LE_BITS_TO_4BYTE(descriptor.data() + 0x08, 0, 9) == 0);
    uint16_t checksum = 0;
    for (unsigned i = 0; i < descriptor.size(); i += 2)
      checksum ^= descriptor[i] | (descriptor[i + 1] << 8);
    assert(checksum == 0);
  }
  std::array<uint8_t, jaguar3::TXDESC_SIZE_8822C> sounding{};
  jaguar3::fill_data_tx_desc_8822c(sounding.data(), 19, 0x36, 9, 0,
      false, false, 0, false, true, 0, 0, 0, false);
  assert(LE_BITS_TO_4BYTE(sounding.data() + 0x08, 0, 9) == 0);
  assert(!devourer::bf::feedback_fresh(1000, 0, 500));
  assert(devourer::bf::feedback_fresh(1000, 500, 500));
  assert(!devourer::bf::feedback_fresh(1001, 500, 500));
  assert(!devourer::bf::feedback_fresh(999, 1000, 500));
  uint8_t phy[28]{};
  phy[0] = 1; phy[7] = 0xa0;
  rx_pkt_attrib attr{};
  attr.data_rate = 0x33;
  assert(jaguar3::parse_phy_sts_jgr3(phy, sizeof(phy), 0, attr) == PhyStsFill::Full);
  assert(attr.beamformed && attr.ldpc && !attr.stbc && attr.phy_fill == PhyStsFill::Full);
  phy[7] = 0;
  jaguar3::parse_phy_sts_jgr3(phy, sizeof(phy), 0, attr);
  assert(!attr.beamformed);
  phy[0] = 2;
  assert(jaguar3::parse_phy_sts_jgr3(phy, sizeof(phy), 0, attr) == PhyStsFill::Power);
  assert(attr.phy_fill == PhyStsFill::Power);
  jaguar3::parse_phy_sts_jgr3(nullptr, 0, 0, attr);
  assert(attr.phy_fill == PhyStsFill::None);
}
