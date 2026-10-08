// SPDX-License-Identifier: GPL-2.0-only
#include "VhtSounding.h"
#include "jaguar3/FrameParserJaguar3.h"
#include "kestrel/TxDescKestrel.h"
#include <cassert>
#include <vector>

int main() {
  devourer::TxMode ht;
  ht.mode = devourer::TxMode::Mode::HT;
  ht.ht_mcs = 7;
  ht.bw_mhz = 40;
  ht.sgi = ht.ldpc = ht.stbc = true;
  const auto vht = devourer::bf::vht_su_mode(ht);
  assert(vht.mode == devourer::TxMode::Mode::VHT && vht.vht_mcs == 7 && vht.vht_nss == 1);
  assert(vht.bw_mhz == 40 && vht.sgi && vht.ldpc && !vht.stbc);
  const uint8_t self[6]{2, 1, 2, 3, 4, 5}, peer[6]{2, 6, 7, 8, 9, 10};
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
        false, true, 1, false, false, 0, 0, 0, steered);
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
  const auto exchange = kestrel::build_vht_sounding(ndpa.data(), 0, 7);
  assert(exchange.size() == 67); // One 48B WD + 19B NDPA, no zero-length host packet.
  const auto word = [&](unsigned offset) {
    return unsigned(exchange[offset]) | (unsigned(exchange[offset + 1]) << 8) |
        (unsigned(exchange[offset + 2]) << 16) | (unsigned(exchange[offset + 3]) << 24);
  };
  assert(word(4) == 0); // No USB aggregation.
  assert((word(8) & 0x3fff) == 19);
  assert(word(24 + 12) == 4); // VHT NDPA selector; no firmware-exchange chain.
  assert(((word(24) >> 16) & 0x1ff) == 0x110); // 2SS VHT MCS0.
  assert((word(24 + 20) >> 16) == 256);
  assert(std::memcmp(exchange.data() + 48, ndpa.data(), ndpa.size()) == 0);
}
