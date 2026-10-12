#include "kestrel/TxDescKestrel.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdio>
#include <vector>

static uint32_t le32(const std::vector<uint8_t> &bytes, size_t offset) {
  return static_cast<uint32_t>(bytes[offset]) |
         (static_cast<uint32_t>(bytes[offset + 1]) << 8) |
         (static_cast<uint32_t>(bytes[offset + 2]) << 16) |
         (static_cast<uint32_t>(bytes[offset + 3]) << 24);
}

int main() {
  std::array<uint8_t, 26> qos{};
  qos[0] = 0x88;
  qos[4] = 0xff; // Broadcast RA, no BlockAck responder.
  kestrel::TxRate rate{.rate = 0x87}; // HT MCS7.
  for (const uint32_t body : {kestrel::WD_BODY_LEN,
                              kestrel::WD_BODY_LEN_V1}) {
    std::vector<uint8_t> plain, aggregated;
    kestrel::build_data_txdesc_into(plain, qos.data(), qos.size(), rate, 0,
                                    1, body, 1);
    kestrel::build_data_txdesc_into(aggregated, qos.data(), qos.size(), rate,
                                    0, 1, body, 1, 16, 7);
    if (le32(plain, 12) != 1 ||
        le32(aggregated, 12) != (1u | kestrel::txd::AGG_EN) ||
        le32(aggregated, body + 4) != le32(plain, body + 4) + 15u ||
        le32(plain, body + 8) != 0 ||
        le32(aggregated, body + 8) != (7u << 18)) {
      std::fprintf(stderr, "Kestrel A-MPDU descriptor mismatch (body=%u)\n",
                   body);
      return 1;
    }
    constexpr std::array<uint8_t, 8> queues{0, 1, 1, 0, 2, 2, 3, 3};
    constexpr std::array<uint8_t, 8> indicators{0, 0, 1, 1, 0, 1, 0, 1};
    for (uint8_t tid = 0; tid < 8; ++tid) {
      qos[24] = tid;
      kestrel::build_data_txdesc_into(plain, qos.data(), qos.size(), rate,
                                      1, 1, body, 13);
      const auto d2 = le32(plain, 8);
      const auto dma = (le32(plain, 0) >> 16) & 0xf;
      if (((d2 >> 17) & 0x3f) != queues[tid] ||
          ((d2 >> 23) & 1) != indicators[tid] ||
          ((d2 >> 24) & 0x7f) != 1 ||
          dma != (body == kestrel::WD_BODY_LEN ? queues[tid] : 0)) {
        std::fprintf(stderr, "Kestrel QoS queue mismatch (body=%u tid=%u)\n",
                     body, tid);
        return 1;
      }
    }
    qos[24] = 0;
  }
  const std::array<uint8_t, 6> peer{2, 0x42, 0x58, 0x49, 0x24, 0x22};
  std::copy(peer.begin(), peer.end(), qos.begin() + 4);
  if (kestrel::data_macid(qos.data(), qos.size(), peer.data()) != 1 ||
      kestrel::data_macid(qos.data(), qos.size(), nullptr) != 0 ||
      kestrel::data_macid(qos.data(), 9, peer.data()) != 0) return 1;
  qos[4] = 0xff;
  if (kestrel::data_macid(qos.data(), qos.size(), peer.data()) != 0) return 1;
  qos[24] = 6;
  if (kestrel::data_qsel(qos.data(), 25) != 0) return 1;
  qos[0] = 0x08;
  if (kestrel::data_qsel(qos.data(), qos.size()) != 0) return 1;
  return 0;
}
