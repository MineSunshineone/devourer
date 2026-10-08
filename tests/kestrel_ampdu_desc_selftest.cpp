#include "kestrel/TxDescKestrel.h"

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
  }
  return 0;
}
