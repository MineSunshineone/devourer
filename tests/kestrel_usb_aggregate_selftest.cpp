// SPDX-License-Identifier: GPL-2.0-only
// BXI modification 2026-11-10: enforce the vendor USB bulk-end WD restriction.
#include "kestrel/TxDescKestrel.h"

#include <algorithm>
#include <cstdio>

static int failures = 0;
#define CHECK(c) do { if (!(c)) { std::fprintf(stderr, "FAIL %d: %s\n", __LINE__, #c); ++failures; } } while (0)

std::vector<uint8_t> descriptor(uint32_t body, size_t payload_size) {
  std::vector<uint8_t> payload(payload_size, 0xa5);
  payload[0] = 0x08; // Non-QoS data frame; unchanged payload is checked below.
  std::vector<uint8_t> result;
  kestrel::build_data_txdesc_into(result, payload.data(), payload.size(),
                                 {}, 0, 0, body);
  return result;
}

void boundary(uint32_t body, size_t packet_size) {
  const size_t wd = body + kestrel::WD_INFO_LEN;
  // A padded first frame leaves the next descriptor ending exactly at 512/1024.
  const auto first = descriptor(body, packet_size - 2 * wd - 3);
  const auto next = descriptor(body, 100);
  std::vector<uint8_t> aggregate;
  CHECK(kestrel::append_usb_tx_frame(aggregate, first, wd, packet_size));
  CHECK(aggregate.size() == packet_size - wd);
  CHECK(std::equal(first.begin(), first.end(), aggregate.begin()));
  CHECK(std::all_of(aggregate.begin() + first.size(), aggregate.end(),
                    [](uint8_t b) { return b == 0; }));
  const auto saved = aggregate;
  CHECK(!kestrel::append_usb_tx_frame(aggregate, next, wd, packet_size));
  CHECK(aggregate == saved); // Rejection must not consume/change the frame.
  aggregate.clear();
  CHECK(kestrel::append_usb_tx_frame(aggregate, next, wd, packet_size));
  CHECK(std::equal(next.begin(), next.end(), aggregate.begin()));

  // Both adjacent placements remain legal; a normal batch is not disabled.
  for (int delta : {-8, 8}) {
    aggregate.clear();
    const auto neighbor = descriptor(body, packet_size - 2 * wd + delta);
    CHECK(kestrel::append_usb_tx_frame(aggregate, neighbor, wd, packet_size));
    const size_t start = aggregate.size();
    CHECK(kestrel::append_usb_tx_frame(aggregate, next, wd, packet_size));
    CHECK(std::equal(next.begin(), next.end(), aggregate.begin() + start));
  }
}

int main() {
  for (uint32_t body : {kestrel::WD_BODY_LEN, kestrel::WD_BODY_LEN_V1})
    for (size_t packet_size : {512u, 1024u}) boundary(body, packet_size);
  std::printf("Kestrel USB aggregate: %d failures\n", failures);
  return failures ? 1 : 0;
}
