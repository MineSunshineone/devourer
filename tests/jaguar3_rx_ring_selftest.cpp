// SPDX-License-Identifier: GPL-2.0-only
#include "jaguar3/RtlJaguar3Device.h"

#include <cstdio>

class RxSpy final : public devourer::ITransport {
 public:
  int bytes = 0, urbs = 0;
  bool is_usb() const override { return true; }
  uint8_t read8(uint16_t) override { return 0; }
  uint16_t read16(uint16_t) override { return 0; }
  uint32_t read32(uint16_t) override { return 0; }
  bool write8(uint16_t, uint8_t) override { return true; }
  bool write16(uint16_t, uint16_t) override { return true; }
  bool write32(uint16_t, uint32_t) override { return true; }
  bool write_bytes(uint16_t, const uint8_t *, size_t) override { return true; }
  bool tx_async(uint8_t, uint8_t *, size_t, unsigned) override { return true; }
  int tx_sync(uint8_t, uint8_t *, size_t n, int) override { return static_cast<int>(n); }
  void rx_loop(int size, int count,
               const std::function<void(const uint8_t *, int)> &,
               const std::function<bool()> &) override {
    bytes = size;
    urbs = count;
  }
};

int main() {
  auto logger = std::make_shared<Logger>();
  logger->set_level(Logger::Level::Silent);
  for (const auto requested : {std::optional<int>{}, std::optional<int>{16},
                               std::optional<int>{0}, std::optional<int>{100}}) {
    auto spy = std::make_shared<RxSpy>();
    devourer::DeviceConfig cfg;
    cfg.rx.urbs = requested;
    RtlJaguar3Device radio(RtlAdapter(spy, logger), logger,
                          jaguar3::ChipVariant::C8822E, cfg);
    radio.StartRxLoop({});
    const int expected = !requested ? 8 : *requested == 16 ? 16 :
                         *requested == 0 ? 1 : 64;
    if (spy->urbs != expected || spy->bytes != 16384) {
      std::fprintf(stderr, "RX transport got %d URBs / %d bytes; expected %d / 16384\n",
                   spy->urbs, spy->bytes, expected);
      return 1;
    }
  }
  std::puts("PASS: Jaguar3 RX ring default, requested depth and bounds");
}
