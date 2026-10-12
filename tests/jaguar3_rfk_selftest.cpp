// SPDX-License-Identifier: GPL-2.0-only
#include "jaguar3/Halrf8822e.h"

#include <cstdio>
#include <map>

static int failures = 0;
#define CHECK(c) do { if (!(c)) { std::fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #c); ++failures; } } while (0)

class FakeRegs final : public devourer::ITransport {
 public:
  std::map<uint16_t, uint32_t> regs;
  bool tssi_dck = false, sweep = false, spur_checked = false;
  bool is_usb() const override { return true; }
  uint8_t read8(uint16_t a) override { return static_cast<uint8_t>(regs[a]); }
  uint16_t read16(uint16_t a) override { return static_cast<uint16_t>(regs[a]); }
  uint32_t read32(uint16_t a) override {
    if (a == 0x1700) return regs[a] | (1u << 29);
    if (a == 0x2d9c) return 0x55;
    if (a == 0x1bfc) return 0x8000;
    return regs[a];
  }
  bool write8(uint16_t a, uint8_t v) override { return write32(a, v); }
  bool write16(uint16_t a, uint16_t v) override { return write32(a, v); }
  bool write32(uint16_t a, uint32_t v) override {
    if (a == 0x522 && v == 0xff && !spur_checked) {
      spur_checked = true;
      if (sweep) {
        CHECK(regs[0x1b08] == 0);
        CHECK((regs[0x1b0c] & 0xc00) == 0xc00);
        CHECK(regs[0x1b14] == 0 && regs[0x1b18] == 1);
        CHECK((regs[0x1b20] & (1u << 29)) != 0);
        CHECK((regs[0x1d0c] & (1u << 16)) != 0);
        CHECK((regs[0x0] & (1u << 16)) != 0);
      }
    }
    if ((a == 0x1830 || a == 0x4130) && v == 0x700b8041) tssi_dck = true;
    regs[a] = v;
    return true;
  }
  bool write_bytes(uint16_t, const uint8_t *, size_t) override { return true; }
  bool tx_async(uint8_t, uint8_t *, size_t, unsigned) override { return true; }
  int tx_sync(uint8_t, uint8_t *, size_t n, int) override { return static_cast<int>(n); }
  void rx_loop(int, int, const std::function<void(const uint8_t *, int)> &,
               const std::function<bool()> &) override {}
};

int main() {
  auto logger = std::make_shared<Logger>();
  logger->set_level(Logger::Level::Silent);
  devourer::DeviceConfig cfg;
  cfg.tuning.skip_txgapk = true;
  for (const uint8_t channel : {54, 58, 149, 151, 161}) {
    auto regs = std::make_shared<FakeRegs>();
    regs->sweep = channel == 151 || channel == 161;
    regs->regs[0x1e28] = 8;
    regs->regs[0x1d58] = 0x12345678;
    jaguar3::Halrf8822e cal(RtlAdapter(regs, logger), logger, cfg);
    cal.phy_iq_calibrate(CHANNEL_WIDTH_20, channel);
    CHECK(regs->spur_checked);
    CHECK(!regs->tssi_dck);
    CHECK(regs->regs[0x1e28] == 8);
    CHECK(regs->regs[0x1d58] == 0x12345678);
    if (channel == 54 || channel == 58) {
      CHECK((regs->regs[0x1814] & (1u << 30)) == 0);
      CHECK((regs->regs[0x4114] & (1u << 30)) == 0);
    } else if (channel == 149) {
      CHECK((regs->regs[0x1814] & (1u << 30)) != 0);
      CHECK((regs->regs[0x4114] & (1u << 30)) != 0);
    }
    if (channel == 161) {
      CHECK((regs->regs[0x1944] & 0x300) == 0x300);
      CHECK((regs->regs[0x4044] & 0x300) == 0x300);
      CHECK((regs->regs[0x88c] & 0x30000) == 0x20000);
    }
  }
  return failures ? 1 : 0;
}
