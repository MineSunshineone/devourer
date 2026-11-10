// SPDX-License-Identifier: GPL-2.0-only
// BXI modification 2026-11-10: verify same-channel surveys avoid RF/BB resets.
#include "kestrel/RtlKestrelDevice.h"

#include <cstdio>
#include <map>

static int failures = 0;
#define CHECK(c) do { if (!(c)) { std::fprintf(stderr, "FAIL %d: %s\n", __LINE__, #c); ++failures; } } while (0)

class Registers final : public devourer::ITransport {
 public:
  std::map<uint32_t, uint32_t> values;
  unsigned writes = 0, reads = 0, sends = 0;
  unsigned scheduler_commands = 0;
  uint64_t failed_writes = 0;
  bool fail_writes = false;
  uint64_t ctrl_write_failures() const override { return failed_writes; }
  bool is_usb() const override { return true; }
  devourer::UsbLinkInfo usb_info() const override {
    return {.valid = true, .bulk_out_eps = {1, 2, 3, 4, 5, 6, 7}};
  }
  uint8_t read8(uint16_t a) override { return static_cast<uint8_t>(read32(a)); }
  uint16_t read16(uint16_t a) override { return static_cast<uint16_t>(read32(a)); }
  uint32_t read32(uint16_t a) override { return read32_wide(a); }
  uint32_t read32_wide(uint32_t a) override { ++reads; return values[a]; }
  bool write8(uint16_t a, uint8_t v) override { return write32(a, v); }
  bool write16(uint16_t a, uint16_t v) override { return write32(a, v); }
  bool write32(uint16_t a, uint32_t v) override { return write32_wide(a, v); }
  bool write32_wide(uint32_t a, uint32_t v) override {
    ++writes;
    if (fail_writes) { ++failed_writes; return false; }
    values[a] = v;
    if (a == 0x8160 && (v & 1)) {
      CHECK((values[0x8140] & 0xffff) == 0x305);
      CHECK(values[0x8144] == 0xffff);
      values[0xc348] = values[0x8140] >> 16;
      values[0x8150] = 0x104;
      values[0x8164] = 1;
      values[0x8160] = v & ~1u;
      ++scheduler_commands;
    }
    return true;
  }
  bool write_bytes(uint16_t, const uint8_t *, size_t) override {
    ++writes;
    if (fail_writes) ++failed_writes;
    return !fail_writes;
  }
  bool tx_async(uint8_t, uint8_t *, size_t, unsigned) override { ++sends; return true; }
  int tx_sync(uint8_t, uint8_t *, size_t n, int) override { ++sends; return static_cast<int>(n); }
  void rx_loop(int, int, const std::function<void(const uint8_t *, int)> &,
               const std::function<bool()> &) override {}
  void clear_counts() { reads = writes = sends = 0; }
};

void check_retune(kestrel::ChipVariant variant) {
  auto logger = std::make_shared<Logger>();
  logger->set_level(Logger::Level::Silent);
  auto regs = std::make_shared<Registers>();
  RtlAdapter adapter(regs, logger);
  RtlKestrelDevice radio(adapter, logger, variant);
  radio.SetMonitorChannel({.Channel = 9, .ChannelOffset = 0,
                           .ChannelWidth = CHANNEL_WIDTH_20});
  regs->values[0xc348] = 0x9a5b;
  regs->clear_counts();
  radio.FastRetune(9);
  CHECK(regs->reads == 0 && regs->writes == 0 && regs->sends == 0);
  CHECK(radio.GetSelectedChannel().Channel == 9);

  radio.FastRetune(10);
  CHECK(regs->writes > 0 || regs->sends > 0);
  CHECK(radio.GetSelectedChannel().Channel == 10);
  CHECK(regs->values[0xc348] == 0x9a5b);
  CHECK(regs->scheduler_commands == (variant == kestrel::ChipVariant::C8852B ? 2u : 0u));
  regs->clear_counts();
  radio.FastRetune(10);
  CHECK(regs->reads == 0 && regs->writes == 0 && regs->sends == 0);

  // A real retune must still surface transport failures and retain old state.
  regs->fail_writes = true;
  bool failed = false;
  try { radio.FastRetune(11); }
  catch (const std::runtime_error &) { failed = true; }
  CHECK(failed);
  CHECK(radio.GetSelectedChannel().Channel == 10);
}

int main() {
  check_retune(kestrel::ChipVariant::C8852B);
  check_retune(kestrel::ChipVariant::C8852C);
  return failures ? 1 : 0;
}
