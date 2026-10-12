// SPDX-License-Identifier: GPL-2.0-only
#include "kestrel/RtlKestrelDevice.h"
#include "kestrel/KestrelLe.h"

#include <array>
#include <cstdio>
#include <map>
#include <stdexcept>

static int failures = 0;
#define CHECK(c) do { if (!(c)) { std::fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #c); ++failures; } } while (0)

class FakeRegs final : public devourer::ITransport {
 public:
  bool usb = true, fail_ba = false;
  uint16_t fail_reg = 0xffff;
  std::map<uint16_t, uint32_t> regs;
  std::vector<bool> ba;
  bool is_usb() const override { return usb; }
  devourer::UsbLinkInfo usb_info() const override {
    return {.valid = usb, .bulk_out_eps = {1, 2, 3, 4, 5, 6, 7}};
  }
  uint8_t read8(uint16_t a) override { return static_cast<uint8_t>(regs[a]); }
  uint16_t read16(uint16_t a) override { return static_cast<uint16_t>(regs[a]); }
  uint32_t read32(uint16_t a) override { return regs[a]; }
  bool write8(uint16_t a, uint8_t v) override { return write32(a, v); }
  bool write16(uint16_t a, uint16_t v) override { return write32(a, v); }
  bool write32(uint16_t a, uint32_t v) override {
    if (a == fail_reg) return false;
    regs[a] = v;
    return true;
  }
  bool write_bytes(uint16_t, const uint8_t *, size_t) override { return true; }
  bool tx_async(uint8_t, uint8_t *, size_t, unsigned) override { return true; }
  int tx_sync(uint8_t, uint8_t *p, size_t n, int) override {
    namespace r = kestrel::reg;
    if (n >= 40 && ((kestrel::le32(p + 24) >> r::H2C_HDR_CLASS_SH) & 0x3f) == r::FWCMD_H2C_CL_BA_CAM) {
      if (fail_ba) return -1;
      ba.push_back((kestrel::le32(p + 32) & 1) != 0);
    }
    return static_cast<int>(n);
  }
  void rx_loop(int, int, const std::function<void(const uint8_t *, int)> &,
               const std::function<bool()> &) override {}
};

class CountingKestrel final : public RtlKestrelDevice {
 public:
  using RtlKestrelDevice::RtlKestrelDevice;
  unsigned frames = 0;
  bool send_packet(const uint8_t *, size_t) override { ++frames; return true; }
};

int main() {
  namespace r = kestrel::reg;
  auto logger = std::make_shared<Logger>();
  logger->set_level(Logger::Level::Silent);
  auto regs = std::make_shared<FakeRegs>();
  RtlAdapter adapter(regs, logger);
  RtlKestrelDevice radio(adapter, logger, kestrel::ChipVariant::C8852B);
  devourer::MacAddr local{{2, 0, 0, 0, 0, 1}}, a{{2, 0, 0, 0, 0, 2}}, b{{2, 0, 0, 0, 0, 3}};
  devourer::AmpduMode mode;
  mode.enabled = true;
  mode.no_ack = false;
  CHECK(radio.ConfigureAckPeer(local, a));
  CHECK(radio.SetAmpduMode(mode));
  CHECK(regs->ba == std::vector<bool>{true});
  CHECK(radio.ConfigureAckPeer(local, b));
  CHECK(radio.SetAmpduMode(mode));
  CHECK((regs->ba == std::vector<bool>{true, false, true}));
  radio.ClearAckPeer();
  CHECK((regs->read32(r::R_AX_PORT_CFG_P0) >> r::B_AX_NET_TYPE_P0_SH & r::B_AX_NET_TYPE_P0_MSK) == r::MAC_AX_NET_TYPE_NO_LINK);
  CHECK(radio.GetAmpduMode().no_ack);
  CHECK(radio.ConfigureAckPeer(local, a));
  CHECK(radio.SetAmpduMode(mode));
  CHECK((regs->ba == std::vector<bool>{true, false, true, false, true}));
  regs->fail_ba = true;
  CHECK(!radio.ConfigureAckPeer(local, b));
  bool threw = false;
  try { radio.ClearAckPeer(); } catch (const std::runtime_error &) { threw = true; }
  CHECK(threw);
  regs->fail_ba = false;
  radio.ClearAckPeer();

  regs->fail_reg = r::R_AX_PORT_CFG_P0;
  CHECK(!radio.ConfigureAckPeer(local, a));
  regs->fail_reg = 0xffff;
  CHECK(radio.ConfigureAckPeer(local, a));
  mode.max_time = 0x30;
  regs->fail_reg = r::R_AX_AMPDU_AGG_LIMIT;
  const auto before = radio.GetAmpduMode();
  CHECK(!radio.SetAmpduMode(mode));
  CHECK(radio.GetAmpduMode().max_time == before.max_time);
  regs->fail_reg = 0xffff;
  regs->fail_ba = true;
  CHECK(!radio.SetAmpduMode(mode));
  CHECK(radio.GetAmpduMode().max_time == before.max_time);
  regs->fail_ba = false;
  CHECK(radio.SetAmpduMode(mode));
  regs->fail_reg = r::R_AX_PORT_CFG_P0;
  threw = false;
  try { radio.ClearAckPeer(); } catch (const std::runtime_error &) { threw = true; }
  CHECK(threw);
  regs->fail_reg = 0xffff;
  radio.ClearAckPeer();

  // A PCIe batch must retain the per-MPDU virtual dispatch, even with a USB knob.
  regs->usb = false;
  devourer::DeviceConfig cfg;
  cfg.tx.usb_agg_max = 15;
  CountingKestrel pcie(RtlAdapter(regs, logger), logger, kestrel::ChipVariant::C8852C, cfg);
  std::array<uint8_t, 34> packet{};
  packet[2] = 8;
  packet[8] = 0x88;
  const TxPacketView views[]{{packet.data(), packet.size()}, {packet.data(), packet.size()}};
  CHECK(pcie.send_packets(views, 2) == 2);
  CHECK(pcie.frames == 2);
  return failures ? 1 : 0;
}
