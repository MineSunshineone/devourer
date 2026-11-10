// SPDX-License-Identifier: GPL-2.0-only
#include "kestrel/KestrelFw.h"
#include "kestrel/MacRegAx.h"

#include <cstdio>
#include <map>
#include <stdexcept>

namespace r = kestrel::reg;
static int failures = 0;
#define CHECK(c) do { if (!(c)) { std::fprintf(stderr, "FAIL %d: %s\n", __LINE__, #c); ++failures; } } while (0)

class Mailbox final : public devourer::ITransport {
 public:
  enum class Mode { good, firmware_error, wrong_function, short_reply, no_reply,
                    ack_failure, send_failure, read_failure };
  Mode mode = Mode::good;
  std::map<uint16_t, uint32_t> regs;
  unsigned sends = 0, acknowledgements = 0, reply_reads = 0;
  std::vector<uint16_t> scheduler_requests;
  bool fail_pause = false, fail_restore = false, mismatch_pause = false;
  bool is_usb() const override { return true; }
  devourer::UsbLinkInfo usb_info() const override {
    return {.valid = true, .bulk_out_eps = {1, 2, 3}};
  }
  uint8_t read8(uint16_t a) override { return static_cast<uint8_t>(read32(a)); }
  uint16_t read16(uint16_t a) override { return static_cast<uint16_t>(read32(a)); }
  uint32_t read32(uint16_t a) override {
    if (mode == Mode::read_failure && sends) throw std::runtime_error("read failed");
    if (a >= r::R_AX_C2HREG_DATA0 && a <= r::R_AX_C2HREG_DATA3) ++reply_reads;
    return regs[a];
  }
  bool write8(uint16_t a, uint8_t v) override { return write32(a, v); }
  bool write16(uint16_t a, uint16_t v) override { return write32(a, v); }
  bool write32(uint16_t a, uint32_t v) override {
    if (a == 0x8160 && (v & 1)) {
      CHECK((regs[0x8140] & 0xffff) == 0x305);
      CHECK(regs[0x8144] == 0xffff && regs[0x8148] == 0 && regs[0x814c] == 0);
      const auto enabled = static_cast<uint16_t>(regs[0x8140] >> 16);
      scheduler_requests.push_back(enabled);
      if ((enabled == 0 && fail_pause) || (enabled != 0 && fail_restore)) return false;
      regs[r::R_AX_CTN_TXEN] = enabled == 0 && mismatch_pause ? 1 : enabled;
      regs[r::R_AX_C2HREG_DATA0] = 0x104;
      regs[r::R_AX_C2HREG_CTRL] = mode == Mode::no_reply ? 0x80 : 0x81;
      regs[a] = v & ~1u;
      reply_reads = 0;
      return true;
    }
    if (a == r::R_AX_C2HREG_CTRL) {
      CHECK(reply_reads >= 4);
      if (mode == Mode::ack_failure) return false;
      ++acknowledgements;
    }
    regs[a] = v;
    return true;
  }
  bool write_bytes(uint16_t, const uint8_t *, size_t) override { return true; }
  bool tx_async(uint8_t, uint8_t *, size_t, unsigned) override { return false; }
  int tx_sync(uint8_t, uint8_t *, size_t n, int) override {
    CHECK(!(regs[r::R_AX_C2HREG_CTRL] & 1));
    ++sends;
    reply_reads = 0;
    if (mode == Mode::send_failure) return -1;
    regs[r::R_AX_C2HREG_DATA0] = mode == Mode::short_reply ? 0x109 :
        mode == Mode::wrong_function ? 0x404 :
        mode == Mode::firmware_error ? 0x10409 : 0x409;
    // Keep unrelated control bits when acknowledging the reply.
    regs[r::R_AX_C2HREG_CTRL] = mode == Mode::no_reply ? 0x80 : 0x81;
    return static_cast<int>(n);
  }
  void rx_loop(int, int, const std::function<void(const uint8_t *, int)> &,
               const std::function<bool()> &) override {}
};

int main() {
  const kestrel::KestrelFw::OfldWrite cmd{0, 0, 0x2344, 0, 1u << 31};
  auto logger = std::make_shared<Logger>();
  logger->set_level(Logger::Level::Silent);
  for (const auto mode : {Mailbox::Mode::good, Mailbox::Mode::firmware_error,
       Mailbox::Mode::wrong_function, Mailbox::Mode::short_reply, Mailbox::Mode::no_reply,
       Mailbox::Mode::ack_failure, Mailbox::Mode::send_failure, Mailbox::Mode::read_failure}) {
    auto io = std::make_shared<Mailbox>();
    io->mode = mode;
    kestrel::KestrelFw fw(RtlAdapter(io, logger), logger, kestrel::ChipVariant::C8852B);
    bool accepted = false, threw = false;
    try { accepted = fw.reg_write_ofld(&cmd, 1); }
    catch (const std::runtime_error &) { threw = true; }
    CHECK(accepted == (mode == Mailbox::Mode::good));
    CHECK(threw == (mode != Mailbox::Mode::good));
    if (mode == Mailbox::Mode::good) {
      CHECK(io->acknowledgements == 1 && io->regs[r::R_AX_C2HREG_CTRL] == 0x80);
      CHECK(fw.reg_write_ofld(&cmd, 1));
      CHECK(io->acknowledgements == 2);
    }
  }
  auto io = std::make_shared<Mailbox>();
  io->regs[r::R_AX_C2HREG_CTRL] = 1;
  kestrel::KestrelFw fw(RtlAdapter(io, logger), logger, kestrel::ChipVariant::C8852B);
  bool rejected = false;
  try { rejected = !fw.reg_write_ofld(&cmd, 1); } catch (const std::runtime_error &) { rejected = true; }
  CHECK(rejected && io->sends == 0);

  // A retune may contain an IO offload, so the scheduler and IO transactions
  // share the mailbox without holding its mutex across the operation itself.
  for (int scenario = 0; scenario < 7; ++scenario) {
    auto scheduler = std::make_shared<Mailbox>();
    constexpr uint16_t original = 0x9a5b;
    scheduler->regs[r::R_AX_CTN_TXEN] = original;
    scheduler->fail_pause = scenario == 2;
    scheduler->fail_restore = scenario == 3;
    scheduler->mismatch_pause = scenario == 4;
    if (scenario == 5) scheduler->regs[0xc6ec] = 0x80;
    if (scenario == 6) scheduler->mode = Mailbox::Mode::no_reply;
    kestrel::KestrelFw paused(RtlAdapter(scheduler, logger), logger,
                             kestrel::ChipVariant::C8852B);
    unsigned operations = 0;
    bool threw = false;
    try {
      paused.with_tx_paused([&] {
        ++operations;
        CHECK(scheduler->regs[r::R_AX_CTN_TXEN] == 0);
        CHECK(paused.reg_write_ofld(&cmd, 1));
        if (scenario == 1) throw std::runtime_error("retune failed");
      });
    } catch (const std::runtime_error &) { threw = true; }
    CHECK(threw == (scenario != 0));
    CHECK(operations == (scenario == 0 || scenario == 1 || scenario == 3 ? 1u : 0u));
    CHECK(scheduler->scheduler_requests == std::vector<uint16_t>({0, original}));
    if (scenario != 3) CHECK(scheduler->regs[r::R_AX_CTN_TXEN] == original);
    else CHECK(scheduler->regs[r::R_AX_CTN_TXEN] == 0);
  }
  return failures ? 1 : 0;
}
