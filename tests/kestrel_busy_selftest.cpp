// SPDX-License-Identifier: GPL-2.0-only
#include "kestrel/BusyWindowKestrel.h"

#include <cstdio>
#include <map>
#include <vector>

static int failures = 0;
#define CHECK(c) do { if (!(c)) { std::fprintf(stderr, "FAIL %d: %s\n", __LINE__, #c); ++failures; } } while (0)

struct Registers {
  std::map<uint32_t, uint32_t> data{{0x10c00, 0x308}, {0x10c04, 0x2300000}};
  std::vector<uint32_t> writes;
  unsigned reads = 0, fail_write = 0;
  uint32_t bad_read = 0;
  bool stale_ready = false, fail_restore = false;
  uint32_t read(uint32_t a) {
    ++reads;
    if (a == bad_read) { bad_read = 0; return UINT32_MAX; }
    return data[a];
  }
  bool write(uint32_t a, uint32_t v) {
    CHECK(a == 0x10c00 || a == 0x10c04);
    writes.push_back(a);
    if (writes.size() == fail_write || (fail_restore && v == 0x308))
      return false;
    data[a] = v;
    if (!stale_ready) {
      if (a == 0x10c00 && (v & 0x1000)) data[0x11ac4] = 0;
      if (a == 0x10c04 && (v & 0x40000)) data[0x11ac8] = 0;
    }
    return true;
  }
  kestrel::BusyWindow::Read rd = [this](uint32_t a) { return read(a); };
  kestrel::BusyWindow::Write wr = [this](uint32_t a, uint32_t v) { return write(a, v); };
  void done(uint32_t cca = 2500, uint32_t edcca = 20000) {
    data[0x11ac4] = 0x10000 | cca;
    data[0x11ac8] = 0x10000 | edcca;
  }
  void restored() {
    CHECK(data[0x10c00] == 0x308);
    CHECK(data[0x10c04] == 0x2300000);
  }
};

int main() {
  using devourer::BusySpoil;
  {
    Registers r; kestrel::BusyWindow w;
    CHECK(w.arm(0, 0, r.rd, r.wr) == 0);
    CHECK(!w.read(0, r.rd, r.wr).valid);
    CHECK(r.reads == 0 && r.writes.empty());
    CHECK(w.arm(100000, 10, r.rd, r.wr) == 100000);
    // Golden 8852B fields: 25,000 ticks, primary20, 4 us, CCX + both enables.
    CHECK(r.data[0x10c00] == (25000u << 16 | 0x1309u));
    CHECK(r.data[0x10c04] == (0x2300000u | 0x40000u | 25000u));
    CHECK(w.read(10, r.rd, r.wr).spoil == BusySpoil::NotElapsed);
    r.data[0x11ac4] = 0x10000 | 2500;
    CHECK(w.read(10, r.rd, r.wr).spoil == BusySpoil::NotElapsed);
    r.done();
    auto b = w.read(12, r.rd, r.wr);
    CHECK(b.valid && b.valid_busy && b.busy_pct == 10);
    CHECK(b.valid_edcca && b.edcca_pct == 80 && !b.valid_energy);
    CHECK(b.window_us == 100000 && b.source == devourer::BusySource::Clm);
    CHECK(b.own_tx_in_window && b.own_tx_frames == 2);
    r.restored();
    const auto reads = r.reads;
    CHECK(!w.read(12, r.rd, r.wr).valid && r.reads == reads);
  }
  for (auto reason : {BusySpoil::Retuned, BusySpoil::Interrupted, BusySpoil::None}) {
    Registers r; kestrel::BusyWindow w;
    CHECK(w.arm(1, 0, r.rd, r.wr) == 20000);
    CHECK(w.cancel(reason, r.rd, r.wr));
    r.restored();
    auto b = w.read(0, r.rd, r.wr);
    CHECK(!b.valid && b.spoil == reason);
    CHECK(w.read(0, r.rd, r.wr).spoil == BusySpoil::None);
    CHECK(w.arm(UINT32_MAX, 0, r.rd, r.wr) == 240000);
    r.done(60000, 0);
    b = w.read(0, r.rd, r.wr);
    CHECK(b.valid && b.busy_pct == 100 && b.edcca_pct == 0 && !b.own_tx_in_window);
  }
  // A pre-existing measurement owner is never disabled or reconfigured.
  for (auto a : {0x10c00u, 0x10c04u}) {
    Registers r; kestrel::BusyWindow w;
    r.data[a] |= a == 0x10c00 ? 1 : 0x40000;
    CHECK(w.arm(100000, 0, r.rd, r.wr) == 0 && r.writes.empty());
  }
  {
    Registers r; kestrel::BusyWindow w;
    r.data[0x10c00] |= 0x80; // Unvalidated CCX_TXON_OPT is not silently changed.
    CHECK(w.arm(100000, 0, r.rd, r.wr) == 0 && r.writes.empty());
  }
  for (auto a : {0x10c00u, 0x10c04u}) {
    Registers r; kestrel::BusyWindow w;
    r.bad_read = a;
    CHECK(w.arm(100000, 0, r.rd, r.wr) == 0 && r.writes.empty());
  }
  for (unsigned failed_write = 1; failed_write <= 4; ++failed_write) {
    Registers r; kestrel::BusyWindow w;
    r.fail_write = failed_write;
    CHECK(w.arm(100000, 0, r.rd, r.wr) == 0);
    r.restored();
    CHECK(w.read(0, r.rd, r.wr).spoil == BusySpoil::Interrupted);
  }
  {
    Registers r; kestrel::BusyWindow w;
    r.stale_ready = true; r.done();
    CHECK(w.arm(100000, 0, r.rd, r.wr) == 0);
    r.restored();
  }
  {
    Registers r; kestrel::BusyWindow w;
    CHECK(w.arm(100000, 0, r.rd, r.wr) == 100000);
    r.done(0, 0);
    auto b = w.read(0, r.rd, r.wr);
    CHECK(b.valid && b.valid_edcca && b.busy_pct == 0 && b.edcca_pct == 0);
    CHECK(w.arm(100000, 0, r.rd, r.wr) == 100000);
    r.data[0x10c00] |= 0x80; // Keep someone else's changed TXON bit on cancellation.
    CHECK(w.read(0, r.rd, r.wr).spoil == BusySpoil::Interrupted);
    CHECK(r.data[0x10c00] == 0x388);
    CHECK(r.data[0x10c04] == 0x2300000);
  }
  for (auto a : {0x10c00u, 0x10c04u, 0x11ac4u, 0x11ac8u}) {
    Registers r; kestrel::BusyWindow w;
    CHECK(w.arm(100000, 0, r.rd, r.wr) == 100000);
    r.done(); r.bad_read = a;
    auto b = w.read(0, r.rd, r.wr);
    CHECK(!b.valid && b.spoil == BusySpoil::Interrupted);
    r.restored();
  }
  {
    Registers r; kestrel::BusyWindow w;
    CHECK(w.arm(100000, 10, r.rd, r.wr) == 100000);
    CHECK(w.arm(0, 10, r.rd, r.wr) == 0); // Zero must leave the first arm intact.
    r.done(25001, 0);
    CHECK(w.read(10, r.rd, r.wr).spoil == BusySpoil::Interrupted);
    r.restored();
    CHECK(w.arm(100000, 10, r.rd, r.wr) == 100000);
    r.done();
    auto b = w.read(9, r.rd, r.wr); // Counter reset is unknown own TX, not quiet.
    CHECK(b.valid && b.own_tx_in_window && b.own_tx_frames == UINT32_MAX);
    CHECK(w.arm(100000, 0, r.rd, r.wr) == 100000);
    r.data[0x10c00] ^= 0x10000; // A changed period must not produce a plausible %.
    CHECK(w.read(0, r.rd, r.wr).spoil == BusySpoil::Interrupted);
    r.restored();
  }
  {
    Registers r; kestrel::BusyWindow w;
    CHECK(w.arm(100000, 0, r.rd, r.wr) == 100000);
    r.done(); r.fail_restore = true;
    auto b = w.read(0, r.rd, r.wr);
    CHECK(!b.valid && b.spoil == BusySpoil::Interrupted);
    const auto writes = r.writes.size();
    CHECK(w.arm(100000, 0, r.rd, r.wr) == 0 && r.writes.size() == writes);
    r.fail_restore = false;
    CHECK(w.cancel(BusySpoil::None, r.rd, r.wr));
    r.restored();
  }
  std::printf("Kestrel CCX busy window: %d failures\n", failures);
  return failures ? 1 : 0;
}
