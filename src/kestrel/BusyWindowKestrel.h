// SPDX-License-Identifier: GPL-2.0-only
#pragma once

#include <algorithm>
#include <cstdint>
#include <functional>
#include "RxSense.h"

namespace kestrel {

// 8852B BB_CLIENT CCX registers, through the BB window (wIndex=1 on USB).
// Own only the measurement controls; never change CCA gates or thresholds.
class BusyWindow {
public:
  using Read = std::function<uint32_t(uint32_t)>;
  using Write = std::function<bool(uint32_t, uint32_t)>;

  uint32_t arm(uint32_t window_us, uint64_t tx, const Read &rd,
               const Write &wr) {
    if (!window_us || blocked_)
      return 0;
    if (!cancel(devourer::BusySpoil::None, rd, wr))
      return 0;
    old_[0] = rd(kControl[0]);
    old_[1] = rd(kControl[1]);
    if (bad(old_[0]) || bad(old_[1]) || (old_[0] & 0x1081u) ||
        (old_[1] & 0x40000u))
      return 0; // Refuse another CCX owner or the unvalidated TXON-counting mode.
    period_ = std::clamp(window_us / 4u, 5000u, 60000u);
    want_[0] = (old_[0] & ~kMask[0]) | (period_ << 16) | 1u;
    want_[1] = (old_[1] & ~kMask[1]) | period_;
    owned_ = true;
    tx_ = tx;
    bool ok = put(0, rd, wr) && put(1, rd, wr);
    want_[0] |= 0x1000u;
    want_[1] |= 0x40000u;
    ok = ok && put(0, rd, wr) && put(1, rd, wr);
    if (ok) {
      const auto cca = rd(kResult[0]), edcca = rd(kResult[1]);
      ok = !bad(cca) && !bad(edcca) && !(cca & kReady) && !(edcca & kReady);
    }
    if (!ok) {
      cancel(devourer::BusySpoil::Interrupted, rd, wr);
      return 0;
    }
    return period_ * 4u;
  }

  devourer::ChannelBusy read(uint64_t tx, const Read &rd, const Write &wr) {
    devourer::ChannelBusy b;
    if (!owned_ || blocked_) {
      b.spoil = blocked_ ? devourer::BusySpoil::Interrupted : spoil_;
      spoil_ = devourer::BusySpoil::None;
      return b;
    }
    const auto c0 = rd(kControl[0]), c1 = rd(kControl[1]);
    const auto cca = rd(kResult[0]), edcca = rd(kResult[1]);
    if (bad(c0) || bad(c1) || bad(cca) || bad(edcca) ||
        c0 != want_[0] || c1 != want_[1] ||
        (cca & 65535u) > period_ || (edcca & 65535u) > period_) {
      cancel(devourer::BusySpoil::Interrupted, rd, wr);
      b.spoil = devourer::BusySpoil::Interrupted;
      spoil_ = devourer::BusySpoil::None;
      return b;
    }
    if (!(cca & kReady) || !(edcca & kReady)) {
      b.spoil = devourer::BusySpoil::NotElapsed;
      return b;
    }
    if (!cancel(devourer::BusySpoil::None, rd, wr)) {
      b.spoil = devourer::BusySpoil::Interrupted;
      return b;
    }
    b.valid = b.valid_busy = b.valid_edcca = true;
    b.source = devourer::BusySource::Clm;
    b.window_us = period_ * 4u;
    b.busy_pct = percent(cca);
    b.edcca_pct = percent(edcca);
    const auto sent = tx >= tx_ ? tx - tx_ : UINT64_MAX;
    b.own_tx_frames = static_cast<uint32_t>(std::min<uint64_t>(sent, UINT32_MAX));
    b.own_tx_in_window = sent != 0;
    return b;
  }

  // Called before NHM, retuning or teardown, under the same CCX mutex.
  // Both controls get a restore attempt even if one USB operation fails.
  bool cancel(devourer::BusySpoil reason, const Read &rd, const Write &wr) {
    if (!owned_) {
      if (reason == devourer::BusySpoil::None)
        spoil_ = reason;
      return !blocked_;
    }
    bool ok = true;
    for (int i : {1, 0}) {
      const auto now = rd(kControl[i]);
      if (bad(now) || !wr(kControl[i], (now & ~kMask[i]) | (old_[i] & kMask[i]))) {
        ok = false;
        continue;
      }
      const auto restored = rd(kControl[i]);
      if (bad(restored) || (restored & kMask[i]) != (old_[i] & kMask[i]))
        ok = false;
    }
    blocked_ = !ok;
    owned_ = !ok;
    spoil_ = ok ? reason : devourer::BusySpoil::Interrupted;
    return ok;
  }

private:
  static constexpr uint32_t kControl[2] = {0x10c00, 0x10c04};
  static constexpr uint32_t kResult[2] = {0x11ac4, 0x11ac8};
  static constexpr uint32_t kMask[2] = {0xfffffc77u, 0x0007ffffu};
  static constexpr uint32_t kReady = 0x10000;
  static bool bad(uint32_t v) { return v == UINT32_MAX || v == 0xdeadbeefu; }
  bool put(int i, const Read &rd, const Write &wr) {
    if (!wr(kControl[i], want_[i]))
      return false;
    const auto v = rd(kControl[i]);
    return !bad(v) && v == want_[i];
  }
  uint8_t percent(uint32_t v) const {
    return static_cast<uint8_t>(((v & 65535u) * 100u + period_ / 2u) / period_);
  }
  uint32_t old_[2]{}, want_[2]{}, period_ = 0;
  uint64_t tx_ = 0;
  bool owned_ = false, blocked_ = false;
  devourer::BusySpoil spoil_ = devourer::BusySpoil::None;
};

} // namespace kestrel
