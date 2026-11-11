// SPDX-License-Identifier: GPL-2.0-only
#include "DeviceSession.h"
#include "RadiotapBuilder.h"
#include "SignalStop.h"
#include "RxPacket.h"
#include "WiFiDriver.h"
#include "env_config.h"
#include "usb_select.h"
#include <algorithm>
#include <array>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <thread>
#include <unordered_set>

using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;

int main(int argc, char **argv) {
  if (argc != 5 || (std::strcmp(argv[1], "rx") && std::strcmp(argv[1], "txoff") &&
                    std::strcmp(argv[1], "txon") && std::strcmp(argv[1], "txlow") &&
                    std::strcmp(argv[1], "txread"))) {
    std::fprintf(stderr, "bfbench rx|txoff|txread|txon|txlow seconds Mbps run_id\n"); return 2;
  }
  install_devourer_signal_handlers();
  const bool tx = std::strcmp(argv[1], "rx") != 0;
  const bool low_power = std::strcmp(argv[1], "txlow") == 0;
  const bool apply = std::strcmp(argv[1], "txon") == 0 || low_power;
  unsigned seconds, run;
  double mbps;
  try {
    seconds = std::stoul(argv[2]); mbps = std::stod(argv[3]); run = std::stoul(argv[4]);
  } catch (...) { return 2; }
  if (!seconds || seconds > 600 || !(mbps > 0 && mbps <= 50)) return 2;
  const devourer::MacAddr sender{{2, 0x42, 0x58, 0x49, 0, 0x11}};
  const devourer::MacAddr receiver{{2, 0x42, 0x58, 0x49, 0, 0x12}};
  const auto self = tx ? sender : receiver, peer = tx ? receiver : sender;
  auto logger = std::make_shared<Logger>();
  apply_logging_env(*logger);
  devourer::DeviceSession session(logger);
  libusb_context *ctx = nullptr;
  if (libusb_init(&ctx)) return 1;
  session.adopt_context(ctx);
  const uint16_t pids[]{0xa81a};
  auto *handle = open_selected_usb(ctx, logger, pids, 1);
  if (!handle) return 1;
  session.adopt_handle(handle);
  std::shared_ptr<devourer::UsbDeviceLock> lock;
  if (devourer::claim_interface_then_reset(handle, devourer::find_wifi_interface(handle),
                                          logger, false, lock)) return 1;
  session.adopt_lock(lock);
  auto cfg = devourer_config_from_env();
  cfg.rx.enable_with_tx = true;
  cfg.rx.ack_responder = self;
  cfg.bf = {};
  if (tx) {
    cfg.bf.arm_sounder = true;
    cfg.bf.sounder_self_mac = self;
    cfg.bf.txbf_peer = peer;
  } else cfg.bf.beamformee_of = peer;
  WiFiDriver driver(logger);
  session.adopt_device(driver.CreateRadio(handle, ctx, lock, cfg));
  auto *radio = session.device();
  if (!radio) return 1;
  radio->InitWrite({.Channel = 153, .ChannelOffset = 0, .ChannelWidth = CHANNEL_WIDTH_20});
  const auto mode = devourer::parse_tx_mode_str("VHT1SS_MCS7/LDPC/20/SGI");
  radio->SetTxMode(mode);
  // Keep both calibrated references above the rail; nominal qdB is not measured RF power.
  if (tx) {
    const int power_offset = low_power ? -48 : -24;
    if (radio->SetTxPowerOffsetQdb(power_offset) != power_offset) return 1;
    const auto power = radio->GetTxPowerState();
    std::printf("BF_POWER offset=%d mcs7_ref=%d low=%d high=%d readback=%d\n",
      power.offset_qdb,power.mcs7_index,power.saturated_low,power.saturated_high,power.hw_readback);
    if (!power.valid || !power.hw_readback || power.saturated_low || power.saturated_high || power.mcs7_index <= 0)
      return 1;
  }
  std::mutex mu;
  std::unordered_set<uint64_t> seen;
  uint64_t bytes = 0, physts = 0, beamformed = 0, ndpas = 0, crc = 0;
  struct Signal { uint64_t power_n=0, full_n=0; int64_t pwdb_a=0,pwdb_b=0,snr_a=0,snr_b=0,evm=0; };
  std::array<Signal,2> signal{};
  uint64_t reports = 0, failures = 0, retries = 0;
  int64_t rssi_a = 0, rssi_b = 0, snr_a = 0, snr_b = 0;
  uint32_t rxrun = 0;
  const auto emit_signal = [&] {
    for (unsigned i=0; i<signal.size(); ++i) {
      const auto &v=signal[i];
      std::printf("BF_PHY run=%u bf=%u power_n=%llu full_n=%llu pwdb_a=%lld pwdb_b=%lld snr_a=%lld snr_b=%lld evm=%lld\n",
        rxrun,i,(unsigned long long)v.power_n,(unsigned long long)v.full_n,
        (long long)v.pwdb_a,(long long)v.pwdb_b,(long long)v.snr_a,(long long)v.snr_b,(long long)v.evm);
    }
    std::fflush(stdout);
  };
  std::atomic<bool> rxfailed{false};
  radio->SetTxReportSink([&](const devourer::TxReport &r) {
    std::lock_guard guard(mu); ++reports; failures += r.state != 0; retries += r.data_retries;
  });
  std::thread rx([&] {
    try {
      radio->StartRxLoop([&](const Packet &p) {
        const auto d = p.Data;
        std::lock_guard guard(mu);
        if (d.size() == 23 && d[0] == 0x54) ++ndpas;
        const size_t header = d.size() && d[0] == 0x88 ? 26 : 24;
        if (d.size() < header + 16 || (d[0] != 0x08 && d[0] != 0x88) ||
            std::memcmp(d.data() + 4, self.data(), 6) ||
            std::memcmp(d.data() + 10, peer.data(), 6) ||
            std::memcmp(d.data() + header, "BXBF", 4)) return;
        if (p.RxAtrib.crc_err || p.RxAtrib.icv_err) { ++crc; return; }
        uint32_t id; uint64_t seq;
        std::memcpy(&id, d.data() + header + 4, 4); std::memcpy(&seq, d.data() + header + 8, 8);
        if (rxrun != id) {
          if (rxrun) emit_signal();
          rxrun = id; seen.clear(); bytes = physts = beamformed = 0;
          rssi_a = rssi_b = snr_a = snr_b = 0; signal = {};
        }
        if (!seen.insert(seq).second) return;
        bytes += d.size() - header - (p.RxAtrib.fcs_present ? 4 : 0);
        if (p.RxAtrib.phy_fill != PhyStsFill::None) {
          ++physts; beamformed += p.RxAtrib.beamformed;
          rssi_a += p.RxAtrib.rssi[0]; rssi_b += p.RxAtrib.rssi[1];
          auto &v = signal[p.RxAtrib.beamformed];
          ++v.power_n; v.pwdb_a += p.RxAtrib.rssi[0]; v.pwdb_b += p.RxAtrib.rssi[1];
          if (p.RxAtrib.phy_fill == PhyStsFill::Full) {
            snr_a += p.RxAtrib.snr[0]; snr_b += p.RxAtrib.snr[1];
            ++v.full_n; v.snr_a += p.RxAtrib.snr[0]; v.snr_b += p.RxAtrib.snr[1];
            v.evm += p.RxAtrib.evm[0];
          }
        }
      });
    } catch (const std::exception &e) { std::fprintf(stderr, "RX ERROR %s\n", e.what()); rxfailed = true; }
  });
  std::printf("BF_READY role=%s run=%u\n", argv[1], run); std::fflush(stdout);
  auto frame = devourer::build_stream_radiotap(mode);
  const size_t offset = frame.size();
  constexpr size_t payload = 1100;
  constexpr size_t header = 26;
  frame.resize(offset + header + payload, 0);
  auto *d = frame.data() + offset;
  d[0] = 0x88;
  std::memcpy(d + 4, peer.data(), 6); std::memcpy(d + 10, self.data(), 6);
  std::memcpy(d + 16, peer.data(), 6); std::memcpy(d + header, "BXBF", 4);
  std::memcpy(d + header + 4, &run, 4);
  d[24] = cfg.tx.ampdu ? cfg.tx.ampdu->tid : 0;
  std::array<std::vector<uint8_t>, 64> batch;
  for (auto &f : batch) f = frame;
  std::array<TxPacketView, 64> views{};
  uint64_t submitted = 0, soundings = 0, active_ticks = 0;
  const auto start = Clock::now(), end = start + std::chrono::seconds(seconds);
  auto next = start + 1ms, next_sounding = start, next_poll = start, next_log = start + 1s;
  auto next_read = start;
  uint64_t send_us=0, send_max_us=0;
  const auto spacing = std::chrono::nanoseconds(static_cast<int64_t>(payload * 8000.0 / mbps));
  unsigned token = 0;
  int result = 0;
  try {
    while (Clock::now() < end && !rxfailed && !g_devourer_should_stop) {
      auto now = Clock::now();
      if (tx && now >= next_sounding && now + 2s < end) {
        if (!radio->SendBeamformingSounding(++token)) throw std::runtime_error("sounding USB submission failed");
        ++soundings; next_sounding = now + 100ms;
      }
      if (tx && now >= next_poll) {
        if (apply) radio->PollBeamforming(500);
        active_ticks += radio->GetBeamformingStatus().active;
        next_poll = now + 10ms;
      }
      if (tx && std::strcmp(argv[1], "txread") == 0 && now >= next_read) {
        uint8_t control[4]{};
        if (libusb_control_transfer(handle, 0xc0, 5, 0x071c, 0, control, 4, 1000) != 4)
          throw std::runtime_error("diagnostic CSI register read failed");
        next_read = now + 100ms;
      }
      if (tx && now >= next) {
        const auto target = static_cast<uint64_t>((now - start) / spacing);
        const size_t count = static_cast<size_t>(std::min<uint64_t>(batch.size(), target > submitted ? target-submitted : 0));
        for (size_t i=0; i<count; ++i) {
          const auto seq = submitted+i;
          std::memcpy(batch[i].data()+offset+header+8, &seq, 8);
          views[i] = {batch[i].data(), batch[i].size()};
        }
        const auto before_send = Clock::now();
        const auto sent = count ? radio->send_packets(views.data(), count) : 0;
        const auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(Clock::now()-before_send).count();
        send_us += elapsed; send_max_us = std::max<uint64_t>(send_max_us, elapsed);
        submitted += sent;
        if (sent != count) throw std::runtime_error("data USB batch incomplete");
        next = count == batch.size() ? now : now + 1ms;
      } else std::this_thread::sleep_for(100us);
      if (now >= next_log) {
        const auto bf = radio->GetBeamformingStatus();
        std::lock_guard guard(mu);
        std::printf("BF_STATS role=%s elapsed_ms=%lld submitted=%llu sounding=%llu active=%d active_ticks=%llu cbr=%llu rejected=%llu age_ms=%llu rxrun=%u unique=%zu bytes=%llu physts=%llu beamformed=%llu rssi_a=%lld rssi_b=%lld snr_a=%lld snr_b=%lld ndpa=%llu crc=%llu reports=%llu failed=%llu retries=%llu\n",
          argv[1], (long long)std::chrono::duration_cast<std::chrono::milliseconds>(now-start).count(),
          (unsigned long long)submitted, (unsigned long long)soundings, bf.active, (unsigned long long)active_ticks,
          (unsigned long long)bf.feedback_reports, (unsigned long long)bf.rejected_reports,
          (unsigned long long)bf.feedback_age_ms, rxrun, seen.size(), (unsigned long long)bytes,
          (unsigned long long)physts, (unsigned long long)beamformed,
          (long long)rssi_a, (long long)rssi_b, (long long)snr_a, (long long)snr_b,
          (unsigned long long)ndpas, (unsigned long long)crc,
          (unsigned long long)reports, (unsigned long long)failures, (unsigned long long)retries);
        std::fflush(stdout); next_log = now + 1s;
      }
    }
    if (rxfailed) result = 1;
    if (tx) {
      radio->ClearBeamforming();
      std::printf("BF_CLEAR active=%d\n", radio->GetBeamformingStatus().active);
    }
  } catch (const std::exception &e) {
    std::fprintf(stderr, "BF ERROR %s\n", e.what()); result = 1;
    try { radio->ClearBeamforming(); } catch (...) {}
  }
  radio->StopRxLoop(); rx.join();
  radio->SetTxReportSink({});
  const auto ts = radio->GetTxStats();
  std::printf("BF_USB submitted=%llu failed=%llu rc=%d timeout=%d needs_reinit=%d send_us=%llu max_batch_us=%llu\n",
    (unsigned long long)ts.submitted,(unsigned long long)ts.failed,ts.last_error_rc,ts.last_was_timeout,ts.needs_reinit,
    (unsigned long long)send_us,(unsigned long long)send_max_us);
  emit_signal();
  std::printf("BF_DONE role=%s submitted=%llu result=%d\n", argv[1], (unsigned long long)submitted, result);
  return result;
}
