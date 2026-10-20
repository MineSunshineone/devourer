// SPDX-License-Identifier: GPL-2.0-only
#include "DeviceSession.h"
#include "RadiotapBuilder.h"
#include "SignalStop.h"
#include "RxPacket.h"
#include "VhtSounding.h"
#include "WiFiDriver.h"
#include "env_config.h"
#include "usb_select.h"
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
                    std::strcmp(argv[1], "txon"))) {
    std::fprintf(stderr, "bfbench rx|txoff|txon seconds Mbps run_id\n"); return 2;
  }
  install_devourer_signal_handlers();
  const bool tx = std::strcmp(argv[1], "rx") != 0;
  const bool apply = std::strcmp(argv[1], "txon") == 0;
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
  // Equal calibrated backoff in both arms, to avoid near-field RX saturation.
  if (tx && !radio->SetTxPowerOffsetQdb(-80)) return 1;
  std::mutex mu;
  std::unordered_set<uint64_t> seen;
  uint64_t bytes = 0, physts = 0, beamformed = 0, ndpas = 0, crc = 0;
  uint64_t reports = 0, failures = 0, retries = 0;
  int64_t rssi_a = 0, rssi_b = 0, snr_a = 0, snr_b = 0;
  uint32_t rxrun = 0;
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
        if (d.size() < 40 || d[0] != 0x08 ||
            std::memcmp(d.data() + 4, self.data(), 6) ||
            std::memcmp(d.data() + 10, peer.data(), 6) ||
            std::memcmp(d.data() + 24, "BXBF", 4)) return;
        if (p.RxAtrib.crc_err || p.RxAtrib.icv_err) { ++crc; return; }
        uint32_t id; uint64_t seq;
        std::memcpy(&id, d.data() + 28, 4); std::memcpy(&seq, d.data() + 32, 8);
        if (rxrun != id) {
          rxrun = id; seen.clear(); bytes = physts = beamformed = 0;
          rssi_a = rssi_b = snr_a = snr_b = 0;
        }
        if (!seen.insert(seq).second) return;
        bytes += d.size() - 24 - (p.RxAtrib.fcs_present ? 4 : 0);
        if (p.RxAtrib.physt) {
          ++physts; beamformed += p.RxAtrib.beamformed;
          rssi_a += p.RxAtrib.rssi[0]; rssi_b += p.RxAtrib.rssi[1];
          snr_a += p.RxAtrib.snr[0]; snr_b += p.RxAtrib.snr[1];
        }
      });
    } catch (const std::exception &e) { std::fprintf(stderr, "RX ERROR %s\n", e.what()); rxfailed = true; }
  });
  std::printf("BF_READY role=%s run=%u\n", argv[1], run); std::fflush(stdout);
  auto frame = devourer::build_stream_radiotap(mode);
  const size_t offset = frame.size();
  constexpr size_t payload = 1100;
  frame.resize(offset + 24 + payload, 0);
  auto *d = frame.data() + offset;
  d[0] = 0x08;
  std::memcpy(d + 4, peer.data(), 6); std::memcpy(d + 10, self.data(), 6);
  std::memcpy(d + 16, self.data(), 6); std::memcpy(d + 24, "BXBF", 4);
  std::memcpy(d + 28, &run, 4);
  uint64_t submitted = 0, soundings = 0, active_ticks = 0;
  const auto start = Clock::now(), end = start + std::chrono::seconds(seconds);
  auto next = start, next_sounding = start, next_poll = start, next_log = start + 1s;
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
      if (tx && now >= next) {
        std::memcpy(d + 32, &submitted, 8);
        if (!radio->send_packet(frame.data(), frame.size())) throw std::runtime_error("data USB submission failed");
        ++submitted; next += spacing;
        if (now - next > 100ms) next = now;
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
  std::printf("BF_DONE role=%s submitted=%llu result=%d\n", argv[1], (unsigned long long)submitted, result);
  return result;
}
