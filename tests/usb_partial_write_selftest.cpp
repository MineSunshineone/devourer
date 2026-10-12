// SPDX-License-Identifier: GPL-2.0-only
#include "UsbTransport.h"

#include <atomic>
#include <barrier>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <thread>

namespace {
std::atomic<int> calls{0};
int first_rc = 0;
int first_actual = 0;
int failures = 0;
libusb_transfer *pending = nullptr;
unsigned async_calls = 0;
bool parallel_probe = false;
int probe_active = 0, probe_peak = 0, probe_entered = 0;
std::mutex probe_mutex;
std::condition_variable probe_ready;

#define CHECK(condition) do { if (!(condition)) { \
  std::fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #condition); \
  ++failures; } } while (0)

void scenario(int rc, int actual, bool poisoned, bool concurrent = false) {
  first_rc = rc;
  first_actual = actual;
  calls = 0;
  auto logger = std::make_shared<Logger>();
  logger->set_level(Logger::Level::Silent);
  devourer::UsbTransport transport(
      reinterpret_cast<libusb_device_handle *>(1), logger, nullptr, {}, true,
      devourer::RxMode::Async, 0, 0, devourer::PoolExhaust::Backpressure, true);
  unsigned char bytes[1000]{};
  int a = 0, b = 0;
  if (concurrent) {
    std::barrier start(2);
    std::thread worker([&] {
      start.arrive_and_wait();
      a = transport.tx_sync_data(8, bytes, sizeof(bytes), 500);
    });
    start.arrive_and_wait();
    b = transport.tx_sync_data(8, bytes, sizeof(bytes), 500);
    worker.join();
    CHECK(a < static_cast<int>(sizeof(bytes)) && b < static_cast<int>(sizeof(bytes)));
  } else {
    a = transport.tx_sync_data(8, bytes, sizeof(bytes), 500);
    b = transport.tx_sync_data(8, bytes, sizeof(bytes), 500);
    CHECK(a == (rc ? rc : actual));
    CHECK(poisoned ? b < 0 : b == static_cast<int>(sizeof(bytes)));
  }
  if (poisoned) {
    CHECK(transport.tx_sync_data(6, bytes, sizeof(bytes), 500) < 0);
    CHECK(!transport.tx_async(6, bytes, sizeof(bytes), 500));
  }
  CHECK(calls == (poisoned ? 1 : 2));
  CHECK(transport.tx_stats().needs_reinit == poisoned);
  CHECK(transport.tx_stats().submitted == static_cast<unsigned>(calls.load()));
  CHECK(transport.tx_stats().failed == static_cast<unsigned>(rc != 0 || actual < 1000));
  CHECK(transport.tx_stats().last_error_rc ==
        (rc ? rc : actual < 1000 ? devourer::kTxShortWriteRc : 0));
  CHECK(transport.tx_stats().last_was_timeout == (rc == LIBUSB_ERROR_TIMEOUT));
}

void endpoint_scenario(bool same_endpoint) {
  auto logger = std::make_shared<Logger>();
  logger->set_level(Logger::Level::Silent);
  devourer::UsbTransport transport(
      reinterpret_cast<libusb_device_handle *>(1), logger, nullptr, {}, true,
      devourer::RxMode::Async, 0, 0, devourer::PoolExhaust::Backpressure, true);
  parallel_probe = true;
  probe_active = probe_peak = probe_entered = 0;
  unsigned char bytes[1000]{};
  std::barrier start(2);
  int a = 0, b = 0;
  std::thread worker([&] {
    start.arrive_and_wait();
    a = transport.tx_sync_data(8, bytes, sizeof(bytes), 500);
  });
  start.arrive_and_wait();
  b = transport.tx_sync_data(same_endpoint ? 8 : 6, bytes, sizeof(bytes), 500);
  worker.join();
  parallel_probe = false;
  CHECK(a == 1000 && b == 1000);
  CHECK(probe_peak == (same_endpoint ? 1 : 2));
  CHECK(!transport.tx_stats().needs_reinit);
}
}  // namespace

// Exercise the real UsbTransport; replace only its libusb device I/O.
extern "C" libusb_device * LIBUSB_CALL libusb_get_device(libusb_device_handle *) {
  return reinterpret_cast<libusb_device *>(1);
}
extern "C" int LIBUSB_CALL libusb_get_device_descriptor(
    libusb_device *, libusb_device_descriptor *desc) {
  *desc = {};
  desc->idVendor = 0x0bda;
  desc->idProduct = 0xa81a;
  return 0;
}
extern "C" int LIBUSB_CALL libusb_get_device_speed(libusb_device *) {
  return LIBUSB_SPEED_HIGH;
}
extern "C" int LIBUSB_CALL libusb_clear_halt(libusb_device_handle *, unsigned char) {
  return 0;
}
extern "C" int LIBUSB_CALL libusb_handle_events_timeout_completed(
    libusb_context *, timeval *, int *) {
  return 0;
}
extern "C" int LIBUSB_CALL libusb_control_transfer(
    libusb_device_handle *, uint8_t, uint8_t, uint16_t, uint16_t,
    unsigned char *data, uint16_t length, unsigned) {
  std::memset(data, 0, length);
  return length;
}
extern "C" int LIBUSB_CALL libusb_submit_transfer(libusb_transfer *transfer) {
  CHECK(pending == nullptr);
  pending = transfer;
  ++async_calls;
  return 0;
}
extern "C" int LIBUSB_CALL libusb_bulk_transfer(
    libusb_device_handle *, unsigned char, unsigned char *, int length,
    int *actual, unsigned timeout) {
  CHECK(timeout == 0);  // Keep the production multi-packet no-cancel policy.
  if (parallel_probe) {
    std::unique_lock lock(probe_mutex);
    probe_peak = std::max(probe_peak, ++probe_active);
    ++probe_entered;
    probe_ready.notify_all();
    probe_ready.wait_for(lock, std::chrono::milliseconds(100), [] { return probe_entered == 2; });
    --probe_active;
    *actual = length;
    return 0;
  }
  if (calls.fetch_add(1) == 0) {
    *actual = first_actual;
    return first_rc;
  }
  *actual = length;
  return 0;
}

void complete(libusb_transfer_status status, int actual) {
  auto *transfer = pending;
  pending = nullptr;
  transfer->status = status;
  transfer->actual_length = actual;
  transfer->callback(transfer);
}

void async_scenario(libusb_transfer_status status, int actual, bool poisoned) {
  async_calls = 0;
  auto logger = std::make_shared<Logger>();
  logger->set_level(Logger::Level::Silent);
  devourer::UsbTransport transport(
      reinterpret_cast<libusb_device_handle *>(1), logger, nullptr, {}, true,
      devourer::RxMode::Async, 0, 0, devourer::PoolExhaust::Backpressure, true);
  unsigned char bytes[1000]{};
  CHECK(transport.tx_async(8, bytes, sizeof(bytes), 500));
  complete(status, actual);
  CHECK(transport.tx_stats().needs_reinit == poisoned);
  const bool accepted = transport.tx_async(8, bytes, sizeof(bytes), 500);
  CHECK(accepted == !poisoned);
  if (accepted) complete(LIBUSB_TRANSFER_COMPLETED, sizeof(bytes));
  CHECK(async_calls == (poisoned ? 1U : 2U));
  CHECK(transport.tx_stats().inflight == 0);
  if (poisoned) {
    const auto before = calls.load();
    CHECK(transport.tx_sync_data(8, bytes, sizeof(bytes), 500) < 0);
    CHECK(calls == before);
  }
  if (status == LIBUSB_TRANSFER_COMPLETED && actual != 1000)
    CHECK(transport.tx_stats().last_error_rc == devourer::kTxShortWriteRc);
}

int main() {
  endpoint_scenario(false);
  endpoint_scenario(true);
  scenario(LIBUSB_ERROR_IO, 512, true);
  scenario(LIBUSB_ERROR_TIMEOUT, 512, true);
  scenario(0, 512, true);
  scenario(LIBUSB_ERROR_IO, 0, false);
  scenario(0, 0, false);
  scenario(LIBUSB_ERROR_IO, 1000, false);
  scenario(0, 1000, false);
  scenario(LIBUSB_ERROR_IO, 512, true, true);
  async_scenario(LIBUSB_TRANSFER_COMPLETED, 512, true);
  async_scenario(LIBUSB_TRANSFER_ERROR, 512, true);
  async_scenario(LIBUSB_TRANSFER_TIMED_OUT, 512, true);
  async_scenario(LIBUSB_TRANSFER_STALL, 512, true);
  async_scenario(LIBUSB_TRANSFER_CANCELLED, 512, false);
  async_scenario(LIBUSB_TRANSFER_COMPLETED, 0, false);
  async_scenario(LIBUSB_TRANSFER_ERROR, 0, false);
  async_scenario(LIBUSB_TRANSFER_ERROR, 1000, false);
  async_scenario(LIBUSB_TRANSFER_COMPLETED, 1000, false);
  if (!failures) std::puts("PASS: partial writes stop later submissions; complete/zero-byte errors retain retry behavior");
  return failures ? 1 : 0;
}
