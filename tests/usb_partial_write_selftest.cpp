// SPDX-License-Identifier: GPL-2.0-only
#include "UsbTransport.h"

#include <atomic>
#include <barrier>
#include <cstdio>
#include <thread>

namespace {
std::atomic<int> calls{0};
int first_rc = 0;
int first_actual = 0;
int failures = 0;

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
  CHECK(calls == (poisoned ? 1 : 2));
  CHECK(transport.tx_stats().needs_reinit == poisoned);
  CHECK(transport.tx_stats().submitted == static_cast<unsigned>(calls.load()));
  CHECK(transport.tx_stats().failed == static_cast<unsigned>(rc != 0 || actual < 1000));
  CHECK(transport.tx_stats().last_error_rc ==
        (rc ? rc : actual < 1000 ? devourer::kTxShortWriteRc : 0));
  CHECK(transport.tx_stats().last_was_timeout == (rc == LIBUSB_ERROR_TIMEOUT));
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
extern "C" int LIBUSB_CALL libusb_bulk_transfer(
    libusb_device_handle *, unsigned char, unsigned char *, int length,
    int *actual, unsigned timeout) {
  CHECK(timeout == 0);  // Keep the production multi-packet no-cancel policy.
  if (calls.fetch_add(1) == 0) {
    *actual = first_actual;
    return first_rc;
  }
  *actual = length;
  return 0;
}

int main() {
  scenario(LIBUSB_ERROR_IO, 512, true);
  scenario(LIBUSB_ERROR_TIMEOUT, 512, true);
  scenario(0, 512, true);
  scenario(LIBUSB_ERROR_IO, 0, false);
  scenario(0, 0, false);
  scenario(LIBUSB_ERROR_IO, 1000, false);
  scenario(0, 1000, false);
  scenario(LIBUSB_ERROR_IO, 512, true, true);
  if (!failures) std::puts("PASS: partial writes stop later submissions; complete/zero-byte errors retain retry behavior");
  return failures ? 1 : 0;
}
