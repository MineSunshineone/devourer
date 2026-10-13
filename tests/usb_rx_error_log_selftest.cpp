// SPDX-License-Identifier: GPL-2.0-only
#include "UsbTransport.h"
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>
#include <thread>

namespace {
libusb_transfer *pending = nullptr;
unsigned completed = 0;
unsigned submitted = 0;
constexpr unsigned errors = 50000;
int payload_length = 0;
int failures = 0;
#define CHECK(ok) do { if (!(ok)) { std::fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #ok); ++failures; } } while (false)
}

extern "C" libusb_device *LIBUSB_CALL libusb_get_device(libusb_device_handle *) {
  return reinterpret_cast<libusb_device *>(1);
}
extern "C" int LIBUSB_CALL libusb_get_device_descriptor(
    libusb_device *, libusb_device_descriptor *) {
  return LIBUSB_ERROR_NOT_FOUND;
}
extern "C" int LIBUSB_CALL libusb_get_device_speed(libusb_device *) {
  return LIBUSB_SPEED_HIGH;
}
extern "C" int LIBUSB_CALL libusb_submit_transfer(libusb_transfer *transfer) {
  CHECK(pending == nullptr);
  if (pending) std::exit(2);
  pending = transfer;
  ++submitted;
  return 0;
}
extern "C" int LIBUSB_CALL libusb_cancel_transfer(libusb_transfer *) {
  CHECK(pending == nullptr);
  return LIBUSB_ERROR_NOT_FOUND;
}
extern "C" int LIBUSB_CALL libusb_handle_events_timeout_completed(
    libusb_context *, timeval *, int *) {
  CHECK(pending != nullptr);
  if (!pending) std::exit(2);
  auto *transfer = pending;
  pending = nullptr;
  if (payload_length && completed == errors / 2)
    std::this_thread::sleep_for(std::chrono::milliseconds(1100));
  transfer->status = completed++ < errors ? LIBUSB_TRANSFER_ERROR
                                          : LIBUSB_TRANSFER_NO_DEVICE;
  transfer->actual_length = transfer->status == LIBUSB_TRANSFER_ERROR ? payload_length : 0;
  transfer->buffer[0] = 0x5a;
  transfer->callback(transfer);
  return 0;
}

int main() {
  for (int length : {0, 1}) {
    payload_length = length;
    completed = submitted = 0;
    auto *file = std::tmpfile();
    if (!file) return 2;
    auto logger = std::make_shared<Logger>();
    logger->set_level(Logger::Level::Error);
    logger->set_diag_stream(file);
    unsigned delivered = 0;
    const auto start = std::chrono::steady_clock::now();
    {
      devourer::UsbTransport transport(
          reinterpret_cast<libusb_device_handle *>(1), logger, nullptr, {}, false);
      transport.rx_loop(1024, 1, [&](const uint8_t *data, int size) {
        CHECK(size == 1 && data[0] == 0x5a);
        ++delivered;
      }, [] { return false; });
    }
    const auto elapsed = std::chrono::steady_clock::now() - start;
    CHECK(completed == errors + 1 && submitted == errors + 1);
    CHECK(delivered == (length ? errors : 0));
    std::rewind(file);
    std::string text;
    char line[1024];
    unsigned repeated_logs = 0;
    while (std::fgets(line, sizeof(line), file)) {
      text += line;
      repeated_logs += std::string(line).find("error status=1 ") != std::string::npos;
    }
    std::fclose(file);
    CHECK(repeated_logs >= (length ? 2u : 1u));
    CHECK(repeated_logs <= 1 + std::chrono::duration_cast<std::chrono::seconds>(elapsed).count());
    CHECK(text.find("error status=5 ") != std::string::npos);
    CHECK(text.find("errors total=50000") != std::string::npos);
    std::printf("payload=%d completions=%u delivered=%u repeated_logs=%u elapsed_us=%lld\n",
                length, completed, delivered, repeated_logs,
                static_cast<long long>(std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count()));
  }
  return failures ? 1 : 0;
}
